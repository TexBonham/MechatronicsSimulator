"""Load-case export for FEA (FAC-013; the milestone 4 consumer, INT-032): per member end, the peak force and the
peak moment over the run with the time, the member-frame location (x = 0 at the start, L at the end) and the unit
direction at that time, as CSV and JSON; the design name and version ride with it (INT-033). Also writes the
integrity report and the load case beside a run archive (INT-022)."""

from __future__ import annotations

import csv
import io
import json
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from mechsim.integrity.plugin import LOADS_NOT_EVALUATED, IntegrityError
from mechsim.integrity.table import IntegrityReport
from mechsim.testing.driver import IntegrityResults, MemberSection, RecordView

_SERIES = (
    ("start", "force", "physics/members/start_force", "N"),
    ("start", "moment", "physics/members/start_moment", "N m"),
    ("end", "force", "physics/members/end_force", "N"),
    ("end", "moment", "physics/members/end_moment", "N m"),
)
CSV_HEADER = ("member", "end", "x", "quantity", "unit", "time", "magnitude", "dx", "dy", "dz")
FRAME = "member frame: x from the start to the end node, y and z the section's principal axes"


@dataclass(frozen=True)
class LoadEntry:
    member: str
    end: str
    x: float
    quantity: str
    unit: str
    time: float
    magnitude: float
    direction: tuple[float, float, float]


@dataclass(frozen=True)
class LoadCase:
    design_name: str
    design_version: str
    run_id: str
    entries: tuple[LoadEntry, ...]

    def to_json(self) -> str:
        data: dict[str, Any] = {
            "design": {"name": self.design_name, "version": self.design_version},
            "frame": FRAME,
            "loads": [
                {
                    "direction": list(entry.direction),
                    "end": entry.end,
                    "magnitude": entry.magnitude,
                    "member": entry.member,
                    "quantity": entry.quantity,
                    "time": entry.time,
                    "unit": entry.unit,
                    "x": entry.x,
                }
                for entry in self.entries
            ],
            "run_id": self.run_id,
        }
        return json.dumps(data, sort_keys=True, indent=1)

    def to_csv(self) -> str:
        buffer = io.StringIO()
        buffer.write(f"# design {self.design_name} {self.design_version}; run {self.run_id}; {FRAME}\n")
        writer = csv.writer(buffer, lineterminator="\n")
        writer.writerow(CSV_HEADER)
        for entry in self.entries:
            writer.writerow(
                (
                    entry.member,
                    entry.end,
                    repr(entry.x),
                    entry.quantity,
                    entry.unit,
                    repr(entry.time),
                    repr(entry.magnitude),
                    *(repr(value) for value in entry.direction),
                )
            )
        return buffer.getvalue()


def load_case(
    recording: RecordView, members: Sequence[MemberSection], design_name: str, design_version: str, run_id: str
) -> LoadCase:
    raw = getattr(recording, "recording", None)
    if raw is None:
        raise IntegrityError("the recording carries no member end-loads")
    times = np.asarray(recording.times(), dtype=np.float64)
    entries: list[LoadEntry] = []
    for member in members:
        for end, quantity, path, unit in _SERIES:
            if path not in raw.series:
                raise IntegrityError(f"the recording has no {path}")
            stored = raw.series[path]
            names = tuple(stored.names)
            if member.name not in names:
                raise IntegrityError(f"member {member.name!r}: no recorded end-loads in {path}")
            values = np.asarray(stored.values, dtype=np.float64)[:, names.index(member.name), :]
            magnitude = np.linalg.norm(values, axis=1)
            direction = (0.0, 0.0, 0.0)
            time = 0.0
            peak = 0.0
            if magnitude.size:
                index = int(np.argmax(magnitude))
                peak = float(magnitude[index])
                time = float(times[index])
                if peak > 0.0:
                    unit_vector = values[index] / peak
                    direction = (float(unit_vector[0]), float(unit_vector[1]), float(unit_vector[2]))
            x = 0.0
            if end == "end":
                x = float(member.length)
            entries.append(LoadEntry(member.name, end, x, quantity, unit, time, peak, direction))
    return LoadCase(design_name, design_version, run_id, tuple(entries))


def beside(archive: Path, suffix: str) -> Path:
    return archive.with_name(f"{archive.stem}{suffix}")


def write_beside(archive: Path, results: IntegrityResults, recording: RecordView, metadata: Any) -> tuple[Path, ...]:
    """`<run>.integrity.json` (the report when `results` is one, else the table) and `<run>.load_case.json/.csv` next
    to the archive (members built rigid, flagged not evaluated, are left out of the load case); returns the paths
    written."""
    text = results.to_json()
    if isinstance(results, IntegrityReport):
        text = results.report_json()
    table = beside(archive, ".integrity.json")
    table.write_text(text + "\n", encoding="utf-8")
    written = [table]
    rigid = {row.member for row in results.members if LOADS_NOT_EVALUATED in row.flags}
    members = tuple(member for member in metadata.members if member.name not in rigid)
    if members:
        case = load_case(recording, members, metadata.design_name, metadata.design_version, metadata.run_id)
        for suffix, body in ((".load_case.json", case.to_json()), (".load_case.csv", case.to_csv())):
            path = beside(archive, suffix)
            if not body.endswith("\n"):
                body += "\n"
            path.write_text(body, encoding="utf-8")
            written.append(path)
    return tuple(written)
