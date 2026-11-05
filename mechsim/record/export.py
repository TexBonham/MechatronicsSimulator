"""Exports of recorded series: CSV (REC-015), JSON with the events (REC-016), MAT (REC-017).

Every export names series by their archive path and states each unit (REC-018). A series set shares one axis
(per tick, or per contact); the axis's time columns come first in every export (REC-011).
"""

from __future__ import annotations

import csv
import json
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np
import scipy.io  # type: ignore[import-untyped]
from numpy.typing import NDArray

from mechsim.record.archive import RunArchive
from mechsim.record.layout import AXIS_TIME, TICK_AXIS, Series
from mechsim.record.tick import RecordError

MAT_FORMAT = "5"


def series_set(archive: RunArchive, names: Sequence[str]) -> dict[str, Series]:
    """The requested series, axis time first, each once, in request order; all on one axis."""
    axis = TICK_AXIS
    if len(names) > 0:
        axis = archive.series(names[0]).axis
    ordered: list[str] = []
    for name in (*AXIS_TIME[axis], *names):
        if name not in ordered:
            ordered.append(name)
    chosen = {name: archive.series(name) for name in ordered}
    for name, series in chosen.items():
        if series.axis != axis:
            raise RecordError(f"series {name!r} is on the {series.axis} axis, the set is on the {axis} axis")
    return chosen


def columns(name: str, series: Series) -> list[tuple[str, str, NDArray[Any]]]:
    """One (column name, unit, values) per scalar column; a multi-dimensional series yields `name[i,j]`."""
    values = series.values
    if values.ndim == 1:
        return [(name, series.unit, values)]
    tail = values.shape[1:]
    flat = values.reshape(values.shape[0], int(np.prod(tail)))
    result: list[tuple[str, str, NDArray[Any]]] = []
    for column in range(flat.shape[1]):
        index = ",".join(str(int(part)) for part in np.unravel_index(column, tail))
        result.append((f"{name}[{index}]", series.unit, flat[:, column]))
    return result


def _cell(value: Any) -> str:
    if isinstance(value, bool):
        return str(int(value))
    return repr(value)


def export_csv(archive: RunArchive, path: Path, names: Sequence[str]) -> None:
    """One row per axis entry; header cells `name [unit]`; floats in shortest round-trip form."""
    table = [column for name, series in series_set(archive, names).items() for column in columns(name, series)]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow([f"{name} [{unit}]" for name, unit, _ in table])
        lists = [values.tolist() for _, _, values in table]
        rows = 0
        if len(lists) > 0:
            rows = len(lists[0])
        for row in range(rows):
            writer.writerow([_cell(values[row]) for values in lists])


def export_json(archive: RunArchive, path: Path, names: Sequence[str]) -> None:
    """Identity, run end, every event and the requested series with units; NaN or infinity is an error."""
    identity = archive.identity
    end = archive.end
    fault: dict[str, Any] | None = None
    if end.fault is not None:
        fault = {"party": end.fault.party, "time": end.fault.time, "cause": end.fault.cause}
    document = {
        "identity": {
            "design_name": identity.design_name,
            "design_version": identity.design_version,
            "design_source_hash": identity.design_source_hash,
            "test_case_id": identity.test_case_id,
            "test_case_hash": identity.test_case_hash,
            "seed": identity.seed,
            "controller_mode": identity.controller_mode,
            "base_rate_hz": [archive.base_rate.numerator, archive.base_rate.denominator],
            "physics_rate_hz": [identity.physics_rate.numerator, identity.physics_rate.denominator],
            "control_rate_hz": [identity.control_rate.numerator, identity.control_rate.denominator],
            "repository_commit": identity.repository_commit,
            "tool_version": identity.tool_version,
        },
        "end": {"outcome": end.outcome, "end_time": end.end_time, "ticks": end.ticks, "fault": fault},
        "events": [
            {
                "row": row,
                "tick": event.time,
                "seconds": seconds,
                "source": event.source,
                "kind": event.kind,
                "detail": event.detail,
            }
            for (row, event), seconds in zip(archive.events(), archive.events_table()["seconds"].tolist(), strict=True)
        ],
        "series": {
            name: {"unit": series.unit, "axis": series.axis, "values": series.values.tolist()}
            for name, series in series_set(archive, names).items()
        },
    }
    try:
        text = json.dumps(document, allow_nan=False, ensure_ascii=False)
    except ValueError as error:
        raise RecordError(f"JSON export of {path.name}: {error}") from error
    path.write_text(text, encoding="utf-8")


def export_mat(archive: RunArchive, path: Path, names: Sequence[str]) -> None:
    """MAT version 5: 1 x n cell arrays `series_names`, `series_units`, `series_values`, archive names verbatim."""
    chosen = series_set(archive, names)
    count = len(chosen)
    names_cell = np.empty((1, count), dtype=object)
    units_cell = np.empty((1, count), dtype=object)
    values_cell = np.empty((1, count), dtype=object)
    for position, (name, series) in enumerate(chosen.items()):
        names_cell[0, position] = name
        units_cell[0, position] = series.unit
        values_cell[0, position] = series.values
    scipy.io.savemat(
        path,
        {"series_names": names_cell, "series_units": units_cell, "series_values": values_cell},
        format=MAT_FORMAT,
        do_compression=False,
        oned_as="column",
    )
