"""The in-memory fake driver, builder, archiver and archive reader the runner, scoring and CLI tests share.

`InMemoryFakeDriver` plays a scripted recording: at each base tick n it asks the script for that row, delivers the
operator commands scheduled for n as operator-message events, and stops at the end condition, or with a run error at
`error_tick` (no row for the failing tick, as the kernel does).
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from fractions import Fraction
from pathlib import Path
from typing import Any

from mechsim.design.schema import Design
from mechsim.testing.driver import (
    MESSAGE_KIND,
    BuildResult,
    EndSpec,
    RecordedEvent,
    RunConfig,
    RunMetadata,
    RunReport,
    ScheduledCommand,
    ScheduledFault,
    Series,
)
from mechsim.testing.schema import Case, InitialState

ROOT = Path(__file__).resolve().parents[2]
PENDULUM = ROOT / "designs" / "pendulum" / "design.yaml"


@dataclass
class Row:
    series: dict[str, float]
    state: str = ""
    events: list[RecordedEvent] = field(default_factory=list)


Script = Callable[[int, float], Row]


def still(n: int, t: float) -> Row:
    return Row({})


class FakeRecording:
    def __init__(self, times: Sequence[float], rows: Sequence[Row], units: Mapping[str, str]) -> None:
        self._times = list(times)
        self._rows = list(rows)
        self._units = dict(units)
        self.validity: tuple[Any, ...] = ()

    def times(self) -> Sequence[float]:
        return self._times

    def series(self, reference: str) -> Series:
        if len(self._rows) == 0 or reference not in self._rows[0].series:
            raise KeyError(reference)
        values = tuple(row.series[reference] for row in self._rows)
        return Series(tuple(self._times), values, self._units.get(reference, ""))

    def controller_states(self) -> Sequence[str]:
        return [row.state for row in self._rows]

    def events(self) -> Sequence[RecordedEvent]:
        return [event for row in self._rows for event in row.events]


def recording_from(times: Sequence[float], series: Mapping[str, Sequence[float]], unit: str = "") -> FakeRecording:
    rows = [Row({name: values[index] for name, values in series.items()}) for index in range(len(times))]
    return FakeRecording(times, rows, dict.fromkeys(series, unit))


class InMemoryFakeDriver:
    def __init__(
        self,
        script: Script = still,
        units: Mapping[str, str] | None = None,
        reported: Mapping[str, bool | str] | None = None,
        error_tick: int | None = None,
        error: str = "physics: solver did not converge",
    ) -> None:
        self.script = script
        self.units = dict(units or {})
        self.reported = dict(reported or {})
        self.error_tick = error_tick
        self.error = error
        self.calls: list[str] = []
        self.config: RunConfig | None = None
        self.initial: InitialState | None = None
        self.commands: list[ScheduledCommand] = []
        self.faults: list[ScheduledFault] = []
        self.case: Case | None = None
        self.end: EndSpec | None = None
        self._times: list[float] = []
        self._rows: list[Row] = []

    def load(self, build: BuildResult, case: Case) -> None:
        self.calls.append("load")
        self.case = case

    def reported_controller_state(self) -> Mapping[str, bool | str]:
        self.calls.append("reported_controller_state")
        return self.reported

    def configure(self, config: RunConfig) -> None:
        self.calls.append("configure")
        self.config = config

    def set_initial_state(self, state: InitialState) -> None:
        self.calls.append("set_initial_state")
        self.initial = state

    def schedule_command(self, command: ScheduledCommand) -> None:
        self.calls.append("schedule_command")
        self.commands.append(command)

    def schedule_fault(self, fault: ScheduledFault) -> None:
        self.calls.append("schedule_fault")
        self.faults.append(fault)

    def run(self, end: EndSpec) -> RunReport:
        self.calls.append("run")
        self.end = end
        assert self.config is not None
        base = self.config.base_rate
        for n in range(end.max_tick + 1):
            if self.error_tick is not None and n == self.error_tick:
                return RunReport("run_error", n, base, self.error)
            t = float(Fraction(n) / base)
            row = self.script(n, t)
            for command in self.commands:
                if command.tick == n:
                    row.events.append(RecordedEvent(t, "operator", MESSAGE_KIND, command.message))
            self._times.append(t)
            self._rows.append(row)
            if end.at_tick is not None and n == end.at_tick:
                return RunReport("end_time", n, base)
            if end.message is not None:
                for event in row.events:
                    if event.kind == MESSAGE_KIND and event.detail == end.message:
                        return RunReport("message", n, base)
            if end.controller_state is not None and row.state == end.controller_state:
                return RunReport("controller_state", n, base)
        return RunReport("max_duration", end.max_tick, base)

    def recording(self) -> FakeRecording:
        return FakeRecording(self._times, self._rows, self.units)


@dataclass(frozen=True)
class FakeBuild:
    design: Design
    module_instances: tuple[str, ...]


class FakeBuilder:
    def __init__(self, instances: Sequence[str] = ("physics", "sensors", "controller")) -> None:
        self.instances = tuple(instances)
        self.built: list[Design] = []

    def __call__(self, design: Design, adapter: str = "mujoco") -> FakeBuild:
        self.built.append(design)
        return FakeBuild(design, self.instances)


class FakeArchiver:
    """Writes a JSON stand-in for the HDF5 archive; the digest is the file's sha256."""

    def __init__(self) -> None:
        self.written: list[tuple[Path, RunMetadata]] = []

    def write(self, path: Path, recording: FakeRecording, metadata: RunMetadata) -> str:  # type: ignore[override]
        rows = []
        for time, event_row in zip(recording.times(), recording._rows, strict=True):
            rows.append({"time": time, "series": event_row.series, "state": event_row.state})
        events = [[event.time, event.source, event.kind, event.detail] for event in recording.events()]
        report = metadata.report
        data = {
            "metadata": {
                "run_id": metadata.run_id,
                "case_id": metadata.case_id,
                "design_name": metadata.design_name,
                "design_version": metadata.design_version,
                "design_hash": metadata.design_hash,
                "case_hash": metadata.case_hash,
                "seed": metadata.seed,
                "controller_mode": metadata.controller_mode,
                "reason": report.reason,
                "end_tick": report.end_tick,
                "base_rate": str(report.base_rate),
                "error": report.error,
            },
            "units": recording._units,
            "rows": rows,
            "events": events,
        }
        text = json.dumps(data, sort_keys=True)
        path.write_text(text, encoding="utf-8")
        self.written.append((path, metadata))
        return hashlib.sha256(text.encode()).hexdigest()


class FakeArchiveReader:
    def open(self, path: Path) -> tuple[RunMetadata, FakeRecording]:
        data = json.loads(path.read_text(encoding="utf-8"))
        meta = data["metadata"]
        report = RunReport(meta["reason"], meta["end_tick"], Fraction(meta["base_rate"]), meta["error"])
        metadata = RunMetadata(
            run_id=meta["run_id"],
            case_id=meta["case_id"],
            design_name=meta["design_name"],
            design_version=meta["design_version"],
            design_hash=meta["design_hash"],
            case_hash=meta["case_hash"],
            seed=meta["seed"],
            controller_mode=meta["controller_mode"],
            sweep=None,
            report=report,
        )
        by_time: dict[float, list[RecordedEvent]] = {}
        for time, source, kind, detail in data["events"]:
            by_time.setdefault(time, []).append(RecordedEvent(time, source, kind, detail))
        rows = [Row(row["series"], row["state"], by_time.get(row["time"], [])) for row in data["rows"]]
        return metadata, FakeRecording([row["time"] for row in data["rows"]], rows, data["units"])


MACHINE_EXTRA = """
open_parameters:
  - name: link_mass
    dimension: "[mass]"
    owed_by: test
    note: Link mass, left open to exercise resolution.
firmware:
  path: firmware
  chip: stm32_placeholder
  tick_rate: 1 kHz
operator_messages:
  - name: home
    label: home
    source: operator
    destination: controller
  - name: target
    label: target
    source: operator
    destination: controller
    fields:
      - name: angle
        dimension: dimensionless
  - name: done
    label: done
    source: controller
    destination: operator
"""


def write_designs(root: Path) -> Path:
    """`root/designs` with the shipped pendulum (release) and `machine`: the pendulum as a draft with firmware,
    operator messages and one open parameter (`link_mass`)."""
    designs = root / "designs"
    (designs / "pendulum").mkdir(parents=True)
    text = PENDULUM.read_text(encoding="utf-8")
    (designs / "pendulum" / "design.yaml").write_text(text, encoding="utf-8")
    machine = text.replace("name: pendulum", "name: machine", 1).replace("status: release", "status: draft", 1)
    machine = machine.replace("      mass: 1 kg", "      mass: ${link_mass}", 1)
    (designs / "machine").mkdir()
    (designs / "machine" / "design.yaml").write_text(machine + MACHINE_EXTRA, encoding="utf-8")
    return designs


MACHINE_CASE = """id: M1
title: Machine case
design:
  name: machine
  version: 0.1.0
controller_mode: native
seed: 7
initial_state:
  joints:
    - joint: pivot
      position: 0.05 rad
      velocity: 0 rad/s
  controller:
    homed: false
commands:
  - at: 0.0015 s
    message: home
  - at: 3 ms
    message: target
    payload:
      angle: 0.25 rad
end:
  at: 10 ms
  max_duration: 20 ms
criteria:
  - name: final_angle
    check: value
    quantity: joint:pivot.position
    statistic: final
    comparison: within
    target: 0.05 rad
    limit: 0.001 rad
"""


def write_case(directory: Path, text: str, name: str = "M1.yaml") -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / name
    path.write_text(text, encoding="utf-8")
    return path


def unrun_baseline(suite: str, cases: Sequence[str]) -> str:
    """A baseline for `machine` cases with one not-run `final_angle` entry each, as MACHINE_CASE states it."""
    lines = ["design:", "  name: machine", "  version: 0.1.0", f"suite: {suite}", "entries:"]
    for case in cases:
        lines.extend(
            [
                f"  - case: {case}",
                "    criterion: final_angle",
                "    status: not_run",
                "    limit: final within 0.001 rad of 0.05 rad",
            ]
        )
    return "\n".join(lines) + "\n"
