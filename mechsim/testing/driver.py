"""The protocol the test runner drives (TST-012, TST-013, TST-021, TST-036) and the record view scoring reads.

The runner talks to the simulation only through `SimulationDriver`: it loads the built design and the resolved case,
reads the firmware's reported controller state before t = 0, configures the run (rates, seed per module instance,
controller mode), sets the initial state, schedules operator commands and faults by base tick, and runs to the end
condition. The real driver over the core binding is wired at integration; tests use a scripted in-memory driver.
Times cross this boundary as integer base ticks; quantities as SI floats.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from fractions import Fraction
from pathlib import Path
from typing import Literal, Protocol

from mechsim.design.schema import Design
from mechsim.testing.schema import Case, InitialState

FaultOwner = Literal["signals", "physics", "controller"]
EndReason = Literal["end_time", "message", "controller_state", "max_duration", "run_error"]
MESSAGE_KIND = "operator_message"
FAULT_KIND = "controller_fault"


@dataclass(frozen=True)
class RunConfig:
    """Rates as exact rationals in Hz; `base_rate` is lcm(physics, control) and one tick is 1 / base_rate (KER)."""

    physics_rate: Fraction
    control_rate: Fraction | None
    base_rate: Fraction
    seed: int
    module_seeds: tuple[tuple[str, int], ...]
    controller_mode: Literal["native", "emulated"] | None
    dry: bool = False


@dataclass(frozen=True)
class ScheduledCommand:
    """An operator message for the operator link (TST-013); payload in SI by field name."""

    tick: int
    message: str
    payload: tuple[tuple[str, float], ...] = ()


@dataclass(frozen=True)
class ScheduledFault:
    """A fault for its owning module: active for start_tick <= tick < end_tick; end_tick None is instantaneous."""

    owner: FaultOwner
    kind: str
    target: str | None
    start_tick: int
    end_tick: int | None
    parameters: tuple[tuple[str, float | str], ...] = ()


@dataclass(frozen=True)
class EndSpec:
    """Exactly one trigger (a tick, an observed operator message, a controller state) and the tick limit."""

    max_tick: int
    at_tick: int | None = None
    message: str | None = None
    controller_state: str | None = None


@dataclass(frozen=True)
class RunReport:
    """How a run ended (KER RunReport plus the reason); `error` names the party and cause on a run error;
    `approximations` are the physics bridge's translation report lines (ADR 0003, invariant 7)."""

    reason: EndReason
    end_tick: int
    base_rate: Fraction
    error: str | None = None
    sink_errors: tuple[str, ...] = ()
    hook_errors: tuple[str, ...] = ()
    approximations: tuple[str, ...] = ()

    @property
    def errored(self) -> bool:
        return self.reason == "run_error"

    @property
    def end_time(self) -> float:
        return float(Fraction(self.end_tick) / self.base_rate)


@dataclass(frozen=True)
class Series:
    """One recorded scalar series: sample times in s, SI values, the unit text the recorder stored."""

    times: tuple[float, ...]
    values: tuple[float, ...]
    unit: str


@dataclass(frozen=True)
class RecordedEvent:
    """A timestamped event (REC-007 to REC-010). An operator message has kind MESSAGE_KIND and detail = its name."""

    time: float
    source: str
    kind: str
    detail: str


class RecordView(Protocol):
    """The read side of a recording that scoring and replay need (SCO-001, TST-033).

    `series` takes a case quantity reference (`signal:`, `pose:`, `joint:`, `state:`) or `actuator:<name>.position`
    and raises KeyError when the recording has no such series. `controller_states` has one entry per row.
    """

    def times(self) -> Sequence[float]: ...

    def series(self, reference: str) -> Series: ...

    def controller_states(self) -> Sequence[str]: ...

    def events(self) -> Sequence[RecordedEvent]: ...


class BuildResult(Protocol):
    """What the runner needs from the model builder: the resolved design it built and the module instances in
    kernel registration order (KER-009), which fixes the seed spawning order (D1)."""

    @property
    def design(self) -> Design: ...

    @property
    def module_instances(self) -> Sequence[str]: ...


class Builder(Protocol):
    def __call__(self, design: Design) -> BuildResult: ...


class SimulationDriver(Protocol):
    def load(self, build: BuildResult, case: Case) -> None: ...

    def reported_controller_state(self) -> Mapping[str, bool | str]: ...

    def configure(self, config: RunConfig) -> None: ...

    def set_initial_state(self, state: InitialState) -> None: ...

    def schedule_command(self, command: ScheduledCommand) -> None: ...

    def schedule_fault(self, fault: ScheduledFault) -> None: ...

    def run(self, end: EndSpec) -> RunReport: ...

    def recording(self) -> RecordView: ...


@dataclass(frozen=True)
class RunMetadata:
    """What an archive records beside the rows (L8): identity of design, case, seed, mode, sweep value, and the run
    configuration's control rate (None without a controller: the archive records the base rate)."""

    run_id: str
    case_id: str
    design_name: str
    design_version: str
    design_hash: str
    case_hash: str
    seed: int
    controller_mode: str | None
    sweep: tuple[str, str] | None
    report: RunReport
    bindings: tuple[tuple[str, str], ...] = field(default_factory=tuple)
    control_rate: Fraction | None = None


class Archiver(Protocol):
    """Writes a run archive through the record module; returns the archive's sha256 hex digest."""

    def write(self, path: Path, recording: RecordView, metadata: RunMetadata) -> str: ...


class ArchiveReader(Protocol):
    """Opens an archive alone, without the engine that produced it (TST-033)."""

    def open(self, path: Path) -> tuple[RunMetadata, RecordView]: ...
