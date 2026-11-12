"""The protocol the test runner drives (TST-012, TST-013, TST-021, TST-036) and the record view scoring reads.

The runner talks to the simulation only through `SimulationDriver`: it loads the built design and the resolved case,
reads the firmware's reported controller state before t = 0, configures the run (rates, seed per module instance,
controller mode), sets the initial state, schedules operator commands and faults by base tick, and runs to the end
condition. The real driver over the core binding is wired at integration; tests use a scripted in-memory driver.
Times cross this boundary as integer base ticks; quantities as SI floats.
"""

from __future__ import annotations

import json
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
# A tier 1 integrity limit crossing (ADR 0010, INT-024, REC-010): source and kind `integrity`, detail
# `IntegrityCrossing.detail()`.
INTEGRITY_KIND = "integrity"
INTEGRITY_SOURCE = "integrity"
# Archive metadata key of the material overrides text (brief M's `materials.overrides.METADATA_KEY`); its sha256 is
# under `<key>_sha256`.
MATERIAL_OVERRIDES_KEY = "material_overrides"


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
    configuration's control rate (None without a controller: the archive records the base rate). `members` is the
    build's structural member list (BLD-008), `material_overrides` the design's overrides file (MAT), `variation` the
    drawn values of a seeded variation run (its seed is `seed`, TST-026), `integrity` the per-member table (INT-022)."""

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
    members: tuple[MemberSection, ...] = ()
    material_overrides: MaterialOverridesRecord | None = None
    variation: tuple[tuple[str, str], ...] | None = None
    integrity: IntegrityResults | None = None


class Archiver(Protocol):
    """Writes a run archive through the record module; returns the archive's sha256 hex digest."""

    def write(self, path: Path, recording: RecordView, metadata: RunMetadata) -> str: ...


class ArchiveReader(Protocol):
    """Opens an archive alone, without the engine that produced it (TST-033)."""

    def open(self, path: Path) -> tuple[RunMetadata, RecordView]: ...


# ===== Structural results ===============================================


class StructuralError(ValueError):
    """An integrity event detail or results text that does not parse."""


@dataclass(frozen=True)
class IntegrityCrossing:
    """One tier 1 limit crossing (INT-024): the member (or rope), the quantity (`stress`, `deflection`,
    `safety_factor`, `tension`), the value and the limit it crossed, SI. Recorded as a `RecordedEvent` with
    source and kind `integrity` and this detail, stamped with the simulated time of the crossing (REC-010)."""

    member: str
    quantity: str
    value: float
    limit: float
    tier: int = 1

    def detail(self) -> str:
        return json.dumps(
            {
                "limit": self.limit,
                "member": self.member,
                "quantity": self.quantity,
                "tier": self.tier,
                "value": self.value,
            },
            sort_keys=True,
        )

    @classmethod
    def parse(cls, detail: str) -> IntegrityCrossing:
        try:
            data = json.loads(detail)
            crossing = cls(
                str(data["member"]),
                str(data["quantity"]),
                float(data["value"]),
                float(data["limit"]),
                int(data["tier"]),
            )
        except (ValueError, KeyError, TypeError) as error:
            raise StructuralError(f"integrity event detail {detail!r}: {error}") from error
        return crossing


@dataclass(frozen=True)
class MemberIntegrity:
    """One row of the per-member table (INT-022): peak stress (Pa), peak deflection (m) and minimum safety factor
    over the run, each with the simulated time it occurred (s), and the integrity flags raised on the member."""

    member: str
    peak_stress: float
    peak_stress_time: float
    deflection: float
    deflection_time: float
    safety_factor: float
    safety_factor_time: float
    flags: tuple[str, ...] = ()


@dataclass(frozen=True)
class RopeIntegrity:
    """One row of the per-rope table: peak tension (N) over the run, the simulated time it occurred (s), and the
    integrity flags raised on the rope."""

    rope: str
    peak_tension: float
    peak_tension_time: float
    flags: tuple[str, ...] = ()


@dataclass(frozen=True)
class IntegrityResults:
    """What integrity attaches to a run (INT-022, brief I1 fills it): the per-member table and, optionally, the
    per-rope table; scoring reads structural criteria from it and never recomputes them (SCO-014)."""

    members: tuple[MemberIntegrity, ...]
    ropes: tuple[RopeIntegrity, ...] = ()

    def member(self, name: str) -> MemberIntegrity:
        for entry in self.members:
            if entry.member == name:
                return entry
        raise KeyError(name)

    def to_json(self) -> str:
        """`{"members": [...], "ropes": [...]}`, keys sorted."""
        members = [
            {
                "deflection": entry.deflection,
                "deflection_time": entry.deflection_time,
                "flags": list(entry.flags),
                "member": entry.member,
                "peak_stress": entry.peak_stress,
                "peak_stress_time": entry.peak_stress_time,
                "safety_factor": entry.safety_factor,
                "safety_factor_time": entry.safety_factor_time,
            }
            for entry in self.members
        ]
        ropes = [
            {
                "flags": list(entry.flags),
                "peak_tension": entry.peak_tension,
                "peak_tension_time": entry.peak_tension_time,
                "rope": entry.rope,
            }
            for entry in self.ropes
        ]
        return json.dumps({"members": members, "ropes": ropes}, sort_keys=True)

    @classmethod
    def from_json(cls, text: str) -> IntegrityResults:
        try:
            data = json.loads(text)
            members = tuple(
                MemberIntegrity(
                    str(row["member"]),
                    float(row["peak_stress"]),
                    float(row["peak_stress_time"]),
                    float(row["deflection"]),
                    float(row["deflection_time"]),
                    float(row["safety_factor"]),
                    float(row["safety_factor_time"]),
                    tuple(str(flag) for flag in row["flags"]),
                )
                for row in data["members"]
            )
            ropes = tuple(
                RopeIntegrity(
                    str(row["rope"]),
                    float(row["peak_tension"]),
                    float(row["peak_tension_time"]),
                    tuple(str(flag) for flag in row["flags"]),
                )
                for row in data["ropes"]
            )
        except (ValueError, KeyError, TypeError) as error:
            raise StructuralError(f"integrity results: {error}") from error
        return cls(members, ropes)


@dataclass(frozen=True)
class MemberSection:
    """A structural member as the run recorded it (BLD-008), SI, member frame (x along the member, y the bending
    direction, z across; ruling 25): area, second moments about z (`second_moment_z`, the neutral model's
    `second_moment_of_area`) and y, extreme-fibre distances along y and z, length. A section given only by area and
    second moment has no y-axis properties (None); stress overlays need them."""

    name: str
    material: str
    area: float
    second_moment_z: float
    length: float
    second_moment_y: float | None = None
    fibre_y: float | None = None
    fibre_z: float | None = None


_SECTION_FIELDS = ("area", "second_moment_z", "length", "second_moment_y", "fibre_y", "fibre_z")


def members_json(members: Sequence[MemberSection]) -> str:
    rows: list[dict[str, object]] = []
    for member in members:
        row: dict[str, object] = {"material": member.material, "name": member.name}
        for name in _SECTION_FIELDS:
            row[name] = getattr(member, name)
        rows.append(row)
    return json.dumps(rows, sort_keys=True)


def _optional(value: object) -> float | None:
    if value is None:
        return None
    return float(value)  # type: ignore[arg-type]


def members_from_json(text: str) -> tuple[MemberSection, ...]:
    try:
        rows = json.loads(text)
        members = tuple(
            MemberSection(
                str(row["name"]),
                str(row["material"]),
                float(row["area"]),
                float(row["second_moment_z"]),
                float(row["length"]),
                _optional(row["second_moment_y"]),
                _optional(row["fibre_y"]),
                _optional(row["fibre_z"]),
            )
            for row in rows
        )
    except (ValueError, KeyError, TypeError) as error:
        raise StructuralError(f"member list: {error}") from error
    return members


@dataclass(frozen=True)
class MaterialOverridesRecord:
    """The design's material overrides file as the run used it: its text and sha256 (metadata key
    `material_overrides`, MAT; brief M's `METADATA_KEY`)."""

    text: str
    sha256: str


class IntegrityCheck(Protocol):
    """Integrity over a finished run (brief I1): the per-member table from the recording and the build."""

    def __call__(self, build: BuildResult, case: Case, recording: RecordView) -> IntegrityResults | None: ...
