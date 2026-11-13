"""Integrity check plugin interface and the run inputs every check reads (INT-001, INT-003, ADR 0009).

A check is one class: `name`, the fidelity `tiers` it offers (INT-003: like component plugins; a higher tier adds rows
and never changes a lower tier's rows), its `parameters` model (its entry in `schemas/integrity.json`) and
`evaluate(inputs, parameters, tier) -> CheckResult`. "Tier" here is the fidelity tier of ADR 0009, not ADR 0010's
evaluation tier: every check in this package runs after the run, over the recording (round 5 ruling 5).

Member end-loads are in the member frame (round 5 ruling 24): x from the start to the end node, y and z the section's
principal axes. Each end's wrench is the load applied to the member at that end by what it attaches to, so the
internal axial force is -F_start.x at the start and +F_end.x at the end (tension positive).
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Annotated, Any, ClassVar, Literal, Protocol

import numpy as np
from numpy.typing import NDArray
from pydantic import BeforeValidator, WithJsonSchema

from mechsim.schema_base import StrictModel
from mechsim.units import UnitError, parse_quantity

FloatArray = NDArray[np.float64]

QUANTITY_PATTERN = r"^\s*[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?(\s+\S.*)?$"


class IntegrityError(ValueError):
    """Integrity input that cannot be evaluated: missing material, unsupported member, bad configuration."""


def _parser(dimension: str) -> Callable[[Any], float]:
    def _parse(value: Any) -> float:
        try:
            return parse_quantity(value, dimension)
        except UnitError as error:
            raise ValueError(str(error)) from error

    return _parse


_QUANTITY_SCHEMA = WithJsonSchema({"type": "string", "pattern": QUANTITY_PATTERN})
# Configuration quantities: a unit string in the file, the SI magnitude in the model (L5).
ForceQuantity = Annotated[float, BeforeValidator(_parser("[mass] * [length] / [time] ** 2")), _QUANTITY_SCHEMA]
TimeQuantity = Annotated[float, BeforeValidator(_parser("[time]")), _QUANTITY_SCHEMA]


class NoParameters(StrictModel):
    """A check without parameters."""


# ===== Material view ====================================================


class SpreadLike(Protocol):
    @property
    def min(self) -> float: ...

    @property
    def typ(self) -> float: ...

    @property
    def max(self) -> float: ...


StrengthKind = Literal["yield", "ultimate", "shear"]


class MaterialView(Protocol):
    """What a check reads from one material: `materials.database.IntegrityMaterialView` (MAT-021)."""

    def strength_spread(
        self,
        kind: StrengthKind,
        temperature: float,
        direction_body: Sequence[float] | None = None,
        process: Any = None,
    ) -> SpreadLike: ...

    def sn_curve(self) -> tuple[tuple[float, float], ...]: ...


@dataclass(frozen=True)
class Spread:
    min: float
    typ: float
    max: float

    @classmethod
    def of(cls, spread: SpreadLike) -> Spread:
        return cls(float(spread.min), float(spread.typ), float(spread.max))

    def over(self, denominator: float) -> Spread:
        """Each spread value divided by `denominator`: a safety factor from strengths and one stress."""
        if denominator <= 0.0:
            return Spread(math.inf, math.inf, math.inf)
        return Spread(self.min / denominator, self.typ / denominator, self.max / denominator)


# ===== Run inputs =======================================================


@dataclass(frozen=True)
class SectionProperties:
    """A member section about both principal axes (round 5 ruling 25): bending about y uses I_y and the fibre c_z,
    about z uses I_z and c_y. E in Pa, rho in kg/m^3, lengths in m."""

    area: float
    elastic_modulus: float
    density: float
    length: float
    i_y: float
    i_z: float
    c_y: float
    c_z: float
    single_axis: bool = False

    def i_min(self) -> float:
        return min(self.i_y, self.i_z)


@dataclass(frozen=True)
class Wrenches:
    """Per-sample end loads of one member, member frame: arrays of shape (rows, 3)."""

    start_force: FloatArray
    start_moment: FloatArray
    end_force: FloatArray
    end_moment: FloatArray

    def scaled(self, factor: float) -> Wrenches:
        return Wrenches(
            self.start_force * factor, self.start_moment * factor, self.end_force * factor, self.end_moment * factor
        )


SupportCondition = Literal["clamped", "pinned", "roller"]


@dataclass(frozen=True)
class NodeChain:
    """Where a lumped member's chain sits among the recorded body poses (`physics/bodies/*`, the native adapter's
    `PhysicsOutputs.bodies`: the neutral bodies, then each lumped member's node bodies root to tip, a pinned root's
    pin segment as node 0), and its root frame: the parent body's index (None: the world) with the member's placement
    in it. `segment_length` l = L / N: the last node's far end lies l / 2 along its x (INT-006, ruling 28)."""

    first_body: int
    count: int
    root: Literal["clamped", "pinned"]
    parent_body: int | None
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    segment_length: float

    def to_limits(self) -> dict[str, Any]:
        return {
            "count": self.count,
            "first_body": self.first_body,
            "orientation": list(self.orientation),
            "parent_body": self.parent_body,
            "position": list(self.position),
            "root": self.root,
        }


@dataclass(frozen=True)
class BodyPoses:
    """Recorded body poses per sample: positions (rows, bodies, 3) and orientations (rows, bodies, 4), w first."""

    positions: FloatArray
    orientations: FloatArray


@dataclass(frozen=True)
class MemberInputs:
    """One structural member (BLD-008) with its recorded end-loads. `axis_body` is the member axis in its body frame
    (the direction for direction-dependent strengths); `natural_frequency` the calibrated first frequency (Hz) when
    the design records one (BLD-019)."""

    name: str
    material: str
    section: SectionProperties
    supports: tuple[tuple[float, SupportCondition], ...]
    loads: Wrenches
    axis_body: tuple[float, float, float] = (1.0, 0.0, 0.0)
    natural_frequency: float | None = None
    mounts: tuple[str, ...] = ()
    nodes: NodeChain | None = None


@dataclass(frozen=True)
class RopeLoads:
    """A rope (ADR 0016) with its recorded largest segment tension per sample (N) and its breaking tension (None:
    never snaps, ruling 19)."""

    name: str
    tension: FloatArray
    breaking_tension: float | None = None


@dataclass(frozen=True)
class JointLoads:
    """A mount or fastener set (BLD-031) with its joint's recorded reaction, world-aligned, arrays (rows, 3).
    `member` is the structural member a mount holds."""

    name: str
    kind: Literal["mount", "fastener"]
    joint: str
    force: FloatArray
    moment: FloatArray
    member: str | None = None


Excitation = Callable[[str], tuple[FloatArray, FloatArray]]


def _no_series(reference: str) -> tuple[FloatArray, FloatArray]:
    raise KeyError(reference)


@dataclass(frozen=True)
class RunInputs:
    """Everything a check reads from one run. `load_factor` is the design-load rule's factor from the design file
    (INT-019; 1 when the design states none); `temperature` the run's ambient temperature (K); `series` reads a
    recorded scalar series by case quantity reference (times, values)."""

    times: FloatArray
    members: tuple[MemberInputs, ...]
    materials: Mapping[str, MaterialView]
    temperature: float
    load_factor: float = 1.0
    minimum_safety_factor: float | None = None
    deflection_limit: float | None = None
    joints: tuple[JointLoads, ...] = ()
    series: Excitation = _no_series
    ropes: tuple[RopeLoads, ...] = ()
    design_tension: float | None = None
    bodies: BodyPoses | None = None

    def material(self, name: str) -> MaterialView:
        if name not in self.materials:
            raise IntegrityError(f"material {name!r}: not in the run's material set")
        return self.materials[name]

    def member(self, name: str) -> MemberInputs:
        for member in self.members:
            if member.name == name:
                return member
        raise IntegrityError(f"no structural member {name!r}")


# ===== Results ==========================================================


@dataclass(frozen=True)
class CheckRow:
    """One result: the subject (member, mount, fastener), the quantity, its governing value (SI) and when it
    occurred, the limit it is judged against, the safety factor or margin with its material spread (None when the
    row carries no margin), the fidelity tier that produced it, and flags."""

    subject: str
    quantity: str
    value: float
    time: float
    tier: int
    limit: float | None = None
    margin: Spread | None = None
    flags: tuple[str, ...] = ()
    detail: tuple[tuple[str, float], ...] = ()

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "detail": dict(self.detail),
            "flags": list(self.flags),
            "limit": self.limit,
            "quantity": self.quantity,
            "subject": self.subject,
            "tier": self.tier,
            "time": self.time,
            "value": self.value,
        }
        if self.margin is None:
            out["margin"] = None
        else:
            out["margin"] = {"max": self.margin.max, "min": self.margin.min, "typ": self.margin.typ}
        return out


@dataclass(frozen=True)
class CheckResult:
    """A check's rows at the evaluated tier; `cycle_counts` (subject, cycles) for the wear seam (INT-037)."""

    check: str
    tier: int
    rows: tuple[CheckRow, ...]
    cycle_counts: tuple[tuple[str, float], ...] = ()
    notes: tuple[str, ...] = ()

    def rows_of_tier(self, tier: int) -> tuple[CheckRow, ...]:
        return tuple(row for row in self.rows if row.tier <= tier)


class IntegrityCheck(ABC):
    """An integrity check plugin (INT-001)."""

    name: ClassVar[str]
    tiers: ClassVar[tuple[int, ...]] = (1,)
    parameters: ClassVar[type[StrictModel]] = NoParameters

    @abstractmethod
    def evaluate(self, inputs: RunInputs, parameters: StrictModel, tier: int) -> CheckResult: ...

    def check_tier(self, tier: int) -> None:
        if tier not in self.tiers:
            raise IntegrityError(f"check {self.name!r}: tier {tier} not offered (tiers {list(self.tiers)})")


__all__ = [
    "CheckResult",
    "CheckRow",
    "ForceQuantity",
    "IntegrityCheck",
    "IntegrityError",
    "JointLoads",
    "MaterialView",
    "MemberInputs",
    "NoParameters",
    "RopeLoads",
    "RunInputs",
    "SectionProperties",
    "Spread",
    "TimeQuantity",
    "Wrenches",
]
