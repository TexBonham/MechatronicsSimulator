"""Engine-neutral physics model (ADR 0003): the builder's output and every adapter's input.

Data only: pydantic v2 models whose validation is the whole behavior. Files carry unit strings; the models hold plain
SI floats (L5). `python -m mechsim.physics.neutral --write-schema` regenerates `schemas/physics_model.json` (L4).
"""

from __future__ import annotations

import argparse
import io
import json
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Annotated, Any, Literal

from pydantic import (
    AfterValidator,
    BaseModel,
    ConfigDict,
    Field,
    JsonValue,
    PlainSerializer,
    WithJsonSchema,
    model_validator,
)
from pydantic.json_schema import GenerateJsonSchema, JsonSchemaValue
from pydantic_core import SchemaSerializer, core_schema
from ruamel.yaml import YAML

from mechsim import units

# ===== Field types ======================================================

SCHEMA_PATH = Path(__file__).resolve().parents[2] / "schemas" / "physics_model.json"
SCHEMA_ID = "https://mechsim.local/schemas/physics_model.json"
WORLD = "world"
_QUANTITY_PATTERN = r"^\s*[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?(\s|$)"


def _validator(field_type: Any) -> Any:
    return field_type.__metadata__[0]


def _serializer(unit: str) -> PlainSerializer:
    def _format(value: float) -> str:
        return f"{value!r} {unit}"

    return PlainSerializer(_format, return_type=str)


def _schema(dimension: str, unit: str) -> WithJsonSchema:
    return WithJsonSchema(
        {
            "type": "string",
            "pattern": _QUANTITY_PATTERN,
            "description": f"Quantity with an explicit unit of dimension {dimension}; SI unit {unit}",
        }
    )


def _positive(value: float) -> float:
    if not value > 0.0:
        raise ValueError(f"must be greater than zero, got {value!r}")
    return value


def _non_negative(value: float) -> float:
    if not value >= 0.0:
        raise ValueError(f"must be zero or greater, got {value!r}")
    return value


def _nonzero(value: float) -> float:
    if value == 0.0:
        raise ValueError("must not be zero")
    return value


def _unit_interval_open_low(value: float) -> float:
    if not (value > 0.0 and value <= 1.0):
        raise ValueError(f"must be in (0, 1], got {value!r}")
    return value


def _unit_interval(value: float) -> float:
    if not (value >= 0.0 and value <= 1.0):
        raise ValueError(f"must be in [0, 1], got {value!r}")
    return value


_AngularVelocityField = units.Frequency
_LinearDampingField = units.quantity_field("[mass] / [time]")
_RotationalDampingField = units.quantity_field("[mass] * [length] ** 2 / [time]")

Length = Annotated[float, _validator(units.Length), _serializer("m"), _schema("[length]", "m")]
Mass = Annotated[float, _validator(units.Mass), _serializer("kg"), _schema("[mass]", "kg")]
Time = Annotated[float, _validator(units.Time), _serializer("s"), _schema("[time]", "s")]
Angle = Annotated[float, _validator(units.Angle), _serializer("rad"), _schema("angle", "rad")]
Force = Annotated[float, _validator(units.Force), _serializer("N"), _schema("force", "N")]
Torque = Annotated[float, _validator(units.Torque), _serializer("N*m"), _schema("torque", "N*m")]
Stiffness = Annotated[float, _validator(units.Stiffness), _serializer("N/m"), _schema("[mass]/[time]**2", "N/m")]
RotationalStiffness = Annotated[
    float, _validator(units.RotationalStiffness), _serializer("N*m/rad"), _schema("torque per angle", "N*m/rad")
]
LinearDamping = Annotated[
    float, _validator(_LinearDampingField), _serializer("N*s/m"), _schema("[mass]/[time]", "N*s/m")
]
RotationalDamping = Annotated[
    float, _validator(_RotationalDampingField), _serializer("N*m*s/rad"), _schema("torque*time/angle", "N*m*s/rad")
]
Velocity = Annotated[float, _validator(units.Velocity), _serializer("m/s"), _schema("velocity", "m/s")]
AngularVelocity = Annotated[
    float, _validator(_AngularVelocityField), _serializer("rad/s"), _schema("angle/[time]", "rad/s")
]
Pressure = Annotated[float, _validator(units.Pressure), _serializer("Pa"), _schema("pressure", "Pa")]
Density = Annotated[float, _validator(units.Density), _serializer("kg/m**3"), _schema("density", "kg/m**3")]
Acceleration = Annotated[
    float, _validator(units.Acceleration), _serializer("m/s**2"), _schema("acceleration", "m/s**2")
]
Inertia = Annotated[float, _validator(units.Inertia), _serializer("kg*m**2"), _schema("inertia", "kg*m**2")]
Area = Annotated[float, _validator(units.Area), _serializer("m**2"), _schema("[length]**2", "m**2")]
SecondMomentOfArea = Annotated[
    float, _validator(units.SecondMomentOfArea), _serializer("m**4"), _schema("[length]**4", "m**4")
]

PositiveLength = Annotated[Length, AfterValidator(_positive)]
NonNegativeLength = Annotated[Length, AfterValidator(_non_negative)]
PositiveMass = Annotated[Mass, AfterValidator(_positive)]
PositiveTime = Annotated[Time, AfterValidator(_positive)]
PositiveAngle = Annotated[Angle, AfterValidator(_positive)]
NonNegativeAngle = Annotated[Angle, AfterValidator(_non_negative)]
NonNegativeForce = Annotated[Force, AfterValidator(_non_negative)]
NonNegativeTorque = Annotated[Torque, AfterValidator(_non_negative)]
NonNegativeStiffness = Annotated[Stiffness, AfterValidator(_non_negative)]
PositiveStiffness = Annotated[Stiffness, AfterValidator(_positive)]
NonNegativeRotationalStiffness = Annotated[RotationalStiffness, AfterValidator(_non_negative)]
PositiveRotationalStiffness = Annotated[RotationalStiffness, AfterValidator(_positive)]
NonNegativeLinearDamping = Annotated[LinearDamping, AfterValidator(_non_negative)]
NonNegativeRotationalDamping = Annotated[RotationalDamping, AfterValidator(_non_negative)]
PositiveVelocity = Annotated[Velocity, AfterValidator(_positive)]
PositiveAngularVelocity = Annotated[AngularVelocity, AfterValidator(_positive)]
PositivePressure = Annotated[Pressure, AfterValidator(_positive)]
PositiveDensity = Annotated[Density, AfterValidator(_positive)]
PositiveArea = Annotated[Area, AfterValidator(_positive)]
PositiveSecondMoment = Annotated[SecondMomentOfArea, AfterValidator(_positive)]

Number = Annotated[float, Field(strict=True, allow_inf_nan=False)]
PositiveNumber = Annotated[Number, AfterValidator(_positive)]
Count = Annotated[int, Field(strict=True)]
Flag = Annotated[bool, Field(strict=True)]
Name = Annotated[str, Field(strict=True, pattern=r"^[a-z][a-z0-9_]*$")]

Vector3 = tuple[Length, Length, Length]


def _nonzero_vector(value: tuple[float, ...]) -> tuple[float, ...]:
    if all(component == 0.0 for component in value):
        raise ValueError("must not be the zero vector")
    return value


Direction = Annotated[tuple[Number, Number, Number], AfterValidator(_nonzero_vector)]
Quaternion = Annotated[tuple[Number, Number, Number, Number], AfterValidator(_nonzero_vector)]

_ORIGIN = (0.0, 0.0, 0.0)
_IDENTITY = (1.0, 0.0, 0.0, 0.0)
_Z_AXIS = (0.0, 0.0, 1.0)


class _Model(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


# ===== Run configuration ================================================


class DesignRef(_Model):
    """The design this model was built from (D3): semver plus the design file's git blob hash."""

    name: Name
    version: Annotated[
        str, Field(strict=True, pattern=r"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)(-[0-9A-Za-z.\-]+)?$")
    ]
    blob: Annotated[str, Field(strict=True, pattern=r"^([0-9a-f]{40}|[0-9a-f]{64})$")]


class RunConfig(_Model):
    """Engine-neutral run configuration the kernel needs; defaults per ADR 0013 and the engine contract.

    The engine currently takes one constraint velocity tolerance; the native adapter maps both fields onto it.
    """

    step: PositiveTime = 1e-3
    iteration_cap: Annotated[Count, Field(ge=1)] = 50
    projection_iterations: Annotated[Count, Field(ge=0)] = 3
    drift_threshold_linear: PositiveLength = 1e-6
    drift_threshold_angular: PositiveAngle = 1e-6
    symmetry_tolerance: PositiveNumber = 1e-12
    unit_quaternion_tolerance: PositiveNumber = 1e-12
    velocity_tolerance_linear: PositiveVelocity = 1e-9
    velocity_tolerance_angular: PositiveAngularVelocity = 1e-9


# ===== Joints ===========================================================

JointType = Literal["revolute", "prismatic", "fixed", "planar", "spherical", "free"]

DOF_COUNTS: dict[str, tuple[int, int]] = {
    "revolute": (0, 1),
    "prismatic": (1, 0),
    "fixed": (0, 0),
    "planar": (2, 1),
    "spherical": (0, 3),
    "free": (3, 3),
}
SPRINGLESS_JOINT_TYPES = frozenset({"spherical", "free"})


def _check_limits(lower: float | None, upper: float | None) -> None:
    if lower is not None and upper is not None and not lower < upper:
        raise ValueError(f"lower limit {lower!r} must be below upper limit {upper!r}")


class LinearDof(_Model):
    """One translational degree of freedom: limits (None = unlimited), friction, damping, spring and rest."""

    lower: Length | None = None
    upper: Length | None = None
    friction: NonNegativeForce = 0.0
    damping: NonNegativeLinearDamping = 0.0
    stiffness: NonNegativeStiffness = 0.0
    rest: Length = 0.0

    @model_validator(mode="after")
    def _limits(self) -> LinearDof:
        _check_limits(self.lower, self.upper)
        return self


class AngularDof(_Model):
    """One rotational degree of freedom: limits (None = unlimited), friction, damping, spring and rest."""

    lower: Angle | None = None
    upper: Angle | None = None
    friction: NonNegativeTorque = 0.0
    damping: NonNegativeRotationalDamping = 0.0
    stiffness: NonNegativeRotationalStiffness = 0.0
    rest: Angle = 0.0

    @model_validator(mode="after")
    def _limits(self) -> AngularDof:
        _check_limits(self.lower, self.upper)
        return self


class Joint(_Model):
    """Joint to the parent: placement in the parent frame, axis in the joint frame, per-DOF parameters.

    `linear` and `angular` are empty (no limits, friction or springs) or list every DOF of the type in order:
    planar [x, y] + [theta]; free [x, y, z] + [rx, ry, rz]; spherical [rx, ry, rz].
    """

    name: Name
    type: JointType
    position: Vector3
    orientation: Quaternion = _IDENTITY
    axis: Direction = _Z_AXIS
    linear: tuple[LinearDof, ...] = ()
    angular: tuple[AngularDof, ...] = ()

    @model_validator(mode="after")
    def _dof_counts(self) -> Joint:
        linear_count, angular_count = DOF_COUNTS[self.type]
        if len(self.linear) not in (0, linear_count):
            raise ValueError(
                f"joint '{self.name}' ({self.type}) takes {linear_count} linear DOFs, got {len(self.linear)}"
            )
        if len(self.angular) not in (0, angular_count):
            raise ValueError(
                f"joint '{self.name}' ({self.type}) takes {angular_count} angular DOFs, got {len(self.angular)}"
            )
        if self.type in SPRINGLESS_JOINT_TYPES:
            dofs: tuple[LinearDof | AngularDof, ...] = (*self.linear, *self.angular)
            for dof in dofs:
                if dof.stiffness != 0.0 or dof.rest != 0.0:
                    raise ValueError(f"joint '{self.name}' ({self.type}) takes no stiffness or rest position (ENG-051)")
                if dof.lower is not None or dof.upper is not None:
                    raise ValueError(f"joint '{self.name}' ({self.type}) takes no limits")
        return self


# ===== Bodies and collision shapes ======================================


class InertiaTensor(_Model):
    """Symmetric inertia tensor about the center of mass in body axes; must be positive semi-definite."""

    ixx: Inertia
    iyy: Inertia
    izz: Inertia
    ixy: Inertia = 0.0
    ixz: Inertia = 0.0
    iyz: Inertia = 0.0

    @model_validator(mode="after")
    def _semi_definite(self) -> InertiaTensor:
        minors = (
            self.ixx,
            self.iyy,
            self.izz,
            self.ixx * self.iyy - self.ixy * self.ixy,
            self.ixx * self.izz - self.ixz * self.ixz,
            self.iyy * self.izz - self.iyz * self.iyz,
            self.ixx * (self.iyy * self.izz - self.iyz * self.iyz)
            - self.ixy * (self.ixy * self.izz - self.iyz * self.ixz)
            + self.ixz * (self.ixy * self.iyz - self.iyy * self.ixz),
        )
        if any(minor < 0.0 for minor in minors):
            raise ValueError("inertia tensor is not positive semi-definite")
        return self


class _Shape(_Model):
    position: Vector3 = _ORIGIN
    orientation: Quaternion = _IDENTITY
    material: Name


class Sphere(_Shape):
    shape: Literal["sphere"]
    radius: PositiveLength


class Capsule(_Shape):
    """Along the shape's z axis; `length` is the cylindrical part between the cap centers."""

    shape: Literal["capsule"]
    radius: PositiveLength
    length: PositiveLength


class Box(_Shape):
    """Full edge lengths along the shape's x, y and z axes."""

    shape: Literal["box"]
    size: tuple[PositiveLength, PositiveLength, PositiveLength]


class Cylinder(_Shape):
    """Along the shape's z axis."""

    shape: Literal["cylinder"]
    radius: PositiveLength
    length: PositiveLength


def _spans_volume(vertices: Sequence[tuple[float, float, float]]) -> bool:
    origin = vertices[0]
    edges = [(v[0] - origin[0], v[1] - origin[1], v[2] - origin[2]) for v in vertices[1:]]
    for i, a in enumerate(edges):
        for j in range(i + 1, len(edges)):
            b = edges[j]
            cross = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
            for c in edges[j + 1 :]:
                if cross[0] * c[0] + cross[1] * c[1] + cross[2] * c[2] != 0.0:
                    return True
    return False


class ConvexHull(_Shape):
    """Convex hull of the vertex list in the shape frame: at least four vertices that span a volume."""

    shape: Literal["convex_hull"]
    vertices: Annotated[tuple[Vector3, ...], Field(min_length=4)]

    @model_validator(mode="after")
    def _volume(self) -> ConvexHull:
        if not _spans_volume(self.vertices):
            raise ValueError("convex hull vertices are coplanar")
        return self


class Plane(_Shape):
    """Half-space below the shape's x-y plane (normal +z); allowed only on the world, as the floor."""

    shape: Literal["plane"]


Shape = Annotated[Sphere | Capsule | Box | Cylinder | ConvexHull | Plane, Field(discriminator="shape")]


class Body(_Model):
    """Rigid body. `parent` is "world" or a body listed earlier; a body without a joint is unconstrained."""

    name: Name
    mass: PositiveMass
    com: Vector3
    inertia: InertiaTensor
    material: Name | None = None
    parent: Name
    joint: Joint | None = None
    shapes: tuple[Shape, ...] = ()
    catalog_part: Name | None = None

    @model_validator(mode="after")
    def _complete(self) -> Body:
        problems: list[str] = []
        if self.material is None:
            problems.append(f"body '{self.name}' has no material")
        if self.joint is None:
            problems.append(f"unconstrained body '{self.name}': no joint to its parent (use a free joint)")
        if any(isinstance(shape, Plane) for shape in self.shapes):
            problems.append(f"body '{self.name}': a plane shape is allowed only on the world")
        if problems:
            raise ValueError("; ".join(problems))
        return self


# ===== Constraints ======================================================


class PointConstraint(_Model):
    """Coincidence of a point fixed in body_a with a point fixed in body_b (anchors in each body frame)."""

    type: Literal["point"]
    name: Name
    body_a: Name
    body_b: Name
    anchor_a: Vector3
    anchor_b: Vector3


class HingeConstraint(_Model):
    """Point coincidence plus alignment of axis_a in body_a with axis_b in body_b."""

    type: Literal["hinge"]
    name: Name
    body_a: Name
    body_b: Name
    anchor_a: Vector3
    anchor_b: Vector3
    axis_a: Direction
    axis_b: Direction


Constraint = Annotated[PointConstraint | HingeConstraint, Field(discriminator="type")]


# ===== Contacts =========================================================


class ContactParameters(_Model):
    """Soft contact (ADR 0002): normal stiffness and damping, Coulomb friction coefficient."""

    stiffness: PositiveStiffness
    damping: NonNegativeLinearDamping
    friction: Annotated[Number, AfterValidator(_non_negative)]


class MaterialPair(ContactParameters):
    """Contact parameters overriding the default for one unordered pair of materials."""

    materials: tuple[Name, Name]


class ContactModel(_Model):
    """Contacts element (ADR 0003): default parameters, material-pair overrides, excluded body pairs."""

    default: ContactParameters
    pairs: tuple[MaterialPair, ...] = ()
    exclusions: tuple[tuple[Name, Name], ...] = ()


# ===== Lumped members ===================================================

DEFAULT_SEGMENTS = 4
PINNED_ROLLER_SEGMENTS = 8
DEFAULT_ROLLER_MASS_FRACTION = 1e-3


class Section(_Model):
    """Member section and material constants: E, I, rho, A, L."""

    elastic_modulus: PositivePressure
    second_moment_of_area: PositiveSecondMoment
    density: PositiveDensity
    area: PositiveArea
    length: PositiveLength


class LumpedMember(_Model):
    """Multi-segment lumped beam (R-13) rooted on `parent` at the given placement, along the root frame's x axis.

    `segments` defaults to 4, or 8 for a pinned root with a roller end (ENG-040).
    """

    name: Name
    parent: Name
    position: Vector3
    orientation: Quaternion = _IDENTITY
    section: Section
    root: Literal["clamped", "pinned"]
    end: Literal["free", "roller"]
    segments: Annotated[Count, Field(ge=1)] = DEFAULT_SEGMENTS
    roller_mass_fraction: PositiveNumber = DEFAULT_ROLLER_MASS_FRACTION
    node_damping: NonNegativeRotationalDamping = 0.0
    structural_member: Name | None = None

    @model_validator(mode="before")
    @classmethod
    def _default_segments(cls, data: Any) -> Any:
        pinned_roller = isinstance(data, dict) and data.get("root") == "pinned" and data.get("end") == "roller"
        if pinned_roller and "segments" not in data:
            return {**data, "segments": PINNED_ROLLER_SEGMENTS}
        return data


# ===== Transmissions ====================================================


class _Transmission(_Model):
    name: Name
    shaft_joint: Name
    driven_joint: Name
    efficiency: Annotated[Number, AfterValidator(_unit_interval_open_low)]
    slip: Annotated[Number, AfterValidator(_unit_interval)] = 0.0


class BeltPulley(_Transmission):
    """Revolute shaft to prismatic driven joint through a pulley of the given pitch radius."""

    type: Literal["belt_pulley"]
    pulley_radius: PositiveLength
    stiffness: PositiveStiffness
    backlash: NonNegativeLength = 0.0


class LeadScrew(_Transmission):
    """Revolute shaft to prismatic driven joint; `lead` is travel per shaft revolution."""

    type: Literal["lead_screw"]
    lead: PositiveLength
    stiffness: PositiveStiffness
    backlash: NonNegativeLength = 0.0
    self_locking: Flag = False


class GearPair(_Transmission):
    """Revolute shaft to revolute driven joint; `ratio` is shaft angle over driven angle, its sign the direction."""

    type: Literal["gear_pair"]
    ratio: Annotated[Number, AfterValidator(_nonzero)]
    stiffness: PositiveRotationalStiffness
    backlash: NonNegativeAngle = 0.0


Transmission = Annotated[BeltPulley | LeadScrew | GearPair, Field(discriminator="type")]

DRIVEN_JOINT_TYPES: dict[str, str] = {"belt_pulley": "prismatic", "lead_screw": "prismatic", "gear_pair": "revolute"}


# ===== Signals and structural members ===================================


class SignalInterface(_Model):
    """Names the signal layer reads from physics: shaft load torques (ENG-048) and body poses for sensors."""

    load_torque_joints: tuple[Name, ...] = ()
    pose_bodies: tuple[Name, ...] = ()


class Support(_Model):
    """Support condition at a distance `at` along the member from its root."""

    at: NonNegativeLength
    condition: Literal["clamped", "pinned", "roller"]


Fastener = Annotated[dict[str, JsonValue], Field(min_length=1)]


class StructuralMember(_Model):
    """Entry of the structural member list for integrity (BLD-008, BLD-009); fasteners are opaque data."""

    name: Name
    section: Section
    material: Name
    supports: Annotated[tuple[Support, ...], Field(min_length=1)]
    fasteners: tuple[Fastener, ...]

    @model_validator(mode="after")
    def _supports_on_member(self) -> StructuralMember:
        for support in self.supports:
            if support.at > self.section.length:
                raise ValueError(
                    f"structural member '{self.name}': support at {support.at!r} m lies beyond its length "
                    f"{self.section.length!r} m"
                )
        return self


# ===== Physics model ====================================================


def _duplicates[Key: (str, tuple[str, str])](names: Iterable[Key]) -> list[Key]:
    seen: set[Key] = set()
    repeated: list[Key] = []
    for name in names:
        if name in seen and name not in repeated:
            repeated.append(name)
        seen.add(name)
    return repeated


class PhysicsModel(_Model):
    """The engine-neutral physics model (ADR 0003). Element names share one namespace; "world" is reserved."""

    schema_version: Literal[1]
    design: DesignRef
    gravity: tuple[Acceleration, Acceleration, Acceleration]
    run: RunConfig = Field(default_factory=RunConfig)
    world_shapes: tuple[Shape, ...] = ()
    contacts: ContactModel | None = None
    bodies: Annotated[tuple[Body, ...], Field(min_length=1)]
    constraints: tuple[Constraint, ...] = ()
    lumped_members: tuple[LumpedMember, ...] = ()
    transmissions: tuple[Transmission, ...] = ()
    signals: SignalInterface = Field(default_factory=SignalInterface)
    structural_members: tuple[StructuralMember, ...] = ()

    @model_validator(mode="after")
    def _references(self) -> PhysicsModel:
        problems: list[str] = []
        joints = {body.joint.name: body.joint for body in self.bodies if body.joint is not None}
        element_names = [body.name for body in self.bodies]
        element_names += [body.joint.name for body in self.bodies if body.joint is not None]
        element_names += [constraint.name for constraint in self.constraints]
        element_names += [member.name for member in self.lumped_members]
        element_names += [transmission.name for transmission in self.transmissions]
        for name in _duplicates(element_names):
            problems.append(f"duplicate name '{name}'")
        if WORLD in element_names:
            problems.append(f"name '{WORLD}' is reserved")
        for name in _duplicates(member.name for member in self.structural_members):
            problems.append(f"duplicate structural member '{name}'")
        structural = {member.name for member in self.structural_members}

        declared: set[str] = {WORLD}
        for body in self.bodies:
            if body.parent not in declared:
                problems.append(f"body '{body.name}': parent '{body.parent}' is not world or a body listed before it")
            declared.add(body.name)
        bodies = declared
        for constraint in self.constraints:
            for body_name in (constraint.body_a, constraint.body_b):
                if body_name not in bodies:
                    problems.append(f"constraint '{constraint.name}': unknown body '{body_name}'")
            if constraint.body_a == constraint.body_b:
                problems.append(f"constraint '{constraint.name}': body_a and body_b are the same body")
        for member in self.lumped_members:
            if member.parent not in bodies:
                problems.append(f"lumped member '{member.name}': unknown parent '{member.parent}'")
            if member.structural_member is not None and member.structural_member not in structural:
                problems.append(
                    f"lumped member '{member.name}': unknown structural member '{member.structural_member}'"
                )
        for transmission in self.transmissions:
            problems += _transmission_problems(transmission, joints)
        if self.contacts is not None:
            problems += _contact_problems(self.contacts, bodies)
        for name in _duplicates(self.signals.load_torque_joints):
            problems.append(f"signals: load torque joint '{name}' listed twice")
        for name in _duplicates(self.signals.pose_bodies):
            problems.append(f"signals: pose body '{name}' listed twice")
        for name in self.signals.load_torque_joints:
            if name not in joints:
                problems.append(f"signals: unknown load torque joint '{name}'")
        for name in self.signals.pose_bodies:
            if name not in bodies or name == WORLD:
                problems.append(f"signals: unknown pose body '{name}'")
        if problems:
            raise ValueError("; ".join(problems))
        return self


def _unordered(pair: tuple[str, str]) -> tuple[str, str]:
    return (min(pair), max(pair))


def _contact_problems(contacts: ContactModel, bodies: set[str]) -> list[str]:
    problems: list[str] = []
    for pair in _duplicates(_unordered(entry.materials) for entry in contacts.pairs):
        problems.append(f"contacts: material pair {pair[0]}/{pair[1]} listed twice")
    for pair in _duplicates(_unordered(exclusion) for exclusion in contacts.exclusions):
        problems.append(f"contacts: exclusion {pair[0]}/{pair[1]} listed twice")
    for exclusion in contacts.exclusions:
        for body_name in exclusion:
            if body_name not in bodies:
                problems.append(f"contacts: exclusion names unknown body '{body_name}'")
        if exclusion[0] == exclusion[1]:
            problems.append(f"contacts: exclusion pairs body '{exclusion[0]}' with itself")
    return problems


def _transmission_problems(transmission: BeltPulley | LeadScrew | GearPair, joints: dict[str, Joint]) -> list[str]:
    problems: list[str] = []
    label = f"transmission '{transmission.name}'"
    shaft = joints.get(transmission.shaft_joint)
    driven = joints.get(transmission.driven_joint)
    if shaft is None:
        problems.append(f"{label}: unknown shaft joint '{transmission.shaft_joint}'")
    elif shaft.type != "revolute":
        problems.append(f"{label}: shaft joint '{shaft.name}' must be revolute, is {shaft.type}")
    if driven is None:
        problems.append(f"{label}: unknown driven joint '{transmission.driven_joint}'")
    elif driven.type != DRIVEN_JOINT_TYPES[transmission.type]:
        expected = DRIVEN_JOINT_TYPES[transmission.type]
        problems.append(f"{label}: driven joint '{driven.name}' must be {expected}, is {driven.type}")
    if transmission.shaft_joint == transmission.driven_joint:
        problems.append(f"{label}: shaft and driven joint are the same joint")
    return problems


# ===== Files and schema =================================================


def from_yaml(text: str) -> PhysicsModel:
    """Parse and validate a physics model YAML document."""
    return PhysicsModel.model_validate(YAML(typ="safe", pure=True).load(text))


def to_yaml(model: PhysicsModel) -> str:
    """Serialize with SI unit strings; the same model always gives the same text."""
    stream = io.StringIO()
    yaml = YAML(typ="safe", pure=True)
    yaml.default_flow_style = False
    yaml.dump(model.model_dump(mode="json"), stream)
    return stream.getvalue()


class _SchemaGenerator(GenerateJsonSchema):
    """Writes field defaults as files carry them: quantities as SI unit strings."""

    def default_schema(self, schema: core_schema.WithDefaultSchema) -> JsonSchemaValue:
        result = super().default_schema(schema)
        if "default" in result:
            result["default"] = SchemaSerializer(schema["schema"]).to_python(schema["default"], mode="json")
        return result


def schema_text() -> str:
    """The JSON Schema of the physics model file, as committed in `schemas/physics_model.json`."""
    schema = PhysicsModel.model_json_schema(schema_generator=_SchemaGenerator)
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["$id"] = SCHEMA_ID
    return json.dumps(schema, indent=2, sort_keys=True) + "\n"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m mechsim.physics.neutral")
    parser.add_argument("--write-schema", action="store_true", help=f"write {SCHEMA_PATH.name}")
    arguments = parser.parse_args(argv)
    if not arguments.write_schema:
        parser.print_help()
        return 2
    SCHEMA_PATH.write_text(schema_text(), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
