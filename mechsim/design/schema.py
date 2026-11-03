"""Design files (DES). Generates `schemas/design.json`.

A design file declares bodies, joints, component instances, the pin-binding table, power rails, environment, firmware
and the operator message set. Quantities are unit strings held as SI floats. A draft design may leave values unset as
`${name}` references to its declared open parameters; a release design may not.
"""

from __future__ import annotations

import argparse
import functools
import hashlib
import json
import math
import re
import sys
from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Annotated, Any, Literal

from pydantic import (
    AfterValidator,
    BaseModel,
    BeforeValidator,
    ConfigDict,
    Field,
    GetCoreSchemaHandler,
    GetJsonSchemaHandler,
    ValidationError,
    ValidationInfo,
    WithJsonSchema,
    field_validator,
    model_validator,
)
from pydantic.json_schema import JsonSchemaValue
from pydantic_core import CoreSchema, core_schema
from ruamel.yaml import YAML

from mechsim.library.registry import Registry, RegistryError, default_registry
from mechsim.library.schema import QuantityParameter, dimensions_equal
from mechsim.materials.schema import DATA_DIR, Material, load_material_file
from mechsim.units import parse_quantity

SCHEMA_PATH = Path(__file__).resolve().parents[2] / "schemas" / "design.json"

QUANTITY_PATTERN = r"^\s*[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?(\s+\S.*)?$"
OPEN_PATTERN = r"^\$\{[a-z][a-z0-9_]*\}$"
CHOICE = "choice"
WORLD = "world"
CONTROLLER = "controller"
OPERATOR = "operator"
RESERVED_MESSAGES = ("heartbeat",)

_OPEN = re.compile(r"^\$\{([a-z][a-z0-9_]*)\}$")
_QUANTITY_OR_OPEN_SCHEMA = {
    "anyOf": [{"type": "string", "pattern": QUANTITY_PATTERN}, {"type": "string", "pattern": OPEN_PATTERN}]
}

Name = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_]*$")]
SignalName = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_]*\.[a-z][a-z0-9_]*$")]
SemVer = Annotated[str, Field(pattern=r"^\d+\.\d+\.\d+$")]
McuPin = Annotated[str, Field(pattern=r"^[A-Z][A-Z0-9_]*$")]


def _no_parent_segment(value: str) -> str:
    if ".." in value.split("/"):
        raise ValueError(f"path {value!r} may not leave its folder with '..'")
    return value


RepoPath = Annotated[str, Field(pattern=r"^[A-Za-z0-9_.\-]+(/[A-Za-z0-9_.\-]+)*$"), AfterValidator(_no_parent_segment)]

JointType = Literal["revolute", "prismatic", "fixed", "planar", "spherical", "free"]
DOF_COUNTS: dict[str, tuple[int, int]] = {
    "revolute": (0, 1),
    "prismatic": (1, 0),
    "fixed": (0, 0),
    "planar": (2, 1),
    "spherical": (0, 3),
    "free": (3, 3),
}
AXIS_JOINTS = ("revolute", "prismatic", "planar")
UNLIMITED_JOINTS = ("spherical", "free")

PeripheralMode = Literal["gpio_input", "gpio_output", "timer_pwm", "timer_encoder", "adc", "dac", "uart", "spi", "i2c"]
READ_MODES = ("gpio_input", "timer_encoder", "adc", "uart", "spi", "i2c")
WRITE_MODES = ("gpio_output", "timer_pwm", "dac", "uart", "spi", "i2c")
EMIT_PIN_DIRECTIONS = ("output", "bidirectional")
CONSUME_PIN_DIRECTIONS = ("input", "bidirectional")


class DesignError(ValueError):
    """A design file failed to load; the message names the file, the path within it and the reason."""


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)


# ===== Open values =====================================================


class OpenValue:
    """A `${name}` stand-in for a value a draft design has not set; `expected` is a pint dimension or "choice"."""

    __slots__ = ("expected", "name")

    def __init__(self, name: str, expected: str) -> None:
        self.name = name
        self.expected = expected

    def __str__(self) -> str:
        return "${" + self.name + "}"

    def __repr__(self) -> str:
        return f"OpenValue({self.name!r}, {self.expected!r})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, OpenValue):
            return NotImplemented
        return self.name == other.name and self.expected == other.expected

    def __hash__(self) -> int:
        return hash((self.name, self.expected))

    @classmethod
    def __get_pydantic_core_schema__(cls, source: Any, handler: GetCoreSchemaHandler) -> CoreSchema:
        return core_schema.is_instance_schema(cls, serialization=core_schema.plain_serializer_function_ser_schema(str))

    @classmethod
    def __get_pydantic_json_schema__(cls, schema: CoreSchema, handler: GetJsonSchemaHandler) -> JsonSchemaValue:
        return {"type": "string", "pattern": OPEN_PATTERN}


def open_name(value: Any) -> str | None:
    if not isinstance(value, str):
        return None
    match = _OPEN.match(value.strip())
    if match is None:
        return None
    return match.group(1)


def _quantity(dimension: str) -> Callable[[Any], Any]:
    def _parse(value: Any) -> Any:
        name = open_name(value)
        if name is not None:
            return OpenValue(name, dimension)
        return parse_quantity(value, dimension)

    return _parse


def _choice(value: Any) -> Any:
    name = open_name(value)
    if name is not None:
        return OpenValue(name, CHOICE)
    return value


_QS = WithJsonSchema(_QUANTITY_OR_OPEN_SCHEMA)

Length = Annotated[float | OpenValue, BeforeValidator(_quantity("[length]")), _QS]
Mass = Annotated[float | OpenValue, BeforeValidator(_quantity("[mass]")), _QS]
Time = Annotated[float | OpenValue, BeforeValidator(_quantity("[time]")), _QS]
Angle = Annotated[float | OpenValue, BeforeValidator(_quantity("dimensionless")), _QS]
Fraction = Annotated[float | OpenValue, BeforeValidator(_quantity("dimensionless")), _QS]
Force = Annotated[float | OpenValue, BeforeValidator(_quantity("[mass] * [length] / [time] ** 2")), _QS]
Torque = Annotated[float | OpenValue, BeforeValidator(_quantity("[mass] * [length] ** 2 / [time] ** 2")), _QS]
LinearDamping = Annotated[float | OpenValue, BeforeValidator(_quantity("[mass] / [time]")), _QS]
AngularDamping = Annotated[float | OpenValue, BeforeValidator(_quantity("[mass] * [length] ** 2 / [time]")), _QS]
Acceleration = Annotated[float | OpenValue, BeforeValidator(_quantity("[length] / [time] ** 2")), _QS]
Frequency = Annotated[float | OpenValue, BeforeValidator(_quantity("1 / [time]")), _QS]
Inertia = Annotated[float | OpenValue, BeforeValidator(_quantity("[mass] * [length] ** 2")), _QS]
Area = Annotated[float | OpenValue, BeforeValidator(_quantity("[length] ** 2")), _QS]
SecondMomentOfArea = Annotated[float | OpenValue, BeforeValidator(_quantity("[length] ** 4")), _QS]
Temperature = Annotated[float | OpenValue, BeforeValidator(_quantity("[temperature]")), _QS]

OpenName = Annotated[Name | OpenValue, BeforeValidator(_choice)]


def _is_set(value: Any) -> bool:
    return isinstance(value, float)


def _require_positive(value: Any, what: str) -> None:
    if _is_set(value) and value <= 0.0:
        raise ValueError(f"{what} must be positive, got {value}")


def _require_non_negative(value: Any, what: str) -> None:
    if _is_set(value) and value < 0.0:
        raise ValueError(f"{what} must not be negative, got {value}")


def _unit_vector(value: list[float], what: str) -> list[float]:
    norm = math.sqrt(sum(component * component for component in value))
    if abs(norm - 1.0) > 1e-6:
        raise ValueError(f"{what} must be a unit vector, norm is {norm}")
    return value


UnitVector = Annotated[list[float], Field(min_length=3, max_length=3)]
Quaternion = Annotated[list[float], Field(min_length=4, max_length=4, description="w, x, y, z")]
LengthVector = Annotated[list[Length], Field(min_length=3, max_length=3)]


class OpenParameter(_Strict):
    name: Name
    dimension: str = Field(min_length=1, description='pint dimensionality string, or "choice" for a non-quantity')
    owed_by: str = Field(min_length=1)
    note: str = Field(min_length=1)

    @field_validator("dimension")
    @classmethod
    def _known(cls, value: str) -> str:
        if value == CHOICE:
            return value
        try:
            dimensions_equal(value, value)
        except Exception as error:
            raise ValueError(f"unknown dimension {value!r}") from error
        return value


# ===== Bodies ==========================================================


class PrintedProcess(_Strict):
    kind: Literal["printed"]
    build_direction: UnitVector
    walls: int = Field(ge=1)
    infill: Fraction

    @field_validator("build_direction")
    @classmethod
    def _unit(cls, value: list[float]) -> list[float]:
        return _unit_vector(value, "build_direction")

    @field_validator("infill")
    @classmethod
    def _infill(cls, value: Any) -> Any:
        if _is_set(value) and (value < 0.0 or value > 1.0):
            raise ValueError(f"infill must lie in 0 to 100 percent, got {value * 100.0} percent")
        return value


class MachinedProcess(_Strict):
    kind: Literal["machined"]


class MoldedProcess(_Strict):
    kind: Literal["molded"]


class ExtrusionProcess(_Strict):
    kind: Literal["extrusion"]


Process = Annotated[
    Annotated[PrintedProcess | MachinedProcess | MoldedProcess | ExtrusionProcess, Field(discriminator="kind")]
    | OpenValue,
    BeforeValidator(_choice),
]


class _Placed(_Strict):
    position: LengthVector
    orientation: Quaternion = Field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])

    @field_validator("orientation")
    @classmethod
    def _unit_quaternion(cls, value: list[float]) -> list[float]:
        norm = math.sqrt(sum(component * component for component in value))
        if abs(norm - 1.0) > 1e-9:
            raise ValueError(f"orientation must be a unit quaternion (w first), norm is {norm}")
        return value


class Sphere(_Placed):
    shape: Literal["sphere"]
    radius: Length

    @model_validator(mode="after")
    def _positive(self) -> Sphere:
        _require_positive(self.radius, "sphere radius")
        return self


class Box(_Placed):
    shape: Literal["box"]
    size: LengthVector

    @model_validator(mode="after")
    def _positive(self) -> Box:
        for component in self.size:
            _require_positive(component, "box size")
        return self


class Cylinder(_Placed):
    shape: Literal["cylinder"]
    radius: Length
    length: Length

    @model_validator(mode="after")
    def _positive(self) -> Cylinder:
        _require_positive(self.radius, "cylinder radius")
        _require_positive(self.length, "cylinder length")
        return self


class Capsule(_Placed):
    shape: Literal["capsule"]
    radius: Length
    length: Length

    @model_validator(mode="after")
    def _positive(self) -> Capsule:
        _require_positive(self.radius, "capsule radius")
        _require_positive(self.length, "capsule length")
        return self


Shape = Annotated[Sphere | Box | Cylinder | Capsule, Field(discriminator="shape")]


class PrimitiveGeometry(_Strict):
    kind: Literal["primitives"]
    shapes: list[Shape] = Field(min_length=1)


class StepGeometry(_Strict):
    """A STEP file, resolved relative to the design file; the body's declared mass properties are checked against it."""

    kind: Literal["step"]
    file: RepoPath
    mass_check_tolerance: Fraction

    @field_validator("file")
    @classmethod
    def _step_suffix(cls, value: str) -> str:
        if not value.lower().endswith((".step", ".stp")):
            raise ValueError(f"STEP file {value!r} must end in .step or .stp")
        return value

    @model_validator(mode="after")
    def _positive(self) -> StepGeometry:
        _require_positive(self.mass_check_tolerance, "mass_check_tolerance")
        return self


Geometry = Annotated[
    Annotated[PrimitiveGeometry | StepGeometry, Field(discriminator="kind")] | OpenValue,
    BeforeValidator(_choice),
]


class InertiaTensor(_Strict):
    """About the center of mass, in the body frame."""

    ixx: Inertia
    iyy: Inertia
    izz: Inertia
    ixy: Inertia = 0.0
    ixz: Inertia = 0.0
    iyz: Inertia = 0.0

    @model_validator(mode="after")
    def _diagonal(self) -> InertiaTensor:
        for name in ("ixx", "iyy", "izz"):
            _require_positive(getattr(self, name), name)
        return self


class MassProperties(_Strict):
    mass: Mass
    com: LengthVector
    inertia: InertiaTensor

    @model_validator(mode="after")
    def _positive(self) -> MassProperties:
        _require_positive(self.mass, "mass")
        return self


class Section(_Strict):
    """Cross-section and span; E and rho come from the body's material."""

    area: Area
    second_moment: SecondMomentOfArea
    length: Length

    @model_validator(mode="after")
    def _positive(self) -> Section:
        _require_positive(self.area, "section area")
        _require_positive(self.second_moment, "section second_moment")
        _require_positive(self.length, "section length")
        return self


class Calibration(_Strict):
    """The known deflection under `load` and first natural frequency the lumped member is calibrated to."""

    source: Literal["hand", "fea"]
    load: Force
    deflection: Length
    first_natural_frequency: Frequency
    design_version: SemVer | None = None

    @model_validator(mode="after")
    def _consistent(self) -> Calibration:
        _require_positive(self.load, "calibration load")
        _require_positive(self.deflection, "calibration deflection")
        _require_positive(self.first_natural_frequency, "first_natural_frequency")
        if self.source == "fea" and self.design_version is None:
            raise ValueError("an FEA calibration must name the design_version it belongs to")
        return self


SupportCondition = Literal["clamped", "pinned", "roller", "free"]


class FlexibleMember(_Strict):
    section: Section
    calibration: Annotated[Calibration | OpenValue, BeforeValidator(_choice)]
    supports: list[SupportCondition] = Field(min_length=2, max_length=2, description="root, end")
    segments: int = Field(default=4, ge=1)


class Fastener(_Strict):
    kind: Name
    count: int = Field(ge=1)
    grade: OpenName
    preload: Force

    @model_validator(mode="after")
    def _preload(self) -> Fastener:
        _require_non_negative(self.preload, "fastener preload")
        return self


class CatalogPartReference(_Strict):
    type: Name
    part: Name


class Body(_Strict):
    name: Name
    material: OpenName
    process: Process
    geometry: Geometry
    mass_properties: Annotated[MassProperties | OpenValue | None, BeforeValidator(_choice)] = None
    catalog_part: CatalogPartReference | None = None
    flexible: FlexibleMember | None = None
    fasteners: list[Fastener] = Field(default_factory=list)

    @model_validator(mode="after")
    def _declared_mass(self) -> Body:
        if isinstance(self.geometry, StepGeometry) and self.mass_properties is None:
            raise ValueError(f"body {self.name!r}: STEP geometry needs declared mass_properties to check against")
        return self


# ===== Joints ==========================================================


class LinearLimits(_Strict):
    lower: Length
    upper: Length

    @model_validator(mode="after")
    def _ordered(self) -> LinearLimits:
        if isinstance(self.lower, float) and isinstance(self.upper, float) and self.lower > self.upper:
            raise ValueError(f"limit lower {self.lower} exceeds upper {self.upper}")
        return self


class AngularLimits(_Strict):
    lower: Angle
    upper: Angle

    @model_validator(mode="after")
    def _ordered(self) -> AngularLimits:
        if isinstance(self.lower, float) and isinstance(self.upper, float) and self.lower > self.upper:
            raise ValueError(f"limit lower {self.lower} exceeds upper {self.upper}")
        return self


class LinearDof(_Strict):
    limits: LinearLimits | None = None
    friction: Force
    damping: LinearDamping
    backlash: Length

    @model_validator(mode="after")
    def _non_negative(self) -> LinearDof:
        _require_non_negative(self.friction, "friction")
        _require_non_negative(self.damping, "damping")
        _require_non_negative(self.backlash, "backlash")
        return self


class AngularDof(_Strict):
    limits: AngularLimits | None = None
    friction: Torque
    damping: AngularDamping
    backlash: Angle

    @model_validator(mode="after")
    def _non_negative(self) -> AngularDof:
        _require_non_negative(self.friction, "friction")
        _require_non_negative(self.damping, "damping")
        _require_non_negative(self.backlash, "backlash")
        return self


class Placement(_Placed):
    """The joint frame in the parent body's frame."""


class Joint(_Strict):
    name: Name
    type: JointType
    parent: Name
    child: Name
    placement: Annotated[Placement | OpenValue, BeforeValidator(_choice)]
    axis: Annotated[UnitVector | OpenValue | None, BeforeValidator(_choice)] = None
    linear: list[LinearDof] = Field(default_factory=list)
    angular: list[AngularDof] = Field(default_factory=list)

    @model_validator(mode="after")
    def _dofs(self) -> Joint:
        linear_count, angular_count = DOF_COUNTS[self.type]
        if len(self.linear) != linear_count or len(self.angular) != angular_count:
            raise ValueError(
                f"joint {self.name!r} of type {self.type} needs {linear_count} linear and {angular_count} angular "
                f"degrees of freedom, got {len(self.linear)} and {len(self.angular)}"
            )
        if self.type in AXIS_JOINTS and self.axis is None:
            raise ValueError(f"joint {self.name!r} of type {self.type} needs an axis")
        if self.type not in AXIS_JOINTS and self.axis is not None:
            raise ValueError(f"joint {self.name!r} of type {self.type} takes no axis")
        if isinstance(self.axis, list):
            _unit_vector(self.axis, f"joint {self.name!r} axis")
        if self.type in UNLIMITED_JOINTS:
            limited = [dof for dof in self.linear if dof.limits is not None]
            limited_angular = [dof for dof in self.angular if dof.limits is not None]
            if len(limited) > 0 or len(limited_angular) > 0:
                raise ValueError(f"joint {self.name!r} of type {self.type} takes no limits")
        if self.parent == self.child:
            raise ValueError(f"joint {self.name!r} joins body {self.child!r} to itself")
        return self


# ===== Components ======================================================


class MechanicalBinding(_Strict):
    """A mount of the instance's type bound to a body (housing, face) or a joint (shaft)."""

    mount: Name
    body: OpenName | None = None
    joint: OpenName | None = None

    @model_validator(mode="after")
    def _one_target(self) -> MechanicalBinding:
        if (self.body is None) == (self.joint is None):
            raise ValueError(f"mount {self.mount!r} binds exactly one of body or joint")
        return self


class SignalDeclaration(_Strict):
    name: SignalName
    pin: Name
    direction: Literal["emits", "consumes"]


class ComponentInstance(_Strict):
    name: Name
    type: Name
    part: Name
    overrides: dict[Name, str | bool] = Field(default_factory=dict)
    mechanical: list[MechanicalBinding] = Field(default_factory=list)
    signals: list[SignalDeclaration] = Field(default_factory=list)


class PinBinding(_Strict):
    signal: SignalName
    pin: McuPin
    mode: PeripheralMode


# ===== Power, environment, firmware, messages ==========================


class Rail(_Strict):
    name: Name
    supply: OpenName
    gated_by: list[OpenName] = Field(default_factory=list)


class Power(_Strict):
    rails: list[Rail] = Field(min_length=1)
    controller_rail: Name


class Environment(_Strict):
    gravity: Annotated[list[Acceleration], Field(min_length=3, max_length=3)]
    ambient_temperature: Temperature

    @model_validator(mode="after")
    def _above_zero(self) -> Environment:
        _require_positive(self.ambient_temperature, "ambient_temperature")
        return self


class Firmware(_Strict):
    path: RepoPath
    chip: OpenName
    tick_rate: Frequency

    @model_validator(mode="after")
    def _positive(self) -> Firmware:
        _require_positive(self.tick_rate, "tick_rate")
        return self


class MessageField(_Strict):
    name: Name
    dimension: str = Field(min_length=1)

    @field_validator("dimension")
    @classmethod
    def _known(cls, value: str) -> str:
        try:
            dimensions_equal(value, value)
        except Exception as error:
            raise ValueError(f"unknown dimension {value!r}") from error
        return value


class OperatorMessage(_Strict):
    name: Name
    label: str = Field(min_length=1)
    source: Name
    destination: Name
    fields: list[MessageField] = Field(default_factory=list)

    @model_validator(mode="after")
    def _controller_end(self) -> OperatorMessage:
        if self.name in RESERVED_MESSAGES:
            raise ValueError(
                f"message {self.name!r} is reserved: the operator link carries operator traffic only, and the "
                "heartbeat is a pin signal through the pin-binding table"
            )
        if (self.source == CONTROLLER) == (self.destination == CONTROLLER):
            raise ValueError(f"message {self.name!r}: exactly one of source and destination is the controller")
        return self


class DesignLoadRule(_Strict):
    """Design load = rated_load * (1 + allowance)."""

    rated_load: Mass
    allowance: Fraction
    note: str | None = None

    @model_validator(mode="after")
    def _bounds(self) -> DesignLoadRule:
        _require_positive(self.rated_load, "rated_load")
        _require_non_negative(self.allowance, "allowance")
        return self


class Structural(_Strict):
    design_load: DesignLoadRule
    accuracy_target: Length
    deflection_target: Annotated[Literal["static", "dynamic"] | OpenValue, BeforeValidator(_choice)]
    minimum_safety_factor: Fraction | None = None

    @model_validator(mode="after")
    def _bounds(self) -> Structural:
        _require_positive(self.accuracy_target, "accuracy_target")
        _require_positive(self.minimum_safety_factor, "minimum_safety_factor")
        return self


# ===== Design ==========================================================


class Design(_Strict):
    """Root of `designs/<name>/design.yaml`."""

    model_config = ConfigDict(extra="forbid", strict=True, title="DesignFile")

    name: Name
    version: SemVer
    source_hash: Annotated[str, Field(pattern=r"^([0-9a-f]{40}|unset)$")]
    status: Literal["draft", "release"]
    description: str | None = None
    open_parameters: list[OpenParameter] = Field(default_factory=list)
    environment: Environment
    bodies: list[Body] = Field(min_length=1)
    joints: list[Joint] = Field(min_length=1)
    components: list[ComponentInstance] = Field(default_factory=list)
    pin_bindings: list[PinBinding] = Field(default_factory=list)
    power: Power | None = None
    firmware: Firmware | None = None
    operator_messages: list[OperatorMessage] = Field(default_factory=list)
    structural: Structural | None = None

    @model_validator(mode="after")
    def _cross_check(self, info: ValidationInfo) -> Design:
        registry = default_registry()
        context = info.context
        if isinstance(context, dict) and isinstance(context.get("registry"), Registry):
            registry = context["registry"]
        errors = cross_check(self, registry, shipped_materials())
        if isinstance(context, dict) and isinstance(context.get("base"), Path):
            errors.extend(check_files(self, context["base"]))
        if len(errors) > 0:
            raise ValueError("\n".join(errors))
        return self

    def signal_names(self) -> list[str]:
        return [signal.name for component in self.components for signal in component.signals]


# ===== Cross-reference checks ==========================================


@functools.cache
def shipped_materials() -> dict[str, Material]:
    materials: dict[str, Material] = {}
    for path in sorted(DATA_DIR.glob("*.yaml")):
        material = load_material_file(path)
        materials[material.name] = material
    return materials


def _duplicates(values: list[str], what: str, errors: list[str]) -> None:
    seen: set[str] = set()
    for value in values:
        if value in seen:
            errors.append(f"duplicate {what} {value!r}")
        seen.add(value)


def iter_open_values(value: Any, path: str) -> Iterator[tuple[str, OpenValue]]:
    if isinstance(value, OpenValue):
        yield path, value
        return
    if isinstance(value, BaseModel):
        for field in type(value).model_fields:
            yield from iter_open_values(getattr(value, field), f"{path}.{field}")
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            yield from iter_open_values(item, f"{path}[{index}]")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            yield from iter_open_values(item, f"{path}.{key}")


def _check_open(design: Design, errors: list[str]) -> dict[str, OpenParameter]:
    declared = {parameter.name: parameter for parameter in design.open_parameters}
    _duplicates([parameter.name for parameter in design.open_parameters], "open parameter", errors)
    if design.status == "release" and len(design.open_parameters) > 0:
        names = ", ".join(parameter.name for parameter in design.open_parameters)
        errors.append(f"open_parameters: a release design may not have open parameters ({names})")
    for path, value in iter_open_values(design, "design"):
        _check_reference(path, value.name, value.expected, declared, errors)
    return declared


def _check_reference(
    path: str, name: str, expected: str, declared: dict[str, OpenParameter], errors: list[str]
) -> None:
    if name not in declared:
        errors.append(f"{path}: references undeclared open parameter ${{{name}}}")
        return
    dimension = declared[name].dimension
    if expected == CHOICE or dimension == CHOICE:
        if expected != dimension:
            errors.append(f"{path}: open parameter {name!r} has dimension {dimension}, expected {expected}")
        return
    if not dimensions_equal(dimension, expected):
        errors.append(f"{path}: open parameter {name!r} has dimension {dimension}, expected {expected}")


def cross_check(design: Design, registry: Registry, materials: dict[str, Material]) -> list[str]:
    """Every reference and uniqueness error in `design`, each with its path (DES-016)."""
    errors: list[str] = []
    declared = _check_open(design, errors)
    bodies = [body.name for body in design.bodies]
    joints = [joint.name for joint in design.joints]
    instances = [component.name for component in design.components]
    _duplicates([*bodies, *joints, *instances], "body, joint or component name", errors)
    for index, body in enumerate(design.bodies):
        _check_body(f"bodies[{index}]", body, registry, materials, errors)
    if WORLD in bodies:
        errors.append(f"bodies: {WORLD!r} is reserved")
    _check_joints(design, errors)
    for index, component in enumerate(design.components):
        _check_component(f"components[{index}]", component, design, registry, declared, errors)
    _check_signals(design, errors)
    _check_power(design, registry, errors)
    _check_messages(design, errors)
    return errors


def _check_material(path: str, material: Any, materials: dict[str, Material], errors: list[str]) -> Material | None:
    if isinstance(material, OpenValue):
        return None
    if material not in materials:
        errors.append(f"{path}: unknown material {material!r}")
        return None
    return materials[material]


def _check_body(path: str, body: Body, registry: Registry, materials: dict[str, Material], errors: list[str]) -> None:
    record = _check_material(f"{path}.material", body.material, materials, errors)
    if record is not None and not isinstance(body.process, OpenValue):
        printed_material = record.process is not None and record.process.kind == "printed"
        printed_body = body.process.kind == "printed"
        if printed_material != printed_body:
            errors.append(f"{path}.process: process {body.process.kind!r} does not match material {record.name!r}")
    for fastener_index, fastener in enumerate(body.fasteners):
        _check_material(f"{path}.fasteners[{fastener_index}].grade", fastener.grade, materials, errors)
    if body.catalog_part is not None:
        reference = body.catalog_part
        if not registry.has_type(reference.type):
            errors.append(f"{path}.catalog_part.type: unknown component type {reference.type!r}")
        elif not registry.has_part(reference.part):
            errors.append(f"{path}.catalog_part.part: unknown catalog part {reference.part!r}")
        elif registry.part(reference.part).type != reference.type:
            errors.append(f"{path}.catalog_part.part: part {reference.part!r} is not of type {reference.type!r}")


def _check_joints(design: Design, errors: list[str]) -> None:
    bodies = {body.name for body in design.bodies}
    parent_of: dict[str, str] = {}
    for index, joint in enumerate(design.joints):
        path = f"joints[{index}]"
        if joint.parent != WORLD and joint.parent not in bodies:
            errors.append(f"{path}.parent: unknown body {joint.parent!r}")
        if joint.child not in bodies:
            errors.append(f"{path}.child: unknown body {joint.child!r}")
            continue
        if joint.child in parent_of:
            errors.append(f"{path}.child: body {joint.child!r} is the child of more than one joint")
            continue
        parent_of[joint.child] = joint.parent
    for body in design.bodies:
        if body.name not in parent_of:
            errors.append(f"bodies: body {body.name!r} is the child of no joint")
    for start in parent_of:
        seen = {start}
        current = parent_of[start]
        while current in parent_of:
            if current in seen:
                errors.append(f"joints: body {start!r} is in a joint cycle")
                break
            seen.add(current)
            current = parent_of[current]


def _check_component(
    path: str,
    component: ComponentInstance,
    design: Design,
    registry: Registry,
    declared: dict[str, OpenParameter],
    errors: list[str],
) -> None:
    if not registry.has_type(component.type):
        errors.append(f"{path}.type: unknown component type {component.type!r}")
        return
    kind = registry.component_type(component.type)
    if not registry.has_part(component.part):
        errors.append(f"{path}.part: unknown catalog part {component.part!r}")
        return
    if registry.part(component.part).type != component.type:
        errors.append(f"{path}.part: part {component.part!r} is not of type {component.type!r}")
        return
    concrete: dict[str, str | bool] = {}
    for key, value in component.overrides.items():
        name = open_name(value)
        if name is None:
            concrete[key] = value
            continue
        try:
            parameter = kind.parameter(key)
        except KeyError:
            errors.append(f"{path}.overrides.{key}: type {kind.name!r} has no parameter {key!r}")
            continue
        if not isinstance(parameter, QuantityParameter):
            errors.append(f"{path}.overrides.{key}: flag parameter {key!r} cannot be an open value")
            continue
        _check_reference(f"{path}.overrides.{key}", name, parameter.dimension, declared, errors)
    try:
        registry.resolve_part(component.part, concrete)
    except RegistryError as error:
        errors.append(f"{path}.overrides: {error}")
    mounts = {mount.name: mount for mount in kind.mechanical.mounts}
    bodies = {body.name for body in design.bodies}
    joints = {joint.name for joint in design.joints}
    bound: set[str] = set()
    for binding_index, binding in enumerate(component.mechanical):
        where = f"{path}.mechanical[{binding_index}]"
        if binding.mount not in mounts:
            errors.append(f"{where}.mount: type {kind.name!r} has no mount {binding.mount!r}")
            continue
        if binding.mount in bound:
            errors.append(f"{where}.mount: mount {binding.mount!r} is bound more than once")
        bound.add(binding.mount)
        if mounts[binding.mount].kind == "shaft" and binding.joint is None:
            errors.append(f"{where}: shaft mount {binding.mount!r} must bind a joint")
        if isinstance(binding.body, str) and binding.body not in bodies:
            errors.append(f"{where}.body: unknown body {binding.body!r}")
        if isinstance(binding.joint, str) and binding.joint not in joints:
            errors.append(f"{where}.joint: unknown joint {binding.joint!r}")
    pins = {pin.name: pin for pin in kind.electrical.pins}
    used_pins: set[str] = set()
    for signal_index, signal in enumerate(component.signals):
        where = f"{path}.signals[{signal_index}]"
        if signal.name.split(".")[0] != component.name:
            errors.append(f"{where}.name: signal {signal.name!r} must start with its instance name {component.name!r}")
        if signal.pin not in pins:
            errors.append(f"{where}.pin: type {kind.name!r} has no pin {signal.pin!r}")
            continue
        if signal.pin in used_pins:
            errors.append(f"{where}.pin: pin {signal.pin!r} carries more than one signal")
        used_pins.add(signal.pin)
        allowed = CONSUME_PIN_DIRECTIONS
        if signal.direction == "emits":
            allowed = EMIT_PIN_DIRECTIONS
        if pins[signal.pin].direction not in allowed:
            errors.append(
                f"{where}.direction: pin {signal.pin!r} is {pins[signal.pin].direction} and cannot {signal.direction}"
            )


def _check_signals(design: Design, errors: list[str]) -> None:
    declared: dict[str, tuple[str, str]] = {}
    for index, component in enumerate(design.components):
        for signal_index, signal in enumerate(component.signals):
            where = f"components[{index}].signals[{signal_index}]"
            if signal.name in declared:
                errors.append(
                    f"{where}: duplicate signal {signal.name!r}, first declared at {declared[signal.name][0]}"
                )
                continue
            declared[signal.name] = (where, signal.direction)
    bound: dict[str, str] = {}
    pins: dict[str, str] = {}
    for index, binding in enumerate(design.pin_bindings):
        where = f"pin_bindings[{index}]"
        if binding.signal not in declared:
            errors.append(f"{where}.signal: unknown signal {binding.signal!r}")
        elif binding.signal in bound:
            errors.append(
                f"{where}.signal: signal {binding.signal!r} is bound more than once, first at {bound[binding.signal]}"
            )
        else:
            bound[binding.signal] = where
            allowed = WRITE_MODES
            if declared[binding.signal][1] == "emits":
                allowed = READ_MODES
            if binding.mode not in allowed:
                errors.append(
                    f"{where}.mode: mode {binding.mode!r} does not suit signal {binding.signal!r}, which "
                    f"{declared[binding.signal][1]}"
                )
        if binding.pin in pins:
            errors.append(f"{where}.pin: MCU pin {binding.pin!r} is bound more than once, first at {pins[binding.pin]}")
        else:
            pins[binding.pin] = where
    for name, (where, _direction) in declared.items():
        if name not in bound:
            errors.append(f"{where}: signal {name!r} has no pin binding")
    if (len(design.pin_bindings) > 0 or len(declared) > 0) and design.firmware is None:
        errors.append("firmware: a design with controller-facing signals must declare its firmware and chip")


def _check_power(design: Design, registry: Registry, errors: list[str]) -> None:
    if design.power is None:
        return
    categories: dict[str, str] = {}
    for component in design.components:
        if registry.has_type(component.type):
            categories[component.name] = registry.component_type(component.type).category
    rails = [rail.name for rail in design.power.rails]
    _duplicates(rails, "rail", errors)
    for index, rail in enumerate(design.power.rails):
        where = f"power.rails[{index}]"
        if isinstance(rail.supply, str):
            if rail.supply not in categories:
                errors.append(f"{where}.supply: unknown component instance {rail.supply!r}")
            elif categories[rail.supply] != "power":
                errors.append(f"{where}.supply: instance {rail.supply!r} is not a power component")
        for gate_index, gate in enumerate(rail.gated_by):
            if not isinstance(gate, str):
                continue
            if gate not in categories:
                errors.append(f"{where}.gated_by[{gate_index}]: unknown component instance {gate!r}")
            elif categories[gate] != "safety":
                errors.append(f"{where}.gated_by[{gate_index}]: instance {gate!r} is not a safety component")
    if design.power.controller_rail not in rails:
        errors.append(f"power.controller_rail: unknown rail {design.power.controller_rail!r}")


def _check_messages(design: Design, errors: list[str]) -> None:
    _duplicates([message.name for message in design.operator_messages], "operator message", errors)
    _duplicates([message.label for message in design.operator_messages], "operator message label", errors)
    instances = {component.name for component in design.components}
    for index, message in enumerate(design.operator_messages):
        for end in ("source", "destination"):
            value = getattr(message, end)
            if value not in (CONTROLLER, OPERATOR) and value not in instances:
                errors.append(f"operator_messages[{index}].{end}: unknown endpoint {value!r}")


# ===== Files and schema ================================================


def yaml_loader() -> YAML:
    """Round-trip YAML (L6) with the indentation the shipped files use, so a rewrite keeps them byte-identical."""
    yaml = YAML(typ="rt")
    yaml.indent(mapping=2, sequence=4, offset=2)
    yaml.width = 120
    yaml.preserve_quotes = True
    return yaml


def blob_hash(path: Path) -> str:
    """The git blob hash of the file (D3), computed without git."""
    content = path.read_bytes()
    header = f"blob {len(content)}\0".encode()
    return hashlib.sha1(header + content, usedforsecurity=False).hexdigest()


def _location_part(part: str | int) -> str | None:
    if isinstance(part, int):
        return str(part)
    if part[:1].isupper() or "[" in part or "-" in part:
        return None
    return part


def _format_errors(error: ValidationError) -> list[str]:
    """One line per error, union branch names dropped from the path; the `${name}` branch's own error is dropped."""
    messages: list[str] = []
    for item in error.errors():
        if any(isinstance(part, str) and part.startswith("is-instance[OpenValue]") for part in item["loc"]):
            continue
        parts = [_location_part(part) for part in item["loc"]]
        location = ".".join(part for part in parts if part is not None)
        text = str(item["msg"]).removeprefix("Value error, ")
        for line in text.split("\n"):
            message = line
            if location != "":
                message = f"{location}: {line}"
            if message not in messages:
                messages.append(message)
    return messages


def check_files(design: Design, base: Path) -> list[str]:
    """Every CAD file the design references must exist relative to the design file's folder (DES-027)."""
    errors: list[str] = []
    for index, body in enumerate(design.bodies):
        if isinstance(body.geometry, StepGeometry) and not (base / body.geometry.file).is_file():
            errors.append(f"bodies[{index}].geometry.file: missing CAD file {body.geometry.file!r}")
    return errors


def _load(path: Path, registry: Registry | None) -> tuple[Design | None, list[str]]:
    """Parse and validate once; model, reference and CAD-file errors come back together, each prefixed by the file."""
    try:
        data = yaml_loader().load(path.read_text(encoding="utf-8"))
    except Exception as error:
        return None, [f"{path}: YAML parse error: {error}"]
    context: dict[str, Any] = {"base": path.parent}
    if registry is not None:
        context["registry"] = registry
    try:
        design = Design.model_validate(data, context=context)
    except ValidationError as error:
        return None, [f"{path}: {message}" for message in _format_errors(error)]
    return design, []


def validate_design_file(path: Path, registry: Registry | None = None) -> list[str]:
    """Every error in the design file at `path`, each prefixed with the file; empty when it is valid (DES-036)."""
    return _load(path, registry)[1]


def load_design(path: Path, registry: Registry | None = None) -> Design:
    design, errors = _load(path, registry)
    if design is None:
        raise DesignError("\n".join(errors))
    return design


def design_identity(path: Path) -> tuple[str, str]:
    """The design's semver version and the file's git blob hash, as every archive records them (DES-008)."""
    return load_design(path).version, blob_hash(path)


def design_json_schema() -> dict[str, Any]:
    schema = Design.model_json_schema()
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["$id"] = SCHEMA_PATH.name
    return schema


def schema_text() -> str:
    return json.dumps(design_json_schema(), indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Design file schema")
    parser.add_argument("--write-schema", action="store_true", help=f"write {SCHEMA_PATH}")
    parser.add_argument("--validate", type=Path, nargs="*", default=[], help="design files to validate")
    arguments = parser.parse_args(argv)
    if arguments.write_schema:
        SCHEMA_PATH.write_text(schema_text(), encoding="utf-8")
        return 0
    if len(arguments.validate) > 0:
        status = 0
        for path in arguments.validate:
            for message in validate_design_file(path):
                sys.stderr.write(message + "\n")
                status = 1
        return status
    sys.stdout.write(schema_text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
