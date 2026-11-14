"""Design files (DES). Generates `schemas/design.json`.

A design file declares bodies, joints, component instances, the wiring between them, the pin-binding table, power
rails, environment (with its contact model), firmware and its chip, and the operator message set. Quantities are unit
strings held as SI floats. A draft design may leave values unset as `${name}` references to its declared open
parameters; a release design may not.
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
from dataclasses import dataclass
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
    PrivateAttr,
    ValidationError,
    ValidationInfo,
    WithJsonSchema,
    field_validator,
    model_validator,
)
from pydantic.json_schema import JsonSchemaValue
from pydantic_core import CoreSchema, core_schema
from ruamel.yaml import YAML

from mechsim.design.chip import Chip, PeripheralMode, shipped_chips
from mechsim.library.registry import Registry, RegistryError, default_registry
from mechsim.library.schema import ROPE_PARAMETERS, SECTION_PARAMETERS, QuantityParameter, dimensions_equal
from mechsim.materials.database import MaterialSet
from mechsim.materials.schema import DATA_DIR, Material, MaterialError, load_material_file
from mechsim.schema_base import StrictModel, format_errors
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
PortName = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_]*\.[a-z][a-z0-9_]*$")]


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

READ_MODES: tuple[str, ...] = ("digital_input", "counter_input", "adc_input", "uart_rx")
WRITE_MODES: tuple[str, ...] = ("digital_output", "pulse_output", "uart_tx")
EMIT_PIN_DIRECTIONS = ("output", "bidirectional")
CONSUME_PIN_DIRECTIONS = ("input", "bidirectional")
POWER_PORT_DIRECTIONS = ("power_in", "power_out", "ground")
WIRE_DIRECTIONS: dict[str, tuple[str, ...]] = {
    "output": ("input", "bidirectional"),
    "bidirectional": ("input", "bidirectional"),
    "power_out": ("power_in",),
    "ground": ("ground",),
}


class DesignError(ValueError):
    """A design file failed to load; the message names the file, the path within it and the reason."""


_Strict = StrictModel
_format_errors = format_errors


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
Stiffness = Annotated[float | OpenValue, BeforeValidator(_quantity("[mass] / [time] ** 2")), _QS]
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
    default: str | dict[str, str] | None = Field(
        default=None,
        description="Value used when neither the case nor --set gives one (DES-044): a quantity string of the "
        "parameter's dimension, or a choice value",
    )

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

    @model_validator(mode="after")
    def _default_fits(self) -> OpenParameter:
        if self.default is None:
            return self
        if isinstance(self.default, dict):
            if self.dimension != CHOICE:
                raise ValueError(f"default of {self.name!r}: a mapping is only a choice value")
            return self
        if len(self.default) == 0:
            raise ValueError(f"default of {self.name!r} is empty")
        if open_name(self.default) is not None:
            raise ValueError(f"default of {self.name!r} may not be an open value")
        if self.dimension != CHOICE:
            try:
                parse_quantity(self.default, self.dimension)
            except Exception as error:
                raise ValueError(f"default of {self.name!r}: {error}") from error
        return self


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


class RectangleShape(_Strict):
    """Solid rectangle: `height` along the bending direction (member y), `width` across it."""

    kind: Literal["rectangle"]
    width: Length
    height: Length


class CircleShape(_Strict):
    kind: Literal["circle"]
    diameter: Length


class TubeShape(_Strict):
    kind: Literal["tube"]
    outer_diameter: Length
    inner_diameter: Length


class RectangularTubeShape(_Strict):
    """Hollow rectangle of uniform `wall`; `height` along the bending direction."""

    kind: Literal["rectangular_tube"]
    width: Length
    height: Length
    wall: Length


SectionShape = Annotated[RectangleShape | CircleShape | TubeShape | RectangularTubeShape, Field(discriminator="kind")]


def shape_area_and_moment(shape: SectionShape) -> tuple[float, float]:
    """Area and second moment of area about the bending axis (member z), SI (DES-033)."""
    if isinstance(shape, RectangleShape):
        width, height = _set_floats(shape.width, shape.height)
        return width * height, width * height**3 / 12.0
    if isinstance(shape, CircleShape):
        (diameter,) = _set_floats(shape.diameter)
        return math.pi * diameter**2 / 4.0, math.pi * diameter**4 / 64.0
    if isinstance(shape, TubeShape):
        outer, inner = _set_floats(shape.outer_diameter, shape.inner_diameter)
        return math.pi * (outer**2 - inner**2) / 4.0, math.pi * (outer**4 - inner**4) / 64.0
    width, height, wall = _set_floats(shape.width, shape.height, shape.wall)
    inner_width = width - 2.0 * wall
    inner_height = height - 2.0 * wall
    area = width * height - inner_width * inner_height
    return area, (width * height**3 - inner_width * inner_height**3) / 12.0


@dataclass(frozen=True)
class SectionProperties:
    """Section properties in the member frame (x along the member, y the bending direction a shape's `height` lies
    along, z across it; ruling 25), SI: area, second moments about y and z, extreme-fibre distances along y and z,
    torsion constant, and shear areas along y and z (Cowper's shear coefficients at Poisson ratio 0: 5/6 for a
    rectangle, 6/7 for a circle, 6 (1 + m^2)^2 / (7 (1 + m^2)^2 + 20 m^2) for a tube of diameter ratio m; the webs
    for a rectangular tube)."""

    area: float
    second_moment_y: float
    second_moment_z: float
    fibre_y: float
    fibre_z: float
    torsion_constant: float
    shear_area_y: float
    shear_area_z: float


def _rectangle_torsion(long: float, short: float) -> float:
    """Saint-Venant torsion constant of a solid rectangle, Roark's form: a b^3 (1/3 - 0.21 (b/a) (1 - b^4 / (12 a^4)))
    with a the long and b the short side (within 4 percent for every aspect ratio)."""
    ratio = short / long
    return long * short**3 * (1.0 / 3.0 - 0.21 * ratio * (1.0 - ratio**4 / 12.0))


def shape_properties(shape: SectionShape) -> SectionProperties:
    """`SectionProperties` of a section shape from its dimensions (ruling 25)."""
    if isinstance(shape, RectangleShape):
        width, height = _set_floats(shape.width, shape.height)
        area = width * height
        shear = 5.0 / 6.0 * area
        return SectionProperties(
            area,
            height * width**3 / 12.0,
            width * height**3 / 12.0,
            height / 2.0,
            width / 2.0,
            _rectangle_torsion(max(width, height), min(width, height)),
            shear,
            shear,
        )
    if isinstance(shape, (CircleShape, TubeShape)):
        if isinstance(shape, CircleShape):
            (outer,) = _set_floats(shape.diameter)
            inner = 0.0
        else:
            outer, inner = _set_floats(shape.outer_diameter, shape.inner_diameter)
        area = math.pi * (outer**2 - inner**2) / 4.0
        moment = math.pi * (outer**4 - inner**4) / 64.0
        m2 = (inner / outer) ** 2
        coefficient = 6.0 * (1.0 + m2) ** 2 / (7.0 * (1.0 + m2) ** 2 + 20.0 * m2)
        shear = coefficient * area
        return SectionProperties(area, moment, moment, outer / 2.0, outer / 2.0, 2.0 * moment, shear, shear)
    width, height, wall = _set_floats(shape.width, shape.height, shape.wall)
    inner_width = width - 2.0 * wall
    inner_height = height - 2.0 * wall
    area = width * height - inner_width * inner_height
    mean_width = width - wall
    mean_height = height - wall
    torsion = 4.0 * (mean_width * mean_height) ** 2 * wall / (2.0 * (mean_width + mean_height))
    return SectionProperties(
        area,
        (height * width**3 - inner_height * inner_width**3) / 12.0,
        (width * height**3 - inner_width * inner_height**3) / 12.0,
        height / 2.0,
        width / 2.0,
        torsion,
        2.0 * wall * height,
        2.0 * wall * width,
    )


def _set_floats(*values: float | OpenValue) -> tuple[float, ...]:
    out: list[float] = []
    for value in values:
        if isinstance(value, OpenValue):
            raise TypeError(f"open section dimension {value} has no area or second moment")
        out.append(value)
    return tuple(out)


def _shape_problem(shape: SectionShape) -> str | None:
    if isinstance(shape, RectangleShape):
        _require_positive(shape.width, "section width")
        _require_positive(shape.height, "section height")
    elif isinstance(shape, CircleShape):
        _require_positive(shape.diameter, "section diameter")
    elif isinstance(shape, TubeShape):
        _require_positive(shape.outer_diameter, "section outer_diameter")
        _require_non_negative(shape.inner_diameter, "section inner_diameter")
        outer = shape.outer_diameter
        inner = shape.inner_diameter
        if isinstance(outer, float) and isinstance(inner, float) and inner >= outer:
            return "section inner_diameter must be below outer_diameter"
    else:
        _require_positive(shape.width, "section width")
        _require_positive(shape.height, "section height")
        _require_positive(shape.wall, "section wall")
        wall, width, height = shape.wall, shape.width, shape.height
        if (
            isinstance(wall, float)
            and isinstance(width, float)
            and isinstance(height, float)
            and 2.0 * wall >= min(width, height)
        ):
            return "section wall must be below half the smaller of width and height"
    return None


class Section(_Strict):
    """Cross-section and span (DES-033, ruling 25): a `shape` with its dimensions; or `area` with `second_moment` (the
    symmetric shorthand: both axes equal, optional `fibre_y` and `fibre_z` together, else evaluated single-axis); or
    `area` with both axes explicit, `second_moment_y`, `second_moment_z`, `fibre_y`, `fibre_z` (member frame: z the
    axis of bending along y, so `second_moment_z` is the neutral member's `second_moment_of_area`). E and rho come from
    the body's material."""

    area: Area | None = None
    second_moment: SecondMomentOfArea | None = None
    second_moment_y: SecondMomentOfArea | None = None
    second_moment_z: SecondMomentOfArea | None = None
    fibre_y: Length | None = None
    fibre_z: Length | None = None
    shape: SectionShape | None = None
    length: Length

    @model_validator(mode="after")
    def _positive(self) -> Section:
        axes = self.second_moment_y is not None or self.second_moment_z is not None
        fibres = self.fibre_y is not None or self.fibre_z is not None
        explicit = self.area is not None or self.second_moment is not None or axes or fibres
        if self.shape is not None and explicit:
            raise ValueError("section takes a shape or area and second_moment, not both")
        if self.shape is None:
            if self.area is None:
                raise ValueError("section needs a shape, or both area and second_moment")
            if self.second_moment is not None and axes:
                raise ValueError("section takes second_moment (both axes equal) or second_moment_y and _z, not both")
            if self.second_moment is None and not axes:
                raise ValueError("section needs a shape, or both area and second_moment")
            if axes and (self.second_moment_y is None or self.second_moment_z is None):
                raise ValueError("section with explicit axes needs both second_moment_y and second_moment_z")
            if axes and (self.fibre_y is None or self.fibre_z is None):
                raise ValueError("section with explicit axes needs both fibre_y and fibre_z")
            if fibres and (self.fibre_y is None or self.fibre_z is None):
                raise ValueError("section fibre distances come together: fibre_y and fibre_z")
        if self.shape is not None:
            problem = _shape_problem(self.shape)
            if problem is not None:
                raise ValueError(problem)
        _require_positive(self.area, "section area")
        _require_positive(self.second_moment, "section second_moment")
        _require_positive(self.second_moment_y, "section second_moment_y")
        _require_positive(self.second_moment_z, "section second_moment_z")
        _require_positive(self.fibre_y, "section fibre_y")
        _require_positive(self.fibre_z, "section fibre_z")
        _require_positive(self.length, "section length")
        return self

    def properties(self) -> SectionProperties | None:
        """Full section properties: from the shape, or None for the explicit forms."""
        if self.shape is None:
            return None
        return shape_properties(self.shape)

    def bending_axes(self) -> tuple[float, float, float, float] | None:
        """SI (I_y, I_z, c_y, c_z) as the file states them: a shape's, the explicit axes, or the shorthand with its
        fibre distances (I_y = I_z = second_moment); None for the shorthand without fibres (single-axis)."""
        if self.shape is not None:
            found = shape_properties(self.shape)
            return found.second_moment_y, found.second_moment_z, found.fibre_y, found.fibre_z
        if self.fibre_y is None or self.fibre_z is None:
            return None
        if self.second_moment_y is not None and self.second_moment_z is not None:
            i_y, i_z, c_y, c_z = _set_floats(self.second_moment_y, self.second_moment_z, self.fibre_y, self.fibre_z)
            return i_y, i_z, c_y, c_z
        if self.second_moment is None:
            return None
        moment, c_y, c_z = _set_floats(self.second_moment, self.fibre_y, self.fibre_z)
        return moment, moment, c_y, c_z

    def area_and_moment(self) -> tuple[float, float]:
        """SI area and second moment about z (the bending axis of the lumped chain); raises TypeError while a
        dimension is open."""
        if self.shape is not None:
            return shape_area_and_moment(self.shape)
        moment = self.second_moment
        if moment is None:
            moment = self.second_moment_z
        if self.area is None or moment is None:
            raise TypeError("section without area and second_moment")
        area, value = _set_floats(self.area, moment)
        return area, value


class Calibration(_Strict):
    """The known deflection under `load` and first natural frequency the lumped member is calibrated to (DES-034).

    `load` is a point load at the free end (clamped-free) or at midspan (any other support pair); `deflection` is
    the deflection at the load point."""

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
    """A flexible member (DES-033). `section` may be omitted when the body's catalog part carries one (LIB-030);
    `calibration` may be omitted, and the member then uses its section and material as is (BLD-019)."""

    section: Section | None = None
    calibration: Annotated[Calibration | OpenValue | None, BeforeValidator(_choice)] = None
    supports: list[SupportCondition] = Field(min_length=2, max_length=2, description="root, end")
    segments: int = Field(default=8, ge=1, description="lumped segment count (BLD-018, ruling 17)")
    carries: list[Name] = Field(
        default_factory=list,
        description="joints the member carries (DES-033, ENG-052, ruling 63): each joint's parent is this body, "
        "the lumped chain's parent; its in-plane load reaches the body through the chain",
    )
    damping_ratio: Fraction | None = Field(
        default=None,
        description="modal damping ratio of the first bending mode, dimensionless in [0, 1) (DES-047, ruling 78); "
        "absent: undamped, the member's ringing set by the integration scheme",
    )

    @model_validator(mode="after")
    def _damping_ratio(self) -> FlexibleMember:
        ratio = self.damping_ratio
        if isinstance(ratio, float) and not (0.0 <= ratio < 1.0):
            raise ValueError(f"damping_ratio {ratio!r}: a modal damping ratio in [0, 1) (DES-047)")
        return self


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


class Senses(_Strict):
    """What a sensor instance reads (DES-043): the joint coordinate, and per type the switch position along the joint
    and its actuation side (limit_switch) or the count direction (incremental_encoder); a load cell may read the case
    payload by name instead of a joint, or the reaction of a joint (`joint_reaction`: a design joint, or a case payload
    by name, read at its fixed joint to the carrier; ruling 24), an IMU reads a body. Exactly one of joint, payload,
    joint_reaction and body."""

    joint: OpenName | None = None
    payload: Name | None = None
    joint_reaction: Name | None = None
    body: OpenName | None = None
    position: Length | None = None
    actuate_above: bool | None = None
    reversed: bool | None = None


SENSES_FIELDS: dict[str, tuple[str, ...]] = {
    "limit_switch": ("position", "actuate_above"),
    "incremental_encoder": ("reversed",),
    "linear_encoder": ("reversed",),
}
SENSES_REQUIRED: dict[str, tuple[str, ...]] = {"limit_switch": ("position", "actuate_above")}
SENSES_TARGETS: dict[str, tuple[str, ...]] = {
    "load_cell": ("joint", "payload", "joint_reaction"),
    "imu": ("body",),
}
SENSES_DEFAULT_TARGETS: tuple[str, ...] = ("joint",)
ACTS_ON_TYPES: tuple[str, ...] = ("brake",)


class ActsOn(_Strict):
    """The joint a brake-type instance holds (DES-043); a brake on a transmission-driven joint sits on its shaft."""

    joint: OpenName


class ComponentInstance(_Strict):
    name: Name
    type: Name
    part: Name
    overrides: dict[Name, str | bool] = Field(default_factory=dict)
    mechanical: list[MechanicalBinding] = Field(default_factory=list)
    signals: list[SignalDeclaration] = Field(default_factory=list)
    senses: Senses | None = None
    acts_on: ActsOn | None = None


class Wire(_Strict):
    """A connection from one instance port to another (DES-041); ports are the pins of each type's electrical facet.

    Wires carry connectivity only in v1: no resistance, inductance, length or failure. Physical wire properties are a
    later component type (a seam).
    """

    source: PortName
    destination: PortName


class PinBinding(_Strict):
    signal: SignalName
    pin: McuPin
    mode: PeripheralMode


# ===== Power, environment, firmware, messages ==========================


class Rail(_Strict):
    name: Name
    supply: OpenName
    gated_by: list[OpenName] = Field(default_factory=list)
    consumers: list[Name] = Field(default_factory=list)


class Power(_Strict):
    rails: list[Rail] = Field(min_length=1)
    controller_rail: Name


class ContactParameters(_Strict):
    """Soft contact (ADR 0002): normal stiffness and damping, Coulomb friction coefficient."""

    stiffness: Stiffness
    damping: LinearDamping
    friction: Fraction

    @model_validator(mode="after")
    def _bounds(self) -> ContactParameters:
        _require_positive(self.stiffness, "stiffness")
        _require_non_negative(self.damping, "damping")
        _require_non_negative(self.friction, "friction")
        return self


class MaterialPair(ContactParameters):
    """Contact parameters overriding the default for one unordered pair of materials."""

    materials: Annotated[list[Name], Field(min_length=2, max_length=2)]


class Floor(_Strict):
    """The floor plane on the world: points x with normal . x = offset; contact acts on the side the normal faces."""

    body: Name = WORLD
    normal: UnitVector
    offset: Length

    @field_validator("normal")
    @classmethod
    def _unit(cls, value: list[float]) -> list[float]:
        return _unit_vector(value, "floor normal")

    @model_validator(mode="after")
    def _on_world(self) -> Floor:
        if self.body != WORLD:
            raise ValueError(f"the floor plane is allowed only on the world, not on {self.body!r}")
        return self


class Contact(_Strict):
    """The neutral model's contacts element field for field (DES-042, ADR 0003), plus the floor plane."""

    default: ContactParameters
    pairs: list[MaterialPair] = Field(default_factory=list)
    exclusions: list[Annotated[list[Name], Field(min_length=2, max_length=2)]] = Field(default_factory=list)
    floor: Floor | None = None


class Environment(_Strict):
    gravity: Annotated[list[Acceleration], Field(min_length=3, max_length=3)]
    ambient_temperature: Temperature
    contact: Contact | None = None

    @model_validator(mode="after")
    def _above_zero(self) -> Environment:
        _require_positive(self.ambient_temperature, "ambient_temperature")
        return self


def _code_keys(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): item for key, item in value.items()}
    return value


CodeNames = Annotated[dict[str, Name], BeforeValidator(_code_keys)]

EventArgument = Literal["axis", "fault", "state", "number", "none"]


class EventCode(_Strict):
    """A named event-log code and how its argument reads (DES-045, round 4 ruling 28): `axis` the instance of the
    pulse_output binding of that channel, `fault` or `state` the design's code name (a state argument may pack a
    from-to pair as (from << 8) | to, read `from>to`), `number` verbatim, `none` no detail."""

    name: Name
    argument: EventArgument


EventCodes = Annotated[dict[str, EventCode], BeforeValidator(_code_keys)]


class Firmware(_Strict):
    """`state_codes` and `fault_codes` name the firmware's reported `state_code` and `fault_code` values (DES-045);
    the application target is `mechsim_fw_<design name>` in the firmware tree (DES-040)."""

    path: RepoPath
    chip: OpenName
    tick_rate: Frequency
    state_codes: CodeNames = Field(default_factory=dict, description="state_code value (0..255) to state name")
    fault_codes: CodeNames = Field(default_factory=dict, description="fault_code value (0..255) to fault name")
    event_codes: EventCodes = Field(
        default_factory=dict,
        description=(
            "event_log code (0..255) to the event's name and its argument kind (axis, fault, state, number, none)"
        ),
    )

    @model_validator(mode="after")
    def _positive(self) -> Firmware:
        _require_positive(self.tick_rate, "tick_rate")
        errors: list[str] = []
        tables = (
            ("state_codes", self.state_codes),
            ("fault_codes", self.fault_codes),
            ("event_codes", {key: item.name for key, item in self.event_codes.items()}),
        )
        for what, table in tables:
            for key in table:
                if not key.isdigit() or int(key) > 255 or str(int(key)) != key:
                    errors.append(f"{what}: code {key!r} is not an integer 0..255")
            _duplicates(list(table.values()), f"{what} name", errors)
        if len(errors) > 0:
            raise ValueError("; ".join(errors))
        return self

    def state_name(self, code: int) -> str | None:
        return self.state_codes.get(str(code))

    def fault_name(self, code: int) -> str | None:
        return self.fault_codes.get(str(code))

    def event_name(self, code: int) -> str | None:
        entry = self.event_codes.get(str(code))
        if entry is None:
            return None
        return entry.name

    def event_argument(self, code: int) -> EventArgument | None:
        entry = self.event_codes.get(str(code))
        if entry is None:
            return None
        return entry.argument


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


def _origin() -> list[float | OpenValue]:
    return [0.0, 0.0, 0.0]


class RopeEnd(_Strict):
    """One end of a rope: a point in a design body's frame, or the case payload named here (attached at run time,
    so that end is not emitted by the builder until the case places the payload)."""

    body: Name | None = None
    payload: Name | None = None
    point: LengthVector = Field(default_factory=lambda: _origin())

    @model_validator(mode="after")
    def _one_target(self) -> RopeEnd:
        if (self.body is None) == (self.payload is None):
            raise ValueError("rope end names exactly one of body and payload")
        return self


class Rope(_Strict):
    """A rope from a catalog part of a rope type (LIB-020), emitted as a lumped chain (BLD-032, ADR 0016)."""

    name: Name
    part: Name
    from_end: RopeEnd = Field(alias="from")
    to_end: RopeEnd = Field(alias="to")
    segments: int = Field(default=8, ge=1, description="lumped chain segment count (ADR 0016)")
    rest_length: Length | None = None

    @model_validator(mode="after")
    def _ends(self) -> Rope:
        _require_positive(self.rest_length, "rope rest_length")
        if self.from_end.payload is not None:
            raise ValueError("a rope's from end is a design body; only its to end may name a case payload")
        return self


class MaterialsConfig(_Strict):
    """`overrides`: path, relative to the design file, of a materials override file (brief M3-M)."""

    overrides: str | None = None


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
    wiring: list[Wire] = Field(default_factory=list)
    pin_bindings: list[PinBinding] = Field(default_factory=list)
    power: Power | None = None
    firmware: Firmware | None = None
    operator_messages: list[OperatorMessage] = Field(default_factory=list)
    structural: Structural | None = None
    ropes: list[Rope] = Field(default_factory=list)
    materials: MaterialsConfig | None = None
    _source_dir: Path | None = PrivateAttr(default=None)

    @model_validator(mode="after")
    def _cross_check(self, info: ValidationInfo) -> Design:
        registry = default_registry()
        context = info.context
        if isinstance(context, dict) and isinstance(context.get("registry"), Registry):
            registry = context["registry"]
        chips: dict[str, Chip] | None = None
        if isinstance(context, dict) and isinstance(context.get("chips"), dict):
            chips = context["chips"]
        base: Path | None = None
        if isinstance(context, dict) and isinstance(context.get("base"), Path):
            base = context["base"]
            self._source_dir = base
        errors: list[str] = []
        try:
            materials = design_materials(self)
        except (DesignError, MaterialError) as error:
            errors.extend(f"materials.overrides: {line}" for line in str(error).splitlines())
            materials = shipped_materials()
        errors.extend(cross_check(self, registry, materials, chips))
        if base is not None:
            errors.extend(check_files(self, base))
        if len(errors) > 0:
            raise ValueError("\n".join(errors))
        return self

    def signal_names(self) -> list[str]:
        return [signal.name for component in self.components for signal in component.signals]


# ===== Cross-reference checks ==========================================


def design_materials(design: Design) -> dict[str, Material]:
    """The material records a design builds with: the shipped records with its `materials.overrides` file applied
    (MAT-016, MAT-020), the path resolved against the design file's folder; shipped records when it names none.
    Raises DesignError when the design names overrides but was not loaded from a file, MaterialError on a bad file."""
    overrides = None
    if design.materials is not None:
        overrides = design.materials.overrides
    if overrides is None:
        return shipped_materials()
    folder = design._source_dir
    if folder is None:
        raise DesignError(f"overrides file {overrides!r} has no design folder to resolve against")
    path = folder / overrides
    if not path.is_file():
        raise DesignError(f"missing materials override file {overrides!r}")
    return MaterialSet.from_files(path).records()


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


def cross_check(
    design: Design, registry: Registry, materials: dict[str, Material], chips: dict[str, Chip] | None = None
) -> list[str]:
    """Every reference and uniqueness error in `design`, each with its path (DES-016).

    `chips` defaults to the shipped chip definitions under `chips/`.
    """
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
    _check_carries(design, errors)
    for index, component in enumerate(design.components):
        _check_component(f"components[{index}]", component, design, registry, declared, errors)
    _check_signals(design, errors)
    _check_wiring(design, registry, errors)
    _check_chip(design, chips, errors)
    _check_power(design, registry, errors)
    _check_contact(design, materials, errors)
    _check_messages(design, errors)
    _check_ropes(design, registry, errors)
    return errors


def section_part_values(body: Body, registry: Registry) -> dict[str, float] | None:
    """The section and material values of the body's catalog part when it carries them all (LIB-030), else None."""
    reference = body.catalog_part
    if reference is None or not registry.has_part(reference.part):
        return None
    values = registry.part(reference.part).values
    out: dict[str, float] = {}
    for name in SECTION_PARAMETERS:
        value = values.get(name)
        if not isinstance(value, float):
            return None
        out[name] = value
    return out


def _check_flexible(path: str, body: Body, registry: Registry, errors: list[str]) -> None:
    flexible = body.flexible
    if flexible is None or flexible.section is not None:
        return
    if section_part_values(body, registry) is None:
        errors.append(
            f"{path}.flexible.section: no section given and the body's catalog part does not carry "
            f"{list(SECTION_PARAMETERS)} (DES-033, LIB-030)"
        )


def _check_ropes(design: Design, registry: Registry, errors: list[str]) -> None:
    bodies = {body.name for body in design.bodies}
    taken = bodies | {joint.name for joint in design.joints} | {component.name for component in design.components}
    _duplicates([rope.name for rope in design.ropes], "rope name", errors)
    for index, rope in enumerate(design.ropes):
        path = f"ropes[{index}]"
        if rope.name in taken:
            errors.append(f"{path}.name: {rope.name!r} is already a body, joint or component name")
        if not registry.has_part(rope.part):
            errors.append(f"{path}.part: unknown catalog part {rope.part!r}")
        else:
            values = registry.part(rope.part).values
            missing = [name for name in ROPE_PARAMETERS if not isinstance(values.get(name), float)]
            if len(missing) > 0:
                errors.append(f"{path}.part: part {rope.part!r} is not a rope part (missing {missing}, LIB-020)")
        for key, end in (("from", rope.from_end), ("to", rope.to_end)):
            if end.body is not None and end.body not in bodies and end.body != WORLD:
                errors.append(f"{path}.{key}.body: unknown body {end.body!r}")


def _check_material(path: str, material: Any, materials: dict[str, Material], errors: list[str]) -> Material | None:
    if isinstance(material, OpenValue):
        return None
    if material not in materials:
        errors.append(f"{path}: unknown material {material!r}")
        return None
    return materials[material]


def _check_body(path: str, body: Body, registry: Registry, materials: dict[str, Material], errors: list[str]) -> None:
    _check_flexible(path, body, registry, errors)
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


def _check_carries(design: Design, errors: list[str]) -> None:
    joints = {joint.name: joint for joint in design.joints}
    carried_by: dict[str, str] = {}
    for index, body in enumerate(design.bodies):
        if body.flexible is None:
            continue
        for carried_index, name in enumerate(body.flexible.carries):
            path = f"bodies[{index}].flexible.carries[{carried_index}]"
            if name not in joints:
                errors.append(f"{path}: unknown joint {name!r}")
                continue
            if name in carried_by:
                errors.append(f"{path}: joint {name!r} is already carried by {carried_by[name]!r} (ENG-052)")
                continue
            carried_by[name] = body.name
            if joints[name].parent != body.name:
                errors.append(
                    f"{path}: joint {name!r} has parent {joints[name].parent!r}; a carried joint's parent is the "
                    f"member's own body {body.name!r}, the lumped chain's parent (ENG-052)"
                )


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
    _check_senses(path, component, kind.category, joints, bodies, errors)
    _check_acts_on(path, component, joints, errors)
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


def _check_senses(
    path: str,
    component: ComponentInstance,
    category: str,
    joints: set[str],
    bodies: set[str],
    errors: list[str],
) -> None:
    senses = component.senses
    if category != "sensor":
        if senses is not None:
            errors.append(f"{path}.senses: only sensor instances carry senses; {component.type!r} is a {category}")
        return
    targets = SENSES_TARGETS.get(component.type, SENSES_DEFAULT_TARGETS)
    if senses is None:
        errors.append(
            f"{path}.senses: sensor instance {component.name!r} must name the {' or '.join(targets)} it senses"
        )
        return
    given = [name for name in ("joint", "payload", "joint_reaction", "body") if getattr(senses, name) is not None]
    for name in given:
        if name not in targets:
            errors.append(f"{path}.senses.{name}: type {component.type!r} takes no {name}")
    if len(given) == 0:
        errors.append(f"{path}.senses: type {component.type!r} must name one of {', '.join(targets)}")
    if len(given) > 1:
        errors.append(f"{path}.senses: names {', '.join(given)}; exactly one is allowed")
    if isinstance(senses.joint, str) and senses.joint not in joints:
        errors.append(f"{path}.senses.joint: unknown joint {senses.joint!r}")
    if isinstance(senses.body, str) and senses.body not in bodies:
        errors.append(f"{path}.senses.body: unknown body {senses.body!r}")
    allowed = SENSES_FIELDS.get(component.type, ())
    for field in ("position", "actuate_above", "reversed"):
        value = getattr(senses, field)
        if value is not None and field not in allowed:
            errors.append(f"{path}.senses.{field}: type {component.type!r} takes no {field}")
    for field in SENSES_REQUIRED.get(component.type, ()):
        if getattr(senses, field) is None:
            errors.append(f"{path}.senses.{field}: required for type {component.type!r}")


def _check_acts_on(path: str, component: ComponentInstance, joints: set[str], errors: list[str]) -> None:
    acts_on = component.acts_on
    if component.type not in ACTS_ON_TYPES:
        if acts_on is not None:
            errors.append(f"{path}.acts_on: only {', '.join(ACTS_ON_TYPES)} instances act on a joint")
        return
    if acts_on is None:
        errors.append(f"{path}.acts_on: {component.type} instance {component.name!r} must name the joint it acts on")
        return
    if isinstance(acts_on.joint, str) and acts_on.joint not in joints:
        errors.append(f"{path}.acts_on.joint: unknown joint {acts_on.joint!r}")


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
    instances = {component.name for component in design.components}
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
        _duplicates(rail.consumers, f"{where}.consumers: consumer", errors)
        for consumer_index, consumer in enumerate(rail.consumers):
            if consumer not in instances:
                errors.append(f"{where}.consumers[{consumer_index}]: unknown component instance {consumer!r}")
    if design.power.controller_rail not in rails:
        errors.append(f"power.controller_rail: unknown rail {design.power.controller_rail!r}")


def _port(path: str, reference: str, design: Design, registry: Registry, errors: list[str]) -> tuple[str, str] | None:
    """The (direction, signal kind) of the port `instance.port`, or None after recording why it does not resolve."""
    instance, port = reference.split(".")
    types = {component.name: component.type for component in design.components}
    if instance not in types:
        errors.append(f"{path}: unknown component instance {instance!r}")
        return None
    if not registry.has_type(types[instance]):
        return None
    for pin in registry.component_type(types[instance]).electrical.pins:
        if pin.name == port:
            return pin.direction, pin.signal_kind
    errors.append(f"{path}: type {types[instance]!r} has no port {port!r}")
    return None


def _check_wiring(design: Design, registry: Registry, errors: list[str]) -> None:
    """Both ends exist, directions are compatible, and a signal port carries one connection (DES-041).

    A signal port's connections count its wires and any controller-facing signal it declares; power ports (power in,
    power out, ground) may be shared.
    """
    uses: dict[str, list[str]] = {}
    for component in design.components:
        for signal in component.signals:
            uses.setdefault(f"{component.name}.{signal.pin}", []).append(f"signal {signal.name!r}")
    seen: set[tuple[str, str]] = set()
    for index, wire in enumerate(design.wiring):
        where = f"wiring[{index}]"
        if (wire.source, wire.destination) in seen:
            errors.append(f"{where}: duplicate wire {wire.source!r} to {wire.destination!r}")
        seen.add((wire.source, wire.destination))
        source = _port(f"{where}.source", wire.source, design, registry, errors)
        destination = _port(f"{where}.destination", wire.destination, design, registry, errors)
        if source is None or destination is None:
            continue
        if destination[0] not in WIRE_DIRECTIONS.get(source[0], ()):
            errors.append(
                f"{where}: cannot wire {source[0]} port {wire.source!r} to {destination[0]} port {wire.destination!r}"
            )
            continue
        for end, direction in ((wire.source, source[0]), (wire.destination, destination[0])):
            if direction not in POWER_PORT_DIRECTIONS:
                uses.setdefault(end, []).append(where)
    for port, connections in uses.items():
        if len(connections) > 1:
            errors.append(f"wiring: signal port {port!r} is connected more than once ({', '.join(connections)})")


def _check_chip(design: Design, chips: dict[str, Chip] | None, errors: list[str]) -> None:
    """Every pin-binding entry's pin and mode must exist in the chip's pin map (DES-029)."""
    if design.firmware is None or not isinstance(design.firmware.chip, str):
        return
    known = chips
    if known is None:
        known = shipped_chips()
    name = design.firmware.chip
    if name not in known:
        errors.append(f"firmware.chip: unknown chip {name!r}; no chips/{name}/chip.yaml")
        return
    modes = known[name].pin_modes()
    for index, binding in enumerate(design.pin_bindings):
        where = f"pin_bindings[{index}]"
        if binding.pin not in modes:
            errors.append(f"{where}.pin: chip {name!r} has no pin {binding.pin!r}")
        elif binding.mode not in modes[binding.pin]:
            errors.append(f"{where}.mode: chip {name!r} pin {binding.pin!r} does not support mode {binding.mode!r}")


def _unordered(pair: list[str]) -> tuple[str, str]:
    return (min(pair), max(pair))


def _check_contact(design: Design, materials: dict[str, Material], errors: list[str]) -> None:
    """The neutral model's contact rules, checked at load so the builder copies a valid section (DES-042)."""
    contact = design.environment.contact
    if contact is None:
        return
    pairs: set[tuple[str, str]] = set()
    for index, pair in enumerate(contact.pairs):
        where = f"environment.contact.pairs[{index}].materials"
        for material in pair.materials:
            if material not in materials:
                errors.append(f"{where}: unknown material {material!r}")
        key = _unordered(pair.materials)
        if key in pairs:
            errors.append(f"{where}: material pair {key[0]}/{key[1]} listed twice")
        pairs.add(key)
    bodies = {body.name for body in design.bodies}
    bodies.add(WORLD)
    exclusions: set[tuple[str, str]] = set()
    for index, exclusion in enumerate(contact.exclusions):
        where = f"environment.contact.exclusions[{index}]"
        for body in exclusion:
            if body not in bodies:
                errors.append(f"{where}: exclusion names unknown body {body!r}")
        if exclusion[0] == exclusion[1]:
            errors.append(f"{where}: exclusion pairs body {exclusion[0]!r} with itself")
        key = _unordered(exclusion)
        if key in exclusions:
            errors.append(f"{where}: exclusion {key[0]}/{key[1]} listed twice")
        exclusions.add(key)


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
