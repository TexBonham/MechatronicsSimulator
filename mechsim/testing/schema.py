"""Test case files (TST). Generates `schemas/test_case.json`.

A test case names the design (name and semver version) it was written against, its initial state, the operator command
timeline, the fault timeline, the end condition, its pass criteria, sweeps and seeded variations, and the regression
baseline it is scored against. Quantities are unit strings held as SI floats. A case for a draft design may leave values
unset as `${name}` references to the design's or its own open parameters; a case for a release design may not.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Annotated, Any, Literal

import pint
from pydantic import (
    BaseModel,
    BeforeValidator,
    ConfigDict,
    Field,
    GetCoreSchemaHandler,
    GetJsonSchemaHandler,
    ValidationError,
    WithJsonSchema,
    model_validator,
)
from pydantic.json_schema import JsonSchemaValue
from pydantic_core import CoreSchema, core_schema

from mechsim.design.schema import (
    CHOICE,
    OPEN_PATTERN,
    OPERATOR,
    QUANTITY_PATTERN,
    Design,
    DesignError,
    Fraction,
    Frequency,
    Mass,
    Name,
    OpenParameter,
    OpenValue,
    RepoPath,
    SemVer,
    Time,
    iter_open_values,
    load_design,
    open_name,
    yaml_loader,
)
from mechsim.library.schema import dimensions_equal
from mechsim.schema_base import StrictModel, format_errors

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = ROOT / "schemas" / "test_case.json"
DESIGNS_ROOT = ROOT / "designs"

ANY = "any"
LENGTH = "[length]"
ANGLE = "dimensionless"
TIME = "[time]"

CASE_ID_PATTERN = r"^[A-Z][A-Z0-9_]*[0-9]+$"
_N = r"[a-z][a-z0-9_]*"
QUANTITY_REF_PATTERN = (
    rf"^(signal:{_N}\.{_N}|pose:{_N}\.(x|y|z|roll|pitch|yaw)|joint:{_N}\.(position|velocity)"
    rf"|state:{_N}|count:(message|state)\.{_N})$"
)
EVENT_REF_PATTERN = rf"^(message|state):{_N}$"
PARAMETER_PATH_PATTERN = rf"^(\$\{{{_N}\}}|(bodies|joints|components)\.{_N}(\.{_N})+)$"

_UNITS = pint.UnitRegistry()
_LEADING_NUMBER = re.compile(r"^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?(\s|$)")
_QUANTITY_OR_OPEN_SCHEMA = {
    "anyOf": [{"type": "string", "pattern": QUANTITY_PATTERN}, {"type": "string", "pattern": OPEN_PATTERN}]
}

CaseId = Annotated[str, Field(pattern=CASE_ID_PATTERN)]
QuantityRef = Annotated[str, Field(pattern=QUANTITY_REF_PATTERN)]
EventRef = Annotated[str, Field(pattern=EVENT_REF_PATTERN)]
ParameterPath = Annotated[str, Field(pattern=PARAMETER_PATH_PATTERN)]


class CaseError(ValueError):
    """A test case file failed to load; the message names the file, the path within it and the reason."""


# ===== Quantities of any dimension =====================================


class Quantity:
    """A unit string whose dimension the file does not fix: the SI magnitude, its pint dimensionality, the text."""

    __slots__ = ("dimension", "text", "value")

    def __init__(self, text: str, value: float, dimension: str) -> None:
        self.text = text
        self.value = value
        self.dimension = dimension

    def __str__(self) -> str:
        return self.text

    def __repr__(self) -> str:
        return f"Quantity({self.text!r})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Quantity):
            return NotImplemented
        return self.value == other.value and dimensions_equal(self.dimension, other.dimension)

    def __hash__(self) -> int:
        return hash(self.value)

    @classmethod
    def __get_pydantic_core_schema__(cls, source: Any, handler: GetCoreSchemaHandler) -> CoreSchema:
        return core_schema.is_instance_schema(cls, serialization=core_schema.plain_serializer_function_ser_schema(str))

    @classmethod
    def __get_pydantic_json_schema__(cls, schema: CoreSchema, handler: GetJsonSchemaHandler) -> JsonSchemaValue:
        return {"type": "string", "pattern": QUANTITY_PATTERN}


def parse_any_quantity(text: Any) -> Quantity:
    """Parse a quantity string of any dimension; a bare number is dimensionless (a count or a fraction)."""
    if isinstance(text, Quantity):
        return text
    if isinstance(text, bool) or not isinstance(text, str):
        raise ValueError(f"quantity must be a string with a unit, got {text!r}")
    stripped = text.strip()
    if _LEADING_NUMBER.match(stripped) is None:
        raise ValueError(f"quantity {text!r} must start with a number followed by its unit")
    try:
        parsed = _UNITS.Quantity(stripped)
    except Exception as error:
        raise ValueError(f"cannot parse quantity {text!r}: {error}") from error
    base = parsed.to_base_units()
    value = float(base.magnitude)
    if not math.isfinite(value):
        raise ValueError(f"quantity {text!r} is not finite")
    dimension = str(base.dimensionality)
    if base.dimensionless:
        dimension = ANGLE
    return Quantity(text, value, dimension)


def _amount(value: Any) -> Any:
    name = open_name(value)
    if name is not None:
        return OpenValue(name, ANY)
    return parse_any_quantity(value)


Amount = Annotated[Quantity | OpenValue, BeforeValidator(_amount), WithJsonSchema(_QUANTITY_OR_OPEN_SCHEMA)]
Measured = Annotated[Quantity, BeforeValidator(parse_any_quantity)]


def _magnitude(value: Any) -> float | None:
    if isinstance(value, float):
        return value
    if isinstance(value, Quantity):
        return value.value
    return None


def _dimension_error(value: Any, expected: str | None, what: str) -> str | None:
    if expected is None or not isinstance(value, Quantity):
        return None
    if dimensions_equal(value.dimension, expected):
        return None
    return f"{what} {value.text!r} has dimension {value.dimension}, expected {expected}"


def _raise_if(errors: list[str]) -> None:
    if len(errors) > 0:
        raise ValueError("\n".join(errors))


# ===== Header ==========================================================


class DesignRef(StrictModel):
    name: Name
    version: SemVer


class Rates(StrictModel):
    """Overrides of the design's rates for this case; at least one is given."""

    physics_step: Time | None = None
    control_rate: Frequency | None = None

    @model_validator(mode="after")
    def _given_and_positive(self) -> Rates:
        if self.physics_step is None and self.control_rate is None:
            raise ValueError("rates needs physics_step, control_rate or both")
        for value, what in ((self.physics_step, "physics_step"), (self.control_rate, "control_rate")):
            if isinstance(value, float) and value <= 0.0:
                raise ValueError(f"{what} must be positive, got {value}")
        return self


# ===== Initial state ===================================================


class JointState(StrictModel):
    """A single-DOF joint's start: a length and a speed (prismatic) or an angle and an angular rate (revolute)."""

    joint: Name
    position: Amount
    velocity: Amount

    @model_validator(mode="after")
    def _consistent(self) -> JointState:
        if isinstance(self.position, Quantity):
            motion = self.motion()
            if motion is None:
                raise ValueError(f"position {self.position.text!r} must be a length or an angle")
            error = _dimension_error(self.velocity, f"{self.expected_position()} / [time]", "velocity")
            _raise_if([error] if error is not None else [])
        elif isinstance(self.velocity, Quantity):
            linear = dimensions_equal(self.velocity.dimension, "[length] / [time]")
            angular = dimensions_equal(self.velocity.dimension, "1 / [time]")
            if not linear and not angular:
                raise ValueError(f"velocity {self.velocity.text!r} must be a speed or an angular rate")
        return self

    def motion(self) -> Literal["linear", "angular"] | None:
        if isinstance(self.position, Quantity):
            if dimensions_equal(self.position.dimension, LENGTH):
                return "linear"
            if dimensions_equal(self.position.dimension, ANGLE):
                return "angular"
            return None
        if isinstance(self.velocity, Quantity):
            if dimensions_equal(self.velocity.dimension, "[length] / [time]"):
                return "linear"
            return "angular"
        return None

    def expected_position(self) -> str:
        if self.motion() == "linear":
            return LENGTH
        return ANGLE


class Payload(StrictModel):
    attached: bool
    body: Name | None = None
    mass: Mass | None = None

    @model_validator(mode="after")
    def _attached_needs_body_and_mass(self) -> Payload:
        if self.attached:
            if self.body is None or self.mass is None:
                raise ValueError("an attached payload needs body and mass")
            if isinstance(self.mass, float) and self.mass <= 0.0:
                raise ValueError(f"payload mass must be positive, got {self.mass}")
        elif self.body is not None or self.mass is not None:
            raise ValueError("a payload that is not attached takes no body or mass")
        return self


class InitialState(StrictModel):
    joints: list[JointState] = Field(default_factory=list)
    controller: dict[Name, bool | Name] = Field(default_factory=dict, description="controller state assumptions")
    payload: Payload | None = None

    @model_validator(mode="after")
    def _unique_joints(self) -> InitialState:
        errors: list[str] = []
        _duplicates([state.joint for state in self.joints], "initial joint", errors)
        _raise_if(errors)
        return self


# ===== Commands and faults =============================================


class Command(StrictModel):
    """An operator message sent through the operator link at `at`; payload keys are the message's field names."""

    at: Time
    message: Name
    payload: dict[Name, Amount] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _not_before_start(self) -> Command:
        if isinstance(self.at, float) and self.at < 0.0:
            raise ValueError(f"command time must not be negative, got {self.at}")
        return self


class _Windowed(StrictModel):
    """A fault active from `start` for its window, given as `end` or `duration` (exactly one)."""

    start: Time
    end: Time | None = None
    duration: Time | None = None

    @model_validator(mode="after")
    def _window(self) -> _Windowed:
        if (self.end is None) == (self.duration is None):
            raise ValueError("a fault window needs exactly one of end and duration")
        if isinstance(self.start, float) and self.start < 0.0:
            raise ValueError(f"fault start must not be negative, got {self.start}")
        if isinstance(self.duration, float) and self.duration <= 0.0:
            raise ValueError(f"fault duration must be positive, got {self.duration}")
        if isinstance(self.end, float) and isinstance(self.start, float) and self.end <= self.start:
            raise ValueError(f"fault end {self.end} must be after its start {self.start}")
        return self


class SensorStuck(_Windowed):
    kind: Literal["sensor_stuck"]
    target: Name


class SensorNoisy(_Windowed):
    kind: Literal["sensor_noisy"]
    target: Name
    standard_deviation: Amount

    @model_validator(mode="after")
    def _positive(self) -> SensorNoisy:
        magnitude = _magnitude(self.standard_deviation)
        if magnitude is not None and magnitude <= 0.0:
            raise ValueError(f"standard_deviation must be positive, got {self.standard_deviation}")
        return self


class SensorDropped(_Windowed):
    kind: Literal["sensor_dropped"]
    target: Name


class SensorForced(_Windowed):
    kind: Literal["sensor_forced"]
    target: Name
    level: Literal["high", "low"]


class Slip(_Windowed):
    kind: Literal["slip"]
    target: Name
    fraction: Fraction

    @model_validator(mode="after")
    def _bounded(self) -> Slip:
        if isinstance(self.fraction, float) and not 0.0 < self.fraction <= 1.0:
            raise ValueError(f"slip fraction must be above 0 and at most 1, got {self.fraction}")
        return self


class MotorStall(_Windowed):
    kind: Literal["motor_stall"]
    target: Name
    available_torque: Fraction

    @model_validator(mode="after")
    def _bounded(self) -> MotorStall:
        if isinstance(self.available_torque, float) and not 0.0 <= self.available_torque < 1.0:
            raise ValueError(f"available_torque must be at least 0 and below 1, got {self.available_torque}")
        return self


class PowerLoss(_Windowed):
    kind: Literal["power_loss"]
    rail: Name


class ControllerFreeze(_Windowed):
    kind: Literal["controller_freeze"]


class EstopPress(_Windowed):
    kind: Literal["estop_press"]
    target: Name


class PayloadChange(StrictModel):
    """Instantaneous: at `start` the payload on `target` becomes `mass`; 0 kg detaches it."""

    kind: Literal["payload_change"]
    start: Time
    target: Name
    mass: Mass

    @model_validator(mode="after")
    def _bounded(self) -> PayloadChange:
        if isinstance(self.start, float) and self.start < 0.0:
            raise ValueError(f"fault start must not be negative, got {self.start}")
        if isinstance(self.mass, float) and self.mass < 0.0:
            raise ValueError(f"payload mass must not be negative, got {self.mass}")
        return self


Fault = Annotated[
    SensorStuck
    | SensorNoisy
    | SensorDropped
    | SensorForced
    | Slip
    | MotorStall
    | PowerLoss
    | ControllerFreeze
    | EstopPress
    | PayloadChange,
    Field(discriminator="kind"),
]
SENSOR_FAULTS = ("sensor_stuck", "sensor_noisy", "sensor_dropped", "sensor_forced")


# ===== End condition ===================================================


class EndCondition(StrictModel):
    """Ends at a time, on an observed operator message, or when the controller enters a state; never later than
    `max_duration`."""

    at: Time | None = None
    message: Name | None = None
    controller_state: Name | None = None
    max_duration: Time

    @model_validator(mode="after")
    def _one_trigger(self) -> EndCondition:
        given = [value for value in (self.at, self.message, self.controller_state) if value is not None]
        if len(given) != 1:
            raise ValueError("an end condition needs exactly one of at, message and controller_state")
        if isinstance(self.max_duration, float) and self.max_duration <= 0.0:
            raise ValueError(f"max_duration must be positive, got {self.max_duration}")
        if isinstance(self.at, float):
            if self.at <= 0.0:
                raise ValueError(f"end time must be positive, got {self.at}")
            if isinstance(self.max_duration, float) and self.at > self.max_duration:
                raise ValueError(f"end time {self.at} exceeds max_duration {self.max_duration}")
        return self

    def limit(self) -> float | None:
        if isinstance(self.max_duration, float):
            return self.max_duration
        return None


# ===== Pass criteria ===================================================


class Window(StrictModel):
    start: Time | None = None
    end: Time | None = None

    @model_validator(mode="after")
    def _ordered(self) -> Window:
        if self.start is None and self.end is None:
            raise ValueError("a window needs start, end or both")
        if isinstance(self.start, float) and self.start < 0.0:
            raise ValueError(f"window start must not be negative, got {self.start}")
        if isinstance(self.end, float) and self.end <= 0.0:
            raise ValueError(f"window end must be positive, got {self.end}")
        if isinstance(self.start, float) and isinstance(self.end, float) and self.end <= self.start:
            raise ValueError(f"window end {self.end} must be after its start {self.start}")
        return self


# below <, at_most <=, above >, at_least >=: inclusive and strict forms are named, never implied.
Ordering = Literal["below", "at_most", "above", "at_least"]


def reference_dimension(reference: str) -> str | None:
    """The dimension a quantity reference measures, or None when only the design knows it (signals, joints)."""
    kind, _, rest = reference.partition(":")
    if kind == "pose":
        component = rest.rpartition(".")[2]
        if component in ("x", "y", "z"):
            return LENGTH
        return ANGLE
    if kind in ("state", "count"):
        return ANGLE
    return None


class _Criterion(StrictModel):
    name: Name

    def limit_text(self) -> str:
        raise NotImplementedError


class ValueCriterion(_Criterion):
    """A statistic of a recorded series against a limit (SCO-020); `within` compares with `target` absolutely
    (`limit`) or relatively (`relative_limit`)."""

    check: Literal["value"]
    quantity: QuantityRef
    statistic: Literal["final", "minimum", "maximum", "peak_absolute", "period"]
    comparison: Literal["below", "at_most", "above", "at_least", "within"]
    target: Amount | None = None
    limit: Amount | None = None
    relative_limit: Fraction | None = None
    window: Window | None = None

    @model_validator(mode="after")
    def _consistent(self) -> ValueCriterion:
        errors: list[str] = []
        if self.comparison == "within":
            if self.target is None:
                errors.append("comparison 'within' needs a target")
            if (self.limit is None) == (self.relative_limit is None):
                errors.append("comparison 'within' needs exactly one of limit and relative_limit")
            magnitude = _magnitude(self.limit)
            if magnitude is not None and magnitude <= 0.0:
                errors.append(f"a 'within' limit must be positive, got {self.limit}")
            if isinstance(self.relative_limit, float) and self.relative_limit <= 0.0:
                errors.append(f"relative_limit must be positive, got {self.relative_limit}")
        else:
            if self.limit is None:
                errors.append(f"comparison {self.comparison!r} needs a limit")
            if self.target is not None or self.relative_limit is not None:
                errors.append(f"comparison {self.comparison!r} takes no target or relative_limit")
        if self.statistic == "period" and self.quantity.split(":")[0] in ("state", "count"):
            errors.append(f"statistic 'period' needs a series, not {self.quantity!r}")
        expected = self.expected_dimension()
        for value, what in ((self.target, "target"), (self.limit, "limit")):
            error = _dimension_error(value, expected, what)
            if error is not None:
                errors.append(error)
        if expected is None and isinstance(self.target, Quantity) and isinstance(self.limit, Quantity):
            error = _dimension_error(self.limit, self.target.dimension, "limit")
            if error is not None:
                errors.append(error)
        _raise_if(errors)
        return self

    def expected_dimension(self) -> str | None:
        if self.statistic == "period":
            return TIME
        return reference_dimension(self.quantity)

    def limit_text(self) -> str:
        if self.comparison == "within":
            tolerance = str(self.limit)
            if isinstance(self.relative_limit, float):
                tolerance = f"{self.relative_limit * 100.0:g} percent"
            elif self.relative_limit is not None:
                tolerance = str(self.relative_limit)
            return f"{self.statistic} within {tolerance} of {self.target}"
        return f"{self.statistic} {self.comparison} {self.limit}"


class IntervalCriterion(_Criterion):
    """The simulated time from one recorded event to the next (SCO-021)."""

    check: Literal["interval"]
    start_event: EventRef
    end_event: EventRef
    comparison: Ordering
    limit: Time

    def limit_text(self) -> str:
        return f"interval {self.comparison} {_text(self.limit, 's')}"


class EventCriterion(_Criterion):
    """Presence or absence of an operator message or controller state within a window (SCO-022)."""

    check: Literal["event"]
    event: EventRef
    expect: Literal["present", "absent"]
    window: Window | None = None

    def limit_text(self) -> str:
        return f"{self.event} {self.expect}"


class _Actuators(_Criterion):
    actuators: list[Name] = Field(min_length=1)
    window: Window | None = None

    @model_validator(mode="after")
    def _unique(self) -> _Actuators:
        errors: list[str] = []
        _duplicates(self.actuators, "actuator", errors)
        _raise_if(errors)
        return self


class NoMotionCriterion(_Actuators):
    """No actuator motion above `threshold` within the window (SCO-023)."""

    check: Literal["no_motion"]
    threshold: Amount

    @model_validator(mode="after")
    def _positive(self) -> NoMotionCriterion:
        magnitude = _magnitude(self.threshold)
        if magnitude is not None and magnitude <= 0.0:
            raise ValueError(f"threshold must be positive, got {self.threshold}")
        return self

    def limit_text(self) -> str:
        return f"no motion above {self.threshold}"


class NoOverlapCriterion(_Actuators):
    """No two commands active at once on the same actuator (SCO-025)."""

    check: Literal["no_overlap"]

    def limit_text(self) -> str:
        return "no overlapping commands"


class SettleCriterion(_Criterion):
    """Time for a series to enter and stay in `band` around `target` (its final value when absent) (SCO-024)."""

    check: Literal["settle_time"]
    quantity: QuantityRef
    band: Amount
    target: Amount | None = None
    comparison: Ordering
    limit: Time
    window: Window | None = None

    @model_validator(mode="after")
    def _consistent(self) -> SettleCriterion:
        errors: list[str] = []
        magnitude = _magnitude(self.band)
        if magnitude is not None and magnitude <= 0.0:
            errors.append(f"band must be positive, got {self.band}")
        expected = reference_dimension(self.quantity)
        for value, what in ((self.band, "band"), (self.target, "target")):
            error = _dimension_error(value, expected, what)
            if error is not None:
                errors.append(error)
        _raise_if(errors)
        return self

    def limit_text(self) -> str:
        return f"settle within {self.band} {self.comparison} {_text(self.limit, 's')}"


def _text(value: Any, unit: str) -> str:
    if isinstance(value, float):
        return f"{value:g} {unit}"
    return str(value)


Criterion = Annotated[
    ValueCriterion | IntervalCriterion | EventCriterion | NoMotionCriterion | NoOverlapCriterion | SettleCriterion,
    Field(discriminator="check"),
]


# ===== Sweeps and variations ===========================================


class Sweep(StrictModel):
    """One run per value of a design parameter: `${name}` for an open parameter, else `<section>.<item>.<field>`."""

    parameter: ParameterPath
    values: list[Annotated[str, Field(min_length=1)]] = Field(min_length=1)

    @model_validator(mode="after")
    def _unique(self) -> Sweep:
        errors: list[str] = []
        _duplicates(self.values, "sweep value", errors)
        _raise_if(errors)
        return self


class Variation(StrictModel):
    """A seeded spread about the nominal value: standard deviation (normal) or half width (uniform)."""

    parameter: ParameterPath
    distribution: Literal["normal", "uniform"]
    spread: Amount

    @model_validator(mode="after")
    def _positive(self) -> Variation:
        magnitude = _magnitude(self.spread)
        if magnitude is not None and magnitude <= 0.0:
            raise ValueError(f"spread must be positive, got {self.spread}")
        return self


class Variations(StrictModel):
    """Stored only: v1 runs use nominal values (Architecture doc, Seams reserved for the roadmap)."""

    count: int = Field(ge=1)
    parameters: list[Variation] = Field(min_length=1)

    @model_validator(mode="after")
    def _unique(self) -> Variations:
        errors: list[str] = []
        _duplicates([variation.parameter for variation in self.parameters], "varied parameter", errors)
        _raise_if(errors)
        return self


# ===== Test case =======================================================


class Case(StrictModel):
    """Root of `test_cases/<suite>/<id>[_<slug>].yaml`."""

    model_config = ConfigDict(extra="forbid", strict=True, title="TestCaseFile")

    id: CaseId
    title: str = Field(min_length=1)
    description: str | None = None
    design: DesignRef
    controller_mode: Literal["native", "emulated"] | None = None
    seed: int = Field(ge=0, le=2**64 - 1)
    rates: Rates | None = None
    open_parameters: list[OpenParameter] = Field(default_factory=list)
    initial_state: InitialState
    commands: list[Command] = Field(default_factory=list)
    faults: list[Fault] = Field(default_factory=list)
    end: EndCondition
    criteria: list[Criterion] = Field(min_length=1)
    sweeps: list[Sweep] = Field(default_factory=list)
    variations: Variations | None = None
    baseline: RepoPath | None = None

    @model_validator(mode="after")
    def _checks(self) -> Case:
        errors: list[str] = []
        _duplicates([parameter.name for parameter in self.open_parameters], "open parameter", errors)
        _duplicates([criterion.name for criterion in self.criteria], "criterion", errors)
        _duplicates([sweep.parameter for sweep in self.sweeps], "sweep parameter", errors)
        previous: float | None = None
        for index, command in enumerate(self.commands):
            if isinstance(command.at, float):
                if previous is not None and command.at < previous:
                    errors.append(
                        f"commands[{index}].at: {command.at} s is before the previous command at {previous} s"
                    )
                previous = command.at
        limit = self.end.limit()
        if limit is not None:
            for index, command in enumerate(self.commands):
                if isinstance(command.at, float) and command.at > limit:
                    errors.append(f"commands[{index}].at: {command.at} s is after end.max_duration {limit} s")
            for index, fault in enumerate(self.faults):
                if isinstance(fault.start, float) and fault.start >= limit:
                    errors.append(f"faults[{index}].start: {fault.start} s is not before end.max_duration {limit} s")
        _raise_if(errors)
        return self


def _duplicates(values: list[str], what: str, errors: list[str]) -> None:
    seen: set[str] = set()
    for value in values:
        if value in seen:
            errors.append(f"duplicate {what} {value!r}")
        seen.add(value)


# ===== Cross-check against the design ==================================


def cross_check(case: Case, design: Design) -> list[str]:
    """Every error between `case` and the design it names, each with its path (TST-001, TST-009)."""
    errors: list[str] = []
    if case.design.name != design.name:
        errors.append(f"design.name: case names design {case.design.name!r}, file holds {design.name!r}")
    if case.design.version != design.version:
        errors.append(f"design.version: case was written against {case.design.version}, design is {design.version}")
    declared = _check_open(case, design, errors)
    joints = {joint.name: joint for joint in design.joints}
    bodies = {body.name for body in design.bodies}
    components = {component.name for component in design.components}
    messages = {message.name: message for message in design.operator_messages}
    if design.firmware is None:
        if case.controller_mode is not None:
            errors.append("controller_mode: the design has no firmware, so the case takes no controller mode")
        if len(case.initial_state.controller) > 0:
            errors.append("initial_state.controller: the design has no controller")
    elif case.controller_mode is None:
        errors.append("controller_mode: the design has firmware; name native or emulated")
    for index, state in enumerate(case.initial_state.joints):
        path = f"initial_state.joints[{index}]"
        joint = joints.get(state.joint)
        if joint is None:
            errors.append(f"{path}.joint: unknown joint {state.joint!r}")
            continue
        if joint.type not in ("prismatic", "revolute"):
            errors.append(f"{path}.joint: initial state of a {joint.type} joint is not supported; single-DOF only")
            continue
        motion = state.motion()
        if motion is not None and (motion == "linear") != (joint.type == "prismatic"):
            errors.append(f"{path}: a {joint.type} joint takes {_joint_dimension(joint.type)} values")
        _check_open_dimension(state.position, _joint_dimension(joint.type), f"{path}.position", declared, errors)
    payload = case.initial_state.payload
    if payload is not None and payload.body is not None and payload.body not in bodies:
        errors.append(f"initial_state.payload.body: unknown body {payload.body!r}")
    for index, command in enumerate(case.commands):
        _check_command(index, command, messages, declared, errors)
    _check_faults(case, design, bodies, components, errors)
    for index, criterion in enumerate(case.criteria):
        _check_criterion(
            f"criteria[{index}]", criterion, design, declared, joints, bodies, components, messages, errors
        )
    if case.end.message is not None and case.end.message not in messages:
        errors.append(f"end.message: unknown operator message {case.end.message!r}")
    for index, sweep in enumerate(case.sweeps):
        _check_parameter(f"sweeps[{index}].parameter", sweep.parameter, design, declared, errors)
    if case.variations is not None:
        for index, variation in enumerate(case.variations.parameters):
            _check_parameter(f"variations.parameters[{index}].parameter", variation.parameter, design, declared, errors)
    return errors


def _joint_dimension(joint_type: str) -> str:
    if joint_type == "prismatic":
        return LENGTH
    return ANGLE


def _check_open(case: Case, design: Design, errors: list[str]) -> dict[str, OpenParameter]:
    found = list(iter_open_values(case, "case"))
    if design.status == "release":
        if len(case.open_parameters) > 0:
            errors.append("open_parameters: a case for a release design may not have open parameters")
        for path, value in found:
            errors.append(f"{path}: a case for a release design may not contain {value}")
        for path, text in _parameter_paths(case):
            if open_name(text) is not None:
                errors.append(f"{path}: a case for a release design may not contain {text}")
        return {}
    declared = {parameter.name: parameter for parameter in design.open_parameters}
    for parameter in case.open_parameters:
        if parameter.name in declared:
            errors.append(f"open_parameters: {parameter.name!r} is already an open parameter of the design")
        else:
            declared[parameter.name] = parameter
    used = {value.name for _, value in found}
    for parameter in case.open_parameters:
        if parameter.name not in used:
            errors.append(f"open_parameters: {parameter.name!r} is declared but never used")
    for path, value in found:
        if value.name not in declared:
            errors.append(f"{path}: references undeclared open parameter {value}")
            continue
        if value.expected != ANY:
            _check_open_dimension(value, value.expected, path, declared, errors)
    return declared


def _parameter_paths(case: Case) -> list[tuple[str, str]]:
    paths = [(f"sweeps[{index}].parameter", sweep.parameter) for index, sweep in enumerate(case.sweeps)]
    if case.variations is not None:
        for index, variation in enumerate(case.variations.parameters):
            paths.append((f"variations.parameters[{index}].parameter", variation.parameter))
    return paths


def _check_open_dimension(
    value: Any, expected: str | None, path: str, declared: dict[str, OpenParameter], errors: list[str]
) -> None:
    if not isinstance(value, OpenValue) or expected is None or value.name not in declared:
        return
    dimension = declared[value.name].dimension
    if dimension == CHOICE or expected == CHOICE:
        if dimension != expected:
            errors.append(f"{path}: open parameter {value.name!r} has dimension {dimension}, expected {expected}")
        return
    if not dimensions_equal(dimension, expected):
        errors.append(f"{path}: open parameter {value.name!r} has dimension {dimension}, expected {expected}")


def _check_value(value: Any, expected: str, path: str, declared: dict[str, OpenParameter], errors: list[str]) -> None:
    error = _dimension_error(value, expected, path)
    if error is not None:
        errors.append(error)
    _check_open_dimension(value, expected, path, declared, errors)


def _check_command(
    index: int, command: Command, messages: dict[str, Any], declared: dict[str, OpenParameter], errors: list[str]
) -> None:
    path = f"commands[{index}]"
    message = messages.get(command.message)
    if message is None:
        errors.append(f"{path}.message: unknown operator message {command.message!r}")
        return
    if message.source != OPERATOR:
        errors.append(f"{path}.message: {command.message!r} is sent by {message.source}, not the operator")
    fields = {field.name: field.dimension for field in message.fields}
    if sorted(command.payload) != sorted(fields):
        errors.append(
            f"{path}.payload: {command.message!r} takes fields {sorted(fields)}, got {sorted(command.payload)}"
        )
    for key, value in command.payload.items():
        if key in fields:
            _check_value(value, fields[key], f"{path}.payload.{key}", declared, errors)


def _check_faults(case: Case, design: Design, bodies: set[str], components: set[str], errors: list[str]) -> None:
    rails: set[str] = set()
    if design.power is not None:
        rails = {rail.name for rail in design.power.rails}
    for index, fault in enumerate(case.faults):
        path = f"faults[{index}]"
        if isinstance(fault, PowerLoss):
            if fault.rail not in rails:
                errors.append(f"{path}.rail: unknown power rail {fault.rail!r}")
        elif isinstance(fault, ControllerFreeze):
            if design.firmware is None:
                errors.append(f"{path}: controller_freeze needs a design with firmware")
        elif isinstance(fault, PayloadChange):
            if fault.target not in bodies:
                errors.append(f"{path}.target: unknown body {fault.target!r}")
        elif fault.target not in components:
            errors.append(f"{path}.target: unknown component instance {fault.target!r}")


def _check_reference(
    path: str,
    reference: str,
    design: Design,
    joints: dict[str, Any],
    bodies: set[str],
    messages: dict[str, Any],
    errors: list[str],
) -> None:
    kind, _, rest = reference.partition(":")
    if kind == "count":
        kind, _, rest = rest.partition(".")
    if kind == "signal" and rest not in design.signal_names():
        errors.append(f"{path}: unknown signal {rest!r}")
    if kind == "pose" and rest.partition(".")[0] not in bodies:
        errors.append(f"{path}: unknown body {rest.partition('.')[0]!r}")
    if kind == "joint" and rest.partition(".")[0] not in joints:
        errors.append(f"{path}: unknown joint {rest.partition('.')[0]!r}")
    if kind == "message" and rest not in messages:
        errors.append(f"{path}: unknown operator message {rest!r}")
    if kind == "state" and design.firmware is None:
        errors.append(f"{path}: controller state {rest!r} needs a design with firmware")


def _joint_reference_dimension(reference: str, joints: dict[str, Any]) -> str | None:
    kind, _, rest = reference.partition(":")
    name, _, component = rest.partition(".")
    if kind != "joint" or name not in joints:
        return None
    joint_type = joints[name].type
    if joint_type not in ("prismatic", "revolute"):
        return None
    dimension = _joint_dimension(joint_type)
    if component == "velocity":
        return f"{dimension} / [time]"
    return dimension


def _check_criterion(
    path: str,
    criterion: Any,
    design: Design,
    declared: dict[str, OpenParameter],
    joints: dict[str, Any],
    bodies: set[str],
    components: set[str],
    messages: dict[str, Any],
    errors: list[str],
) -> None:
    if isinstance(criterion, ValueCriterion | SettleCriterion):
        _check_reference(f"{path}.quantity", criterion.quantity, design, joints, bodies, messages, errors)
        expected = reference_dimension(criterion.quantity)
        if expected is None:
            expected = _joint_reference_dimension(criterion.quantity, joints)
        if isinstance(criterion, ValueCriterion):
            if criterion.statistic == "period":
                expected = TIME
            values = [(criterion.target, "target"), (criterion.limit, "limit")]
        else:
            values = [(criterion.target, "target"), (criterion.band, "band")]
        if expected is not None:
            for value, what in values:
                _check_value(value, expected, f"{path}.{what}", declared, errors)
    elif isinstance(criterion, IntervalCriterion):
        for event, what in ((criterion.start_event, "start_event"), (criterion.end_event, "end_event")):
            _check_reference(f"{path}.{what}", event, design, joints, bodies, messages, errors)
    elif isinstance(criterion, EventCriterion):
        _check_reference(f"{path}.event", criterion.event, design, joints, bodies, messages, errors)
    else:
        for actuator in criterion.actuators:
            if actuator not in components:
                errors.append(f"{path}.actuators: unknown component instance {actuator!r}")


def _check_parameter(
    path: str, parameter: str, design: Design, declared: dict[str, OpenParameter], errors: list[str]
) -> None:
    name = open_name(parameter)
    if name is not None:
        if name not in declared:
            errors.append(f"{path}: references undeclared open parameter {parameter}")
        return
    section, item_name, *fields = parameter.split(".")
    items = {item.name: item for item in getattr(design, section)}
    if item_name not in items:
        errors.append(f"{path}: {section} has no item {item_name!r}")
        return
    current: Any = items[item_name]
    for field in fields:
        if not isinstance(current, BaseModel) or field not in type(current).model_fields:
            errors.append(f"{path}: {parameter!r} does not name a design field")
            return
        current = getattr(current, field)


# ===== Files and schema ================================================


def _parse(path: Path) -> tuple[Case | None, list[str]]:
    try:
        data = yaml_loader().load(path.read_text(encoding="utf-8"))
    except Exception as error:
        return None, [f"{path}: YAML parse error: {error}"]
    try:
        case = Case.model_validate(data)
    except ValidationError as error:
        return None, [f"{path}: {message}" for message in format_errors(error)]
    return case, []


def _load(path: Path, designs_root: Path) -> tuple[Case | None, list[str]]:
    """Parse, validate and cross-check once; every error comes back together, each prefixed by the file."""
    case, errors = _parse(path)
    if case is None:
        return None, errors
    messages: list[str] = []
    stem = path.stem
    if stem != case.id and not stem.startswith(case.id + "_"):
        messages.append(f"id: file name {path.name!r} must be {case.id}.yaml or start with {case.id}_")
    if case.baseline is not None and not (path.parent / case.baseline).is_file():
        messages.append(f"baseline: missing baseline file {case.baseline!r}")
    design_path = designs_root / case.design.name / "design.yaml"
    if not design_path.is_file():
        messages.append(f"design.name: no design file {design_path}")
    else:
        try:
            design = load_design(design_path)
        except DesignError as error:
            messages.append(f"design: the design does not load: {error}")
        else:
            messages.extend(cross_check(case, design))
    if len(messages) > 0:
        return None, [f"{path}: {message}" for message in messages]
    return case, []


def validate_test_case_file(path: Path, designs_root: Path = DESIGNS_ROOT) -> list[str]:
    """Every error in the test case file at `path`, each prefixed with the file; empty when it is valid (TST-009)."""
    return _load(path, designs_root)[1]


def load_test_case(path: Path, designs_root: Path = DESIGNS_ROOT) -> Case:
    case, errors = _load(path, designs_root)
    if case is None:
        raise CaseError("\n".join(errors))
    return case


def case_json_schema() -> dict[str, Any]:
    schema = Case.model_json_schema()
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["$id"] = SCHEMA_PATH.name
    return schema


def schema_text() -> str:
    return json.dumps(case_json_schema(), indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Test case file schema")
    parser.add_argument("--write-schema", action="store_true", help=f"write {SCHEMA_PATH}")
    parser.add_argument("--validate", type=Path, nargs="*", default=[], help="test case files to validate")
    arguments = parser.parse_args(argv)
    if arguments.write_schema:
        SCHEMA_PATH.write_text(schema_text(), encoding="utf-8")
        return 0
    if len(arguments.validate) > 0:
        status = 0
        for path in arguments.validate:
            for message in validate_test_case_file(path):
                sys.stderr.write(message + "\n")
                status = 1
        return status
    sys.stdout.write(schema_text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
