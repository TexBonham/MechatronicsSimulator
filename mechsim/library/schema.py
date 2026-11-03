"""Component type definitions and catalog files (LIB). Generates `schemas/component.json`.

A catalog file holds one or more component types and the catalog parts that instantiate them. Type-level facets name
the parameters that carry their values; parts supply those values as unit strings.
"""

from __future__ import annotations

import argparse
import functools
import itertools
import json
import math
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Annotated, Any, Literal

import pint
from pydantic import (
    BeforeValidator,
    ConfigDict,
    Discriminator,
    Field,
    PrivateAttr,
    Tag,
    ValidationError,
    ValidationInfo,
    WithJsonSchema,
    field_validator,
    model_validator,
)
from ruamel.yaml import YAML

from mechsim.schema_base import StrictModel
from mechsim.units import UnitError, parse_quantity

SCHEMA_PATH = Path(__file__).resolve().parents[2] / "schemas" / "component.json"
CATALOG_DIR = Path(__file__).resolve().parent / "catalog"

QUANTITY_PATTERN = r"^\s*[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?(\s+\S.*)?$"
_QUANTITY_SCHEMA = {"type": "string", "pattern": QUANTITY_PATTERN}

Name = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_]*$")]
SemVer = Annotated[str, Field(pattern=r"^\d+\.\d+\.\d+$")]


def _parser(dimension: str) -> Callable[[Any], float]:
    def _parse(value: Any) -> float:
        return parse_quantity(value, dimension)

    return _parse


SIText = Annotated[float, WithJsonSchema(_QUANTITY_SCHEMA)]
LengthText = Annotated[float, BeforeValidator(_parser("[length]")), WithJsonSchema(_QUANTITY_SCHEMA)]
TorqueText = Annotated[
    float, BeforeValidator(_parser("[mass] * [length] ** 2 / [time] ** 2")), WithJsonSchema(_QUANTITY_SCHEMA)
]
SpeedText = Annotated[float, BeforeValidator(_parser("1 / [time]")), WithJsonSchema(_QUANTITY_SCHEMA)]
TemperatureText = Annotated[float, BeforeValidator(_parser("[temperature]")), WithJsonSchema(_QUANTITY_SCHEMA)]

Category = Literal["actuator", "driver", "sensor", "transmission", "power", "safety", "body", "joint"]
FailureMode = Literal["stuck", "noisy", "dropped", "forced_level"]
PinDirection = Literal["input", "output", "bidirectional", "power_in", "power_out", "ground"]
SignalKind = Literal["digital", "analog", "pwm", "step", "direction", "enable", "quadrature", "power", "ground", "coil"]

REQUIRED_BY_CATEGORY: dict[str, tuple[str, ...]] = {
    "transmission": ("ratio", "efficiency", "stiffness", "backlash", "self_locking", "slip"),
    "sensor": ("resolution", "noise", "delay", "sample_rate"),
}


class CatalogError(ValueError):
    """A catalog file failed to load; the message names the file."""


@functools.cache
def _dimension_registry() -> pint.UnitRegistry:
    return pint.UnitRegistry()


def dimensions_equal(left: str, right: str) -> bool:
    registry = _dimension_registry()
    return bool(registry.get_dimensionality(left) == registry.get_dimensionality(right))


# ===== Parameters ======================================================


class QuantityParameter(StrictModel):
    kind: Literal["quantity"] = "quantity"
    name: Name
    dimension: str = Field(min_length=1)
    default: SIText | None = None
    minimum: SIText | None = None
    maximum: SIText | None = None
    sweepable: bool = True
    description: str | None = None

    @model_validator(mode="before")
    @classmethod
    def _known_dimension(cls, data: Any) -> Any:
        if not isinstance(data, dict):
            return data
        dimension = data.get("dimension")
        if not isinstance(dimension, str):
            raise ValueError(f"parameter {data.get('name')!r} needs a dimension string, got {dimension!r}")
        try:
            _dimension_registry().get_dimensionality(dimension)
        except Exception as error:
            raise ValueError(f"parameter {data.get('name')!r}: unknown dimension {dimension!r}") from error
        return data

    @field_validator("default", "minimum", "maximum", mode="before")
    @classmethod
    def _parse(cls, value: Any, info: ValidationInfo) -> Any:
        if value is None:
            return None
        try:
            return parse_quantity(value, info.data["dimension"])
        except UnitError as error:
            raise ValueError(f"parameter {info.data.get('name')!r} {info.field_name}: {error}") from error

    @model_validator(mode="after")
    def _ordered(self) -> QuantityParameter:
        if self.minimum is not None and self.maximum is not None and self.minimum > self.maximum:
            raise ValueError(f"parameter {self.name!r}: minimum {self.minimum} above maximum {self.maximum}")
        if self.default is not None:
            check_bounds(self, self.default)
        return self


class FlagParameter(StrictModel):
    kind: Literal["flag"]
    name: Name
    default: bool | None = None
    sweepable: bool = True
    description: str | None = None


def _parameter_kind(value: Any) -> str:
    if isinstance(value, dict):
        return str(value.get("kind", "quantity"))
    return str(getattr(value, "kind", "quantity"))


Parameter = Annotated[
    Annotated[QuantityParameter, Tag("quantity")] | Annotated[FlagParameter, Tag("flag")],
    Discriminator(_parameter_kind),
]


def check_bounds(parameter: QuantityParameter, value: float) -> None:
    if parameter.minimum is not None and value < parameter.minimum:
        raise ValueError(f"parameter {parameter.name!r} value {value} below minimum {parameter.minimum}")
    if parameter.maximum is not None and value > parameter.maximum:
        raise ValueError(f"parameter {parameter.name!r} value {value} above maximum {parameter.maximum}")


class Tier(StrictModel):
    name: Name
    parameters: list[Name]


# ===== Facets ==========================================================


class MassFromParameter(StrictModel):
    source: Literal["parameter"]
    parameter: Name


class MassFromGeometry(StrictModel):
    source: Literal["geometry"]


class Mount(StrictModel):
    name: Name
    kind: Literal["face", "bolt_pattern", "shaft", "pad", "clamp"]
    offset: list[LengthText] = Field(min_length=3, max_length=3)


class MechanicalFacet(StrictModel):
    mass: MassFromParameter | MassFromGeometry = Field(discriminator="source")
    mounts: list[Mount]
    shaft_axis: list[float] | None = Field(default=None, min_length=3, max_length=3)

    @field_validator("shaft_axis")
    @classmethod
    def _unit_axis(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return None
        norm = math.sqrt(sum(component * component for component in value))
        if abs(norm - 1.0) > 1e-6:
            raise ValueError(f"shaft_axis must be a unit vector, norm is {norm}")
        return value


class Pin(StrictModel):
    name: Name
    direction: PinDirection
    signal_kind: SignalKind
    voltage_rating: Name | None = None
    current_rating: Name | None = None


class ElectricalFacet(StrictModel):
    pins: list[Pin]


class AbsoluteMaximum(StrictModel):
    name: Name
    parameter: Name


class RatingsFacet(StrictModel):
    absolute_maximums: list[AbsoluteMaximum]
    holding_torque: Name | None = None
    torque_speed_curve: Literal["required", "none"] = "none"


class ThermalFacet(StrictModel):
    held_constant: Literal[True]
    temperature: TemperatureText


class PluginReference(StrictModel):
    python: Annotated[str, Field(pattern=r"^[A-Za-z_][A-Za-z0-9_.]*:[A-Za-z_][A-Za-z0-9_]*$")] | None = None
    cpp: Name | None = None

    @model_validator(mode="after")
    def _one_path(self) -> PluginReference:
        if self.python is None and self.cpp is None:
            raise ValueError("plugin needs a python import path, a cpp built-in name, or both")
        return self


# ===== ComponentType ===================================================


class ComponentType(StrictModel):
    name: Name
    version: SemVer
    category: Category
    description: str | None = None
    parameters: list[Parameter] = Field(min_length=1)
    tiers: list[Tier] = Field(min_length=1)
    mechanical: MechanicalFacet
    electrical: ElectricalFacet
    ratings: RatingsFacet
    thermal: ThermalFacet
    failure_modes: list[FailureMode] = Field(default_factory=list)
    plugin: PluginReference

    def parameter(self, name: str) -> QuantityParameter | FlagParameter:
        for candidate in self.parameters:
            if candidate.name == name:
                return candidate
        raise KeyError(f"type {self.name!r} has no parameter {name!r}")

    def tier_parameters(self, tier: str) -> list[str]:
        """Parameter names active at `tier`: its own plus every lower tier's (tiers are additive, invariant 6)."""
        names: list[str] = []
        for candidate in self.tiers:
            names.extend(candidate.parameters)
            if candidate.name == tier:
                return names
        raise KeyError(f"type {self.name!r} has no tier {tier!r}")

    @model_validator(mode="after")
    def _consistent(self) -> ComponentType:
        names = [parameter.name for parameter in self.parameters]
        _unique(names, f"type {self.name!r} parameter")
        _unique([tier.name for tier in self.tiers], f"type {self.name!r} tier")
        _unique([pin.name for pin in self.electrical.pins], f"type {self.name!r} pin")
        _unique([mount.name for mount in self.mechanical.mounts], f"type {self.name!r} mount")
        _unique([item.name for item in self.ratings.absolute_maximums], f"type {self.name!r} absolute maximum")
        _unique(list(self.failure_modes), f"type {self.name!r} failure mode")
        placed: list[str] = []
        for tier in self.tiers:
            for parameter_name in tier.parameters:
                if parameter_name not in names:
                    raise ValueError(
                        f"type {self.name!r} tier {tier.name!r} references unknown parameter {parameter_name!r}"
                    )
                if parameter_name in placed:
                    raise ValueError(f"type {self.name!r} parameter {parameter_name!r} is placed in more than one tier")
                placed.append(parameter_name)
        for parameter_name in names:
            if parameter_name not in placed:
                raise ValueError(f"type {self.name!r} parameter {parameter_name!r} belongs to no tier")
        if isinstance(self.mechanical.mass, MassFromParameter):
            self._reference(self.mechanical.mass.parameter, "[mass]", "mechanical.mass")
        for pin in self.electrical.pins:
            if pin.voltage_rating is not None:
                self._reference(pin.voltage_rating, "[mass] * [length] ** 2 / [time] ** 3 / [current]", "pin voltage")
            if pin.current_rating is not None:
                self._reference(pin.current_rating, "[current]", "pin current")
        for item in self.ratings.absolute_maximums:
            self._reference(item.parameter, None, f"absolute maximum {item.name!r}")
        if self.ratings.holding_torque is not None:
            self._reference(self.ratings.holding_torque, "[mass] * [length] ** 2 / [time] ** 2", "holding_torque")
        for required in REQUIRED_BY_CATEGORY.get(self.category, ()):
            if required not in names:
                raise ValueError(f"{self.category} type {self.name!r} must carry parameter {required!r}")
        if self.category == "sensor" and len(self.failure_modes) == 0:
            raise ValueError(f"sensor type {self.name!r} must list the failure modes it supports")
        if self.category == "transmission":
            slip = self.parameter("slip")
            if not isinstance(slip, QuantityParameter) or slip.default != 0.0:
                raise ValueError(f"transmission type {self.name!r} parameter 'slip' must default to zero")
        return self

    def _reference(self, name: str, dimension: str | None, where: str) -> None:
        try:
            parameter = self.parameter(name)
        except KeyError as error:
            raise ValueError(f"type {self.name!r} {where} references unknown parameter {name!r}") from error
        if not isinstance(parameter, QuantityParameter):
            raise ValueError(f"type {self.name!r} {where} references flag parameter {name!r}")
        if dimension is not None and not dimensions_equal(parameter.dimension, dimension):
            raise ValueError(f"type {self.name!r} {where} parameter {name!r} must have dimension {dimension}")


def _unique(values: list[str], what: str) -> None:
    seen: set[str] = set()
    for value in values:
        if value in seen:
            raise ValueError(f"duplicate {what} {value!r}")
        seen.add(value)


# ===== CatalogPart =====================================================


class TorqueSpeedPoint(StrictModel):
    speed: SpeedText = Field(description="Shaft speed, for example '300 rpm'")
    torque: TorqueText

    @field_validator("speed", "torque")
    @classmethod
    def _non_negative(cls, value: float) -> float:
        if value < 0.0:
            raise ValueError("torque-speed values must be non-negative")
        return value


class PartEntry(StrictModel):
    """A catalog part as written in the file: values are unit strings or flags, resolved by `CatalogPart.resolve`."""

    name: Name
    type: Name
    tier: Name
    description: str | None = None
    source: str | None = None
    values: dict[str, Annotated[str, WithJsonSchema(_QUANTITY_SCHEMA)] | bool]
    placeholders: list[Name] = Field(default_factory=list)
    torque_speed_curve: list[TorqueSpeedPoint] | None = Field(default=None, min_length=2)

    @field_validator("torque_speed_curve")
    @classmethod
    def _increasing_speed(cls, value: list[TorqueSpeedPoint] | None) -> list[TorqueSpeedPoint] | None:
        if value is None:
            return None
        for earlier, later in itertools.pairwise(value):
            if later.speed <= earlier.speed:
                raise ValueError("torque_speed_curve speeds must be strictly increasing")
        return value


class CatalogPart(StrictModel):
    """A part resolved against its component type: every active parameter as an SI float or flag (LIB-033).

    Validation needs the type in context; build one with `CatalogPart.resolve(entry, component_type)`.
    """

    name: Name
    type: Name
    tier: Name
    description: str | None
    source: str | None
    values: dict[str, float | bool]
    placeholders: list[Name]
    torque_speed_curve: list[TorqueSpeedPoint] | None

    @classmethod
    def resolve(
        cls,
        entry: PartEntry | dict[str, Any],
        component: ComponentType,
        overrides: dict[str, str | bool] | None = None,
    ) -> CatalogPart:
        """Resolve `entry` against `component`; `overrides` replace entry values for one run, then re-resolve."""
        return cls.model_validate(entry, context={"component_type": component, "overrides": overrides})

    @model_validator(mode="before")
    @classmethod
    def _resolve(cls, data: Any, info: ValidationInfo) -> Any:
        context = info.context
        if not isinstance(context, dict) or not isinstance(context.get("component_type"), ComponentType):
            raise ValueError("a catalog part resolves only against its component type: use CatalogPart.resolve")
        component: ComponentType = context["component_type"]
        entry = data
        if not isinstance(entry, PartEntry):
            entry = PartEntry.model_validate(data)
        overrides = context.get("overrides")
        raw_values = dict(entry.values)
        if overrides is not None:
            raw_values.update(overrides)
        if entry.type != component.name:
            raise ValueError(f"part {entry.name!r} is of type {entry.type!r}, resolved against {component.name!r}")
        return {
            "name": entry.name,
            "type": entry.type,
            "tier": entry.tier,
            "description": entry.description,
            "source": entry.source,
            "values": _resolve_values(entry, component, raw_values),
            "placeholders": list(entry.placeholders),
            "torque_speed_curve": entry.torque_speed_curve,
        }

    def parameter_values(self) -> dict[str, float | bool]:
        """Every active parameter of this part in SI floats (or flags), defaults filled in."""
        return dict(self.values)


def _resolve_values(
    entry: PartEntry, component: ComponentType, raw_values: dict[str, str | bool]
) -> dict[str, float | bool]:
    if entry.tier not in [tier.name for tier in component.tiers]:
        raise ValueError(f"part {entry.name!r} selects tier {entry.tier!r} that type {component.name!r} does not name")
    _unique(list(entry.placeholders), f"part {entry.name!r} placeholder")
    for placeholder in entry.placeholders:
        if placeholder not in entry.values:
            raise ValueError(f"part {entry.name!r} marks {placeholder!r} as placeholder but sets no value for it")
    resolved: dict[str, float | bool] = {}
    for key, raw in raw_values.items():
        try:
            parameter = component.parameter(key)
        except KeyError as error:
            raise ValueError(
                f"part {entry.name!r} sets unknown parameter {key!r} of type {component.name!r}"
            ) from error
        resolved[key] = _resolve_value(entry.name, parameter, raw)
    for name in component.tier_parameters(entry.tier):
        if name in resolved:
            continue
        parameter = component.parameter(name)
        if parameter.default is None:
            raise ValueError(f"part {entry.name!r} of type {component.name!r} is missing required parameter {name!r}")
        resolved[name] = parameter.default
    if component.ratings.torque_speed_curve == "required" and entry.torque_speed_curve is None:
        raise ValueError(f"part {entry.name!r} of type {component.name!r} needs a torque_speed_curve")
    if component.ratings.torque_speed_curve == "none" and entry.torque_speed_curve is not None:
        raise ValueError(f"part {entry.name!r}: type {component.name!r} takes no torque_speed_curve")
    return resolved


def _resolve_value(part: str, parameter: QuantityParameter | FlagParameter, raw: str | bool) -> float | bool:
    if isinstance(parameter, FlagParameter):
        if not isinstance(raw, bool):
            raise ValueError(f"part {part!r} flag parameter {parameter.name!r} needs true or false, got {raw!r}")
        return raw
    if isinstance(raw, bool):
        raise ValueError(f"part {part!r} parameter {parameter.name!r} needs a quantity string, got {raw!r}")
    try:
        value = parse_quantity(raw, parameter.dimension)
    except UnitError as error:
        raise ValueError(
            f"part {part!r} parameter {parameter.name!r} expects dimension {parameter.dimension}: {error}"
        ) from error
    try:
        check_bounds(parameter, value)
    except ValueError as error:
        raise ValueError(f"part {part!r} {error} (given {raw!r})") from error
    return value


# ===== CatalogFile =====================================================


class CatalogFile(StrictModel):
    """Root of a file under `mechsim/library/catalog/`: component types and the parts that instantiate them."""

    model_config = ConfigDict(extra="forbid", strict=True, title="ComponentCatalogFile")

    types: list[ComponentType] = Field(min_length=1)
    parts: list[PartEntry]

    _resolved: list[CatalogPart] = PrivateAttr(default_factory=list)

    def resolved_parts(self) -> list[CatalogPart]:
        """The file's parts resolved against their types; filled whenever the file validates."""
        return list(self._resolved)

    def component_type(self, name: str) -> ComponentType:
        for component in self.types:
            if component.name == name:
                return component
        raise KeyError(f"catalog file has no component type {name!r}")

    @model_validator(mode="after")
    def _resolve_parts(self) -> CatalogFile:
        _unique([component.name for component in self.types], "component type")
        _unique([part.name for part in self.parts], "catalog part")
        by_name = {component.name: component for component in self.types}
        resolved: list[CatalogPart] = []
        for part in self.parts:
            if part.type not in by_name:
                raise ValueError(f"part {part.name!r} references unknown component type {part.type!r}")
            resolved.append(_resolve_in_file(part, by_name[part.type]))
        self._resolved = resolved
        return self


def _resolve_in_file(entry: PartEntry, component: ComponentType) -> CatalogPart:
    try:
        return CatalogPart.resolve(entry, component)
    except ValidationError as error:
        messages = "; ".join(str(item["msg"]) for item in error.errors())
        raise ValueError(messages) from error


def yaml_loader() -> YAML:
    """Round-trip YAML (L6) with the indentation the shipped files use, so a rewrite keeps them byte-identical."""
    yaml = YAML(typ="rt")
    yaml.indent(mapping=2, sequence=4, offset=2)
    yaml.width = 120
    yaml.preserve_quotes = True
    return yaml


def load_catalog_file(path: Path) -> CatalogFile:
    try:
        data = yaml_loader().load(path.read_text(encoding="utf-8"))
    except Exception as error:
        raise CatalogError(f"{path}: YAML parse error: {error}") from error
    try:
        return CatalogFile.model_validate(data)
    except ValidationError as error:
        raise CatalogError(f"{path}: {error}") from error


def component_json_schema() -> dict[str, Any]:
    schema = CatalogFile.model_json_schema()
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["$id"] = SCHEMA_PATH.name
    return schema


def schema_text() -> str:
    return json.dumps(component_json_schema(), indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Component catalog schema")
    parser.add_argument("--write-schema", action="store_true", help=f"write {SCHEMA_PATH}")
    arguments = parser.parse_args(argv)
    if arguments.write_schema:
        SCHEMA_PATH.write_text(schema_text(), encoding="utf-8")
        return 0
    sys.stdout.write(schema_text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
