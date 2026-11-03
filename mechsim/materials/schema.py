"""Material records (MAT). Generates `schemas/material.json`.

One material per file under `mechsim/materials/data/`. Every property is a min, typ, max spread of unit strings held as
SI floats after load, with an optional temperature-dependence table of multiplying factors.
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Annotated, Any, ClassVar, Literal

from pydantic import (
    BeforeValidator,
    ConfigDict,
    Field,
    ValidationError,
    ValidationInfo,
    WithJsonSchema,
    field_validator,
    model_validator,
)
from ruamel.yaml import YAML

from mechsim.schema_base import StrictModel
from mechsim.units import UnitError, parse_quantity

SCHEMA_PATH = Path(__file__).resolve().parents[2] / "schemas" / "material.json"
DATA_DIR = Path(__file__).resolve().parent / "data"

QUANTITY_PATTERN = r"^\s*[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?(\s+\S.*)?$"
_QUANTITY_SCHEMA = {"type": "string", "pattern": QUANTITY_PATTERN}

PRESSURE = "[mass] / [length] / [time] ** 2"
TEMPERATURE = "[temperature]"


def _parser(dimension: str) -> Callable[[Any], float]:
    def _parse(value: Any) -> float:
        return parse_quantity(value, dimension)

    return _parse


SIText = Annotated[float, WithJsonSchema(_QUANTITY_SCHEMA)]
TemperatureText = Annotated[float, BeforeValidator(_parser(TEMPERATURE)), WithJsonSchema(_QUANTITY_SCHEMA)]
PressureText = Annotated[float, BeforeValidator(_parser(PRESSURE)), WithJsonSchema(_QUANTITY_SCHEMA)]
LengthText = Annotated[float, BeforeValidator(_parser("[length]")), WithJsonSchema(_QUANTITY_SCHEMA)]
FractionText = Annotated[float, BeforeValidator(_parser("dimensionless")), WithJsonSchema(_QUANTITY_SCHEMA)]


class MaterialError(ValueError):
    """A material file failed to load; the message names the file."""


def _strictly_increasing(values: list[float], what: str) -> None:
    for earlier, later in itertools.pairwise(values):
        if later <= earlier:
            raise ValueError(f"{what} must be strictly increasing")


# ===== Spreads =========================================================


class TemperaturePoint(StrictModel):
    temperature: TemperatureText
    factor: float = Field(gt=0.0)


class Spread(StrictModel):
    """min <= typ <= max, each a unit string of the subclass's dimension."""

    dimension: ClassVar[str] = "dimensionless"

    min: SIText
    typ: SIText
    max: SIText
    temperature_dependence: list[TemperaturePoint] | None = Field(default=None, min_length=2)

    @field_validator("min", "typ", "max", mode="before")
    @classmethod
    def _parse(cls, value: Any, info: ValidationInfo) -> float:
        try:
            return parse_quantity(value, cls.dimension)
        except UnitError as error:
            raise ValueError(f"{info.field_name}: {error}") from error

    @field_validator("temperature_dependence")
    @classmethod
    def _table(cls, value: list[TemperaturePoint] | None) -> list[TemperaturePoint] | None:
        if value is None:
            return None
        _strictly_increasing([point.temperature for point in value], "temperature_dependence temperatures")
        return value

    @model_validator(mode="after")
    def _ordered(self) -> Spread:
        if not (self.min <= self.typ <= self.max):
            raise ValueError(f"spread must satisfy min <= typ <= max, got {self.min}, {self.typ}, {self.max}")
        return self


class DensitySpread(Spread):
    dimension: ClassVar[str] = "[mass] / [length] ** 3"


class PressureSpread(Spread):
    dimension: ClassVar[str] = PRESSURE


class RatioSpread(Spread):
    dimension: ClassVar[str] = "dimensionless"


class ConductivitySpread(Spread):
    dimension: ClassVar[str] = "[mass] * [length] / [time] ** 3 / [temperature]"


class SpecificHeatSpread(Spread):
    dimension: ClassVar[str] = "[length] ** 2 / [time] ** 2 / [temperature]"


class ExpansionSpread(Spread):
    dimension: ClassVar[str] = "1 / [temperature]"


class TemperatureSpread(Spread):
    dimension: ClassVar[str] = TEMPERATURE


class PositiveDensity(DensitySpread):
    @model_validator(mode="after")
    def _positive(self) -> PositiveDensity:
        if self.min <= 0.0:
            raise ValueError(f"density min {self.min} must be positive")
        return self


class PositivePressure(PressureSpread):
    @model_validator(mode="after")
    def _positive(self) -> PositivePressure:
        if self.min <= 0.0:
            raise ValueError(f"min {self.min} must be positive")
        return self


class PoissonSpread(RatioSpread):
    @model_validator(mode="after")
    def _physical(self) -> PoissonSpread:
        if self.min <= -1.0 or self.max >= 0.5:
            raise ValueError(f"Poisson's ratio must lie in (-1, 0.5), got {self.min} to {self.max}")
        return self


class PositiveConductivity(ConductivitySpread):
    @model_validator(mode="after")
    def _positive(self) -> PositiveConductivity:
        if self.min <= 0.0:
            raise ValueError(f"min {self.min} must be positive")
        return self


class PositiveSpecificHeat(SpecificHeatSpread):
    @model_validator(mode="after")
    def _positive(self) -> PositiveSpecificHeat:
        if self.min <= 0.0:
            raise ValueError(f"min {self.min} must be positive")
        return self


class PositiveTemperature(TemperatureSpread):
    @model_validator(mode="after")
    def _positive(self) -> PositiveTemperature:
        if self.min <= 0.0:
            raise ValueError(f"min {self.min} K must be above absolute zero")
        return self


# ===== Fatigue =========================================================


class SNPoint(StrictModel):
    cycles: float = Field(ge=1.0)
    stress_amplitude: PressureText

    @field_validator("stress_amplitude")
    @classmethod
    def _positive(cls, value: float) -> float:
        if value <= 0.0:
            raise ValueError("stress_amplitude must be positive")
        return value


def _sn_monotone(value: list[SNPoint]) -> list[SNPoint]:
    _strictly_increasing([point.cycles for point in value], "S-N cycles")
    for earlier, later in itertools.pairwise(value):
        if later.stress_amplitude > earlier.stress_amplitude:
            raise ValueError("S-N stress_amplitude must not increase with cycles")
    return value


# ===== Processes =======================================================


class StrengthFactors(StrictModel):
    """Strength multipliers by loading direction relative to the build direction."""

    in_layer: float = Field(gt=0.0, le=1.0)
    interlayer: float = Field(gt=0.0, le=1.0)


class PrintedProcess(StrictModel):
    kind: Literal["printed"]
    build_direction: list[float] = Field(min_length=3, max_length=3)
    layer_height: LengthText
    walls: int = Field(ge=1)
    infill: FractionText
    infill_pattern: str | None = None
    strength_factors: StrengthFactors

    @field_validator("build_direction")
    @classmethod
    def _unit(cls, value: list[float]) -> list[float]:
        norm = sum(component * component for component in value) ** 0.5
        if abs(norm - 1.0) > 1e-6:
            raise ValueError(f"build_direction must be a unit vector, norm is {norm}")
        return value

    @field_validator("layer_height")
    @classmethod
    def _positive_layer(cls, value: float) -> float:
        if value <= 0.0:
            raise ValueError("layer_height must be positive")
        return value

    @field_validator("infill")
    @classmethod
    def _infill_range(cls, value: float) -> float:
        if value < 0.0 or value > 1.0:
            raise ValueError(f"infill must lie in 0 to 100 percent, got {value * 100.0} percent")
        return value


class MachinedProcess(StrictModel):
    kind: Literal["machined"]


class MoldedProcess(StrictModel):
    kind: Literal["molded"]


# ===== Material ========================================================


class Material(StrictModel):
    """Root of a file under `mechsim/materials/data/`."""

    model_config = ConfigDict(extra="forbid", strict=True, title="MaterialFile")

    name: Annotated[str, Field(pattern=r"^[a-z][a-z0-9_]*$")]
    family: Literal["aluminium", "steel", "polymer", "other"]
    grade: str | None = None
    description: str | None = None
    source: str
    placeholder: bool
    wide_spread_default: bool
    density: PositiveDensity
    youngs_modulus: PositivePressure
    poissons_ratio: PoissonSpread
    yield_strength: PositivePressure
    ultimate_strength: PositivePressure
    shear_strength: PositivePressure
    sn_curve: list[SNPoint] = Field(min_length=2)
    thermal_conductivity: PositiveConductivity
    specific_heat: PositiveSpecificHeat
    thermal_expansion: ExpansionSpread
    max_service_temperature: PositiveTemperature
    process: PrintedProcess | MachinedProcess | MoldedProcess | None = Field(default=None, discriminator="kind")

    @field_validator("sn_curve")
    @classmethod
    def _sn(cls, value: list[SNPoint]) -> list[SNPoint]:
        return _sn_monotone(value)


def yaml_loader() -> YAML:
    """Round-trip YAML (L6) with the indentation the shipped files use, so a rewrite keeps them byte-identical."""
    yaml = YAML(typ="rt")
    yaml.indent(mapping=2, sequence=4, offset=2)
    yaml.width = 120
    yaml.preserve_quotes = True
    return yaml


def load_material_file(path: Path) -> Material:
    try:
        data = yaml_loader().load(path.read_text(encoding="utf-8"))
    except Exception as error:
        raise MaterialError(f"{path}: YAML parse error: {error}") from error
    try:
        return Material.model_validate(data)
    except ValidationError as error:
        name = "<unnamed>"
        if isinstance(data, dict):
            name = str(data.get("name", name))
        raise MaterialError(f"{path}: material {name!r}: {error}") from error


def material_json_schema() -> dict[str, Any]:
    schema = Material.model_json_schema()
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["$id"] = SCHEMA_PATH.name
    return schema


def schema_text() -> str:
    return json.dumps(material_json_schema(), indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Material schema")
    parser.add_argument("--write-schema", action="store_true", help=f"write {SCHEMA_PATH}")
    arguments = parser.parse_args(argv)
    if arguments.write_schema:
        SCHEMA_PATH.write_text(schema_text(), encoding="utf-8")
        return 0
    sys.stdout.write(schema_text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
