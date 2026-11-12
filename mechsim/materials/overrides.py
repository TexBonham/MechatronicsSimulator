"""Material overrides file (MAT-016, 017). Generates `schemas/material_overrides.json`.

A design references one overrides file (`materials.overrides: <path>`, the field brief D adds). Each entry replaces a
shipped property for one material without editing the shipped database file, and carries the `source` of the new
values (for example "coupon test 2026-11-01").
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Annotated, Any

from pydantic import ConfigDict, Field, PrivateAttr, ValidationError, field_validator, model_validator

from mechsim.materials.schema import (
    QUANTITY_PATTERN,
    SPREAD_PROPERTIES,
    Material,
    MaterialError,
    SNPoint,
    Spread,
    StrengthFactors,
    TemperaturePoint,
    sn_monotone,
    yaml_loader,
)
from mechsim.schema_base import StrictModel, format_errors

SCHEMA_PATH = Path(__file__).resolve().parents[2] / "schemas" / "material_overrides.json"
DESIGN_FIELD = "materials.overrides"
METADATA_KEY = "material_overrides"

QuantityString = Annotated[str, Field(pattern=QUANTITY_PATTERN)]
Source = Annotated[str, Field(min_length=1)]


class PropertyOverride(StrictModel):
    """Either `value` (min = typ = max) or the full `min`, `typ`, `max` spread. A `temperature_dependence` given here
    replaces the shipped table; omitted, the shipped table is kept."""

    source: Source
    value: QuantityString | None = None
    min: QuantityString | None = None
    typ: QuantityString | None = None
    max: QuantityString | None = None
    temperature_dependence: list[TemperaturePoint] | None = Field(default=None, min_length=2)

    @model_validator(mode="after")
    def _one_form(self) -> PropertyOverride:
        bounds = [self.min, self.typ, self.max]
        if self.value is not None:
            if any(bound is not None for bound in bounds):
                raise ValueError("give either value or min, typ and max, not both")
            return self
        if any(bound is None for bound in bounds):
            raise ValueError("give either value or all of min, typ and max")
        return self

    def spread_data(self) -> dict[str, Any]:
        data: dict[str, Any] = {"min": self.min, "typ": self.typ, "max": self.max}
        if self.value is not None:
            data = {"min": self.value, "typ": self.value, "max": self.value}
        if self.temperature_dependence is not None:
            data["temperature_dependence"] = self.temperature_dependence
        return data


class SNCurveOverride(StrictModel):
    source: Source
    points: list[SNPoint] = Field(min_length=2)

    @field_validator("points")
    @classmethod
    def _monotone(cls, value: list[SNPoint]) -> list[SNPoint]:
        return sn_monotone(value)


class StrengthFactorsOverride(StrengthFactors):
    source: Source


def _spread_type(field: str) -> type[Spread]:
    annotation = Material.model_fields[field].annotation
    if not isinstance(annotation, type) or not issubclass(annotation, Spread):
        raise TypeError(f"Material.{field} is not a spread")
    return annotation


class MaterialOverride(StrictModel):
    """Overrides for one shipped material; every field optional, at least one given."""

    density: PropertyOverride | None = None
    youngs_modulus: PropertyOverride | None = None
    poissons_ratio: PropertyOverride | None = None
    yield_strength: PropertyOverride | None = None
    ultimate_strength: PropertyOverride | None = None
    shear_strength: PropertyOverride | None = None
    thermal_conductivity: PropertyOverride | None = None
    specific_heat: PropertyOverride | None = None
    thermal_expansion: PropertyOverride | None = None
    max_service_temperature: PropertyOverride | None = None
    sn_curve: SNCurveOverride | None = None
    strength_factors: StrengthFactorsOverride | None = None

    _spreads: dict[str, Spread] = PrivateAttr(default_factory=dict)

    @model_validator(mode="after")
    def _resolve(self) -> MaterialOverride:
        spreads: dict[str, Spread] = {}
        for field in SPREAD_PROPERTIES:
            entry = getattr(self, field)
            if entry is None:
                continue
            try:
                spreads[field] = _spread_type(field).model_validate(entry.spread_data())
            except ValidationError as error:
                raise ValueError("\n".join(f"{field}: {line}" for line in format_errors(error))) from error
        if not spreads and self.sn_curve is None and self.strength_factors is None:
            raise ValueError("a material override must override at least one property")
        self._spreads = spreads
        return self

    def spreads(self) -> dict[str, Spread]:
        """Overridden spreads, validated with the shipped property's type (units, order, physical bounds)."""
        return dict(self._spreads)

    def sources(self) -> dict[str, str]:
        found: dict[str, str] = {}
        for field in (*SPREAD_PROPERTIES, "sn_curve", "strength_factors"):
            entry = getattr(self, field)
            if entry is not None:
                found[field] = entry.source
        return found


class MaterialOverridesFile(StrictModel):
    """Root of a `material_overrides.yaml` file."""

    model_config = ConfigDict(extra="forbid", strict=True, title="MaterialOverridesFile")

    description: str | None = None
    material_overrides: dict[str, MaterialOverride] = Field(min_length=1)


@dataclass(frozen=True)
class MaterialOverrides:
    """A loaded overrides file: its path, exact text (the recorded input) and validated content."""

    path: Path
    text: str
    file: MaterialOverridesFile

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.text.encode("utf-8")).hexdigest()


def load_overrides(path: Path) -> MaterialOverrides:
    """Load and validate an overrides file; every error names the file and the field."""
    try:
        text = path.read_text(encoding="utf-8")
        data = yaml_loader().load(text)
    except Exception as error:
        raise MaterialError(f"{path}: YAML parse error: {error}") from error
    try:
        file = MaterialOverridesFile.model_validate(data)
    except ValidationError as error:
        raise MaterialError("\n".join(f"{path}: {line}" for line in format_errors(error))) from error
    return MaterialOverrides(path=path, text=text, file=file)


def overrides_json_schema() -> dict[str, Any]:
    schema = MaterialOverridesFile.model_json_schema()
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["$id"] = SCHEMA_PATH.name
    return schema


def schema_text() -> str:
    return json.dumps(overrides_json_schema(), indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Material overrides schema")
    parser.add_argument("--write-schema", action="store_true", help=f"write {SCHEMA_PATH}")
    arguments = parser.parse_args(argv)
    if arguments.write_schema:
        SCHEMA_PATH.write_text(schema_text(), encoding="utf-8")
        return 0
    sys.stdout.write(schema_text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
