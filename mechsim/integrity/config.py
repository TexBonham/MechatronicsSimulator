"""Integrity configuration (INT-002, INT-003, INT-036): which checks apply to a design, each one's fidelity tier and
parameters, from `integrity.yaml` beside the design file. Generates `schemas/integrity.json`; each registered
check's entry is generated from its parameters model, so a new check adds its own schema entry (INT-001).

`halt_on_crossing` mirrors the test case's `integrity.halt_on_crossing` (round 5 ruling 4) for the tier 1 party;
`thermal` is the thermal check slot v1 accepts and does not evaluate (INT-036)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Annotated, Any, Literal, Union, cast

from pydantic import Field, JsonValue, ValidationError, create_model, model_validator

from mechsim.integrity.plugin import IntegrityCheck, IntegrityError
from mechsim.integrity.registry import discover
from mechsim.materials.schema import yaml_loader
from mechsim.schema_base import StrictModel, format_errors

SCHEMA_PATH = Path(__file__).resolve().parents[2] / "schemas" / "integrity.json"
FILE_NAME = "integrity.yaml"
TABLE_CHECKS = ("strength", "deflection")
REGISTRY = discover()


def _entry_model(check: IntegrityCheck) -> type[StrictModel]:
    literal = cast(Any, Literal)
    tier_type: Any = literal[check.tiers]
    parameters: Any
    try:
        check.parameters()
        parameters = (check.parameters, Field(default_factory=check.parameters))
    except ValidationError:
        parameters = (check.parameters, ...)
    title = "".join(part.capitalize() for part in check.name.split("_"))
    model: type[StrictModel] = create_model(
        f"{title}CheckEntry",
        __base__=StrictModel,
        check=(literal[check.name], ...),
        tier=(tier_type, Field(default=check.tiers[0], description="Fidelity tier (INT-003).")),
        parameters=parameters,
    )
    return model


ENTRY_MODELS = tuple(_entry_model(check) for check in REGISTRY.values())
CheckEntry: Any = Annotated[Union[ENTRY_MODELS], Field(discriminator="check")]  # noqa: UP007


class ThermalSlot(StrictModel):
    """Thermal check slot (INT-036): accepted and recorded, not evaluated in v1."""

    parameters: dict[str, JsonValue] = Field(default_factory=dict)


class IntegrityConfig(StrictModel):
    """Root of `designs/<name>/integrity.yaml`."""

    schema_version: Literal[1]
    halt_on_crossing: bool = False
    checks: list[CheckEntry] = Field(default_factory=list)
    thermal: ThermalSlot | None = None

    @model_validator(mode="after")
    def _unique(self) -> IntegrityConfig:
        names = [str(entry.check) for entry in self.checks]
        repeated = sorted({name for name in names if names.count(name) > 1})
        if repeated:
            raise ValueError(f"check listed more than once: {', '.join(repeated)}")
        return self

    def entries(self) -> list[tuple[IntegrityCheck, int, StrictModel]]:
        """(check, tier, parameters) to evaluate: the listed checks, and the table checks (strength, deflection) at
        their first tier with default parameters when not listed (INT-022 needs them)."""
        listed: dict[str, Any] = {str(entry.check): entry for entry in self.checks}
        out: list[tuple[IntegrityCheck, int, StrictModel]] = []
        for name in TABLE_CHECKS:
            if name not in listed:
                check = REGISTRY[name]
                out.append((check, check.tiers[0], check.parameters()))
        for name, entry in listed.items():
            out.append((REGISTRY[name], int(entry.tier), entry.parameters))
        return out


def default_config() -> IntegrityConfig:
    return IntegrityConfig(schema_version=1)


def parse_config(data: Any, origin: str) -> IntegrityConfig:
    try:
        return IntegrityConfig.model_validate(data)
    except ValidationError as error:
        raise IntegrityError("\n".join(f"{origin}: {line}" for line in format_errors(error))) from error


def load_config(path: Path) -> IntegrityConfig:
    try:
        data = yaml_loader().load(path.read_text(encoding="utf-8"))
    except Exception as error:
        raise IntegrityError(f"{path}: cannot read integrity configuration: {error}") from error
    return parse_config(data, str(path))


def config_for(design_dir: Path) -> IntegrityConfig:
    """The design's `integrity.yaml`, or the default (table checks only) when it has none."""
    path = design_dir / FILE_NAME
    if not path.exists():
        return default_config()
    return load_config(path)


def integrity_json_schema() -> dict[str, Any]:
    schema = IntegrityConfig.model_json_schema()
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["$id"] = SCHEMA_PATH.name
    schema["title"] = "IntegrityFile"
    return schema


def schema_text() -> str:
    return json.dumps(integrity_json_schema(), indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Integrity configuration schema")
    parser.add_argument("--write-schema", action="store_true", help=f"write {SCHEMA_PATH}")
    arguments = parser.parse_args(argv)
    if arguments.write_schema:
        SCHEMA_PATH.write_text(schema_text(), encoding="utf-8")
        return 0
    sys.stdout.write(schema_text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
