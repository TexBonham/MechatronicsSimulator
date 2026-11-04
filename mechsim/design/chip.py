"""Chip definitions (DES-029, Decisions log row 11). Generates `schemas/chip.json`.

A chip is data: an emulator platform description, an SVD register map and a pin map, in `chips/<name>/chip.yaml`.
The pin map names every MCU pin and the peripheral modes it supports; a design's pin-binding table may only use a pin
and mode the map lists. A `placeholder` chip stands in until a real part is selected and may leave its platform
description and SVD as `placeholder`; a `verified` chip names both files, relative to its folder.
"""

from __future__ import annotations

import argparse
import functools
import json
import sys
from pathlib import Path
from typing import Annotated, Any, Literal

from pydantic import AfterValidator, ConfigDict, Field, ValidationError, model_validator
from ruamel.yaml import YAML

from mechsim.schema_base import StrictModel, format_errors

SCHEMA_PATH = Path(__file__).resolve().parents[2] / "schemas" / "chip.json"
CHIPS_DIR = Path(__file__).resolve().parents[2] / "chips"
CHIP_FILE = "chip.yaml"
PLACEHOLDER = "placeholder"

PeripheralMode = Literal["gpio_input", "gpio_output", "timer_pwm", "timer_encoder", "adc", "dac", "uart", "spi", "i2c"]

Name = Annotated[str, Field(pattern=r"^[a-z][a-z0-9_]*$")]
McuPin = Annotated[str, Field(pattern=r"^[A-Z][A-Z0-9_]*$")]


def _no_parent_segment(value: str) -> str:
    if ".." in value.split("/"):
        raise ValueError(f"path {value!r} may not leave its folder with '..'")
    return value


ChipFile = Annotated[str, Field(pattern=r"^[A-Za-z0-9_.\-]+(/[A-Za-z0-9_.\-]+)*$"), AfterValidator(_no_parent_segment)]


class ChipError(ValueError):
    """A chip definition failed to load; the message names the file and the reason."""


class ChipPin(StrictModel):
    name: McuPin
    modes: list[PeripheralMode] = Field(min_length=1)

    @model_validator(mode="after")
    def _unique_modes(self) -> ChipPin:
        seen: set[str] = set()
        for mode in self.modes:
            if mode in seen:
                raise ValueError(f"pin {self.name!r} lists mode {mode!r} more than once")
            seen.add(mode)
        return self


class Chip(StrictModel):
    """Root of `chips/<name>/chip.yaml`."""

    model_config = ConfigDict(extra="forbid", strict=True, title="ChipFile")

    name: Name
    family: str = Field(min_length=1)
    status: Literal["placeholder", "verified"]
    description: str | None = None
    platform_description: ChipFile
    svd: ChipFile
    pins: list[ChipPin] = Field(min_length=1)

    @model_validator(mode="after")
    def _consistent(self) -> Chip:
        errors: list[str] = []
        seen: set[str] = set()
        for index, pin in enumerate(self.pins):
            if pin.name in seen:
                errors.append(f"pins[{index}]: duplicate pin {pin.name!r}")
            seen.add(pin.name)
        if self.status == "verified":
            for field in ("platform_description", "svd"):
                if getattr(self, field) == PLACEHOLDER:
                    errors.append(f"{field}: a verified chip must name its file, not {PLACEHOLDER!r}")
        if len(errors) > 0:
            raise ValueError("\n".join(errors))
        return self

    def pin_modes(self) -> dict[str, list[str]]:
        return {pin.name: list(pin.modes) for pin in self.pins}


def check_files(chip: Chip, base: Path) -> list[str]:
    """Every file a chip names, other than a placeholder, must exist relative to the chip's folder."""
    errors: list[str] = []
    for field in ("platform_description", "svd"):
        value = getattr(chip, field)
        if value != PLACEHOLDER and not (base / value).is_file():
            errors.append(f"{field}: missing file {value!r}")
    return errors


def yaml_loader() -> YAML:
    yaml = YAML(typ="rt")
    yaml.indent(mapping=2, sequence=4, offset=2)
    yaml.width = 120
    yaml.preserve_quotes = True
    return yaml


def _load(path: Path) -> tuple[Chip | None, list[str]]:
    try:
        data = yaml_loader().load(path.read_text(encoding="utf-8"))
    except Exception as error:
        return None, [f"{path}: YAML parse error: {error}"]
    try:
        chip = Chip.model_validate(data)
    except ValidationError as error:
        return None, [f"{path}: {message}" for message in format_errors(error)]
    errors = [f"{path}: {message}" for message in check_files(chip, path.parent)]
    if chip.name != path.parent.name:
        errors.append(f"{path}: name: chip {chip.name!r} must match its folder {path.parent.name!r}")
    if len(errors) > 0:
        return None, errors
    return chip, []


def validate_chip_file(path: Path) -> list[str]:
    """Every error in the chip file at `path`, each prefixed with the file; empty when it is valid."""
    return _load(path)[1]


def load_chip(path: Path) -> Chip:
    chip, errors = _load(path)
    if chip is None:
        raise ChipError("\n".join(errors))
    return chip


def load_chips(directory: Path) -> tuple[dict[str, Chip], list[str]]:
    """Every chip under `directory/<name>/chip.yaml` by name, and every error loading them."""
    chips: dict[str, Chip] = {}
    errors: list[str] = []
    for path in sorted(directory.glob(f"*/{CHIP_FILE}")):
        chip, problems = _load(path)
        errors.extend(problems)
        if chip is not None:
            chips[chip.name] = chip
    return chips, errors


@functools.cache
def shipped_chips() -> dict[str, Chip]:
    chips, errors = load_chips(CHIPS_DIR)
    if len(errors) > 0:
        raise ChipError("\n".join(errors))
    return chips


def chip_json_schema() -> dict[str, Any]:
    schema = Chip.model_json_schema()
    schema["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    schema["$id"] = SCHEMA_PATH.name
    return schema


def schema_text() -> str:
    return json.dumps(chip_json_schema(), indent=2, sort_keys=True) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Chip definition schema")
    parser.add_argument("--write-schema", action="store_true", help=f"write {SCHEMA_PATH}")
    parser.add_argument("--validate", type=Path, nargs="*", default=[], help="chip files to validate")
    arguments = parser.parse_args(argv)
    if arguments.write_schema:
        SCHEMA_PATH.write_text(schema_text(), encoding="utf-8")
        return 0
    if len(arguments.validate) > 0:
        status = 0
        for path in arguments.validate:
            for message in validate_chip_file(path):
                sys.stderr.write(message + "\n")
                status = 1
        return status
    sys.stdout.write(schema_text())
    return 0


if __name__ == "__main__":
    sys.exit(main())
