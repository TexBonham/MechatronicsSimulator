"""Invariant tier: the design schema is current, and every shipped design validates (invariant 3)."""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import pytest

from mechsim.design import chip as chip_schema
from mechsim.design import schema as design_schema

ROOT = Path(__file__).resolve().parents[2]
DESIGNS = sorted((ROOT / "designs").glob("*/design.yaml"))
CHIPS = sorted((ROOT / "chips").glob("*/chip.yaml"))


@pytest.mark.req("DES-012")
def test_design_schema_is_current() -> None:
    assert design_schema.SCHEMA_PATH.name == "design.json"
    assert design_schema.SCHEMA_PATH.read_text(encoding="utf-8") == design_schema.schema_text()


@pytest.mark.req("DES-012")
def test_design_schema_declares_dialect_and_id() -> None:
    schema = json.loads(design_schema.SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert schema["$id"] == "design.json"


@pytest.mark.req("DES-011")
def test_both_reference_designs_ship() -> None:
    assert [path.parent.name for path in DESIGNS] == ["gantry", "pendulum"]


@pytest.mark.req("DES-011")
@pytest.mark.parametrize("path", DESIGNS, ids=[path.parent.name for path in DESIGNS])
def test_shipped_design_validates(path: Path) -> None:
    schema = json.loads(design_schema.SCHEMA_PATH.read_text(encoding="utf-8"))
    data = design_schema.yaml_loader().load(path.read_text(encoding="utf-8"))
    errors = [error.message for error in jsonschema.Draft202012Validator(schema).iter_errors(data)]
    assert errors == []
    assert design_schema.validate_design_file(path) == []
    assert design_schema.load_design(path).name == path.parent.name


@pytest.mark.req("DES-029")
def test_chip_schema_is_current_and_declares_dialect_and_id() -> None:
    assert chip_schema.SCHEMA_PATH.name == "chip.json"
    assert chip_schema.SCHEMA_PATH.read_text(encoding="utf-8") == chip_schema.schema_text()
    schema = json.loads(chip_schema.SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert schema["$id"] == "chip.json"


@pytest.mark.req("DES-029")
def test_placeholder_chip_ships() -> None:
    assert [path.parent.name for path in CHIPS] == ["stm32_placeholder"]


@pytest.mark.req("DES-029")
@pytest.mark.parametrize("path", CHIPS, ids=[path.parent.name for path in CHIPS])
def test_shipped_chip_validates(path: Path) -> None:
    schema = json.loads(chip_schema.SCHEMA_PATH.read_text(encoding="utf-8"))
    data = chip_schema.yaml_loader().load(path.read_text(encoding="utf-8"))
    errors = [error.message for error in jsonschema.Draft202012Validator(schema).iter_errors(data)]
    assert errors == []
    assert chip_schema.validate_chip_file(path) == []
