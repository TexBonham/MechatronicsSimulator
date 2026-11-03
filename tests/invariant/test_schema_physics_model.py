"""The committed physics model schema equals a fresh generation, and the worked example validates against it."""

from __future__ import annotations

import json
from pathlib import Path

import jsonschema
import pytest
from ruamel.yaml import YAML

from mechsim.physics import neutral

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = ROOT / "schemas" / "physics_model.json"
EXAMPLES = sorted((ROOT / "schemas" / "examples").glob("physics_model.*.yaml"))


@pytest.mark.req("INV-013")
def test_committed_schema_is_not_stale() -> None:
    assert neutral.SCHEMA_PATH == SCHEMA
    assert SCHEMA.read_text(encoding="utf-8") == neutral.schema_text(), (
        "schemas/physics_model.json is stale: run python -m mechsim.physics.neutral --write-schema"
    )


@pytest.mark.req("INV-012")
@pytest.mark.req("INV-003")
@pytest.mark.parametrize("path", EXAMPLES, ids=[path.name for path in EXAMPLES])
def test_examples_validate_against_committed_schema(path: Path) -> None:
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.validate(YAML(typ="safe", pure=True).load(path.read_text(encoding="utf-8")), schema)
    neutral.from_yaml(path.read_text(encoding="utf-8"))


@pytest.mark.req("INV-012")
def test_examples_exist() -> None:
    assert EXAMPLES
