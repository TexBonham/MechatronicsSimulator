"""Invariant tier: component and material schemas are current, and every shipped YAML validates (invariant 3)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import jsonschema
import pytest

from mechsim.library import schema as component_schema
from mechsim.materials import schema as material_schema

CATALOG = sorted(component_schema.CATALOG_DIR.glob("*.yaml"))
MATERIALS = sorted(material_schema.DATA_DIR.glob("*.yaml"))


def validator(path: Path) -> Any:
    schema = json.loads(path.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    return jsonschema.Draft202012Validator(schema)


@pytest.mark.req("LIB-011")
def test_component_schema_is_current() -> None:
    assert component_schema.SCHEMA_PATH.name == "component.json"
    assert component_schema.SCHEMA_PATH.read_text(encoding="utf-8") == component_schema.schema_text()


@pytest.mark.req("MAT-018")
def test_material_schema_is_current() -> None:
    assert material_schema.SCHEMA_PATH.name == "material.json"
    assert material_schema.SCHEMA_PATH.read_text(encoding="utf-8") == material_schema.schema_text()


@pytest.mark.req("LIB-011")
@pytest.mark.req("MAT-018")
@pytest.mark.parametrize(
    ("schema_path", "name"),
    [(component_schema.SCHEMA_PATH, "component.json"), (material_schema.SCHEMA_PATH, "material.json")],
)
def test_schema_declares_dialect_and_id(schema_path: Path, name: str) -> None:
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert schema["$id"] == name


@pytest.mark.req("LIB-010")
def test_catalog_folder_is_not_empty() -> None:
    assert len(CATALOG) >= 8


@pytest.mark.req("MAT-018")
def test_materials_folder_is_not_empty() -> None:
    assert len(MATERIALS) >= 5


@pytest.mark.req("LIB-010")
@pytest.mark.parametrize("path", CATALOG, ids=[path.stem for path in CATALOG])
def test_shipped_catalog_file_validates(path: Path) -> None:
    data = component_schema.yaml_loader().load(path.read_text(encoding="utf-8"))
    errors = [error.message for error in validator(component_schema.SCHEMA_PATH).iter_errors(data)]
    assert errors == []
    component_schema.load_catalog_file(path)


@pytest.mark.req("MAT-018")
@pytest.mark.parametrize("path", MATERIALS, ids=[path.stem for path in MATERIALS])
def test_shipped_material_file_validates(path: Path) -> None:
    data = material_schema.yaml_loader().load(path.read_text(encoding="utf-8"))
    errors = [error.message for error in validator(material_schema.SCHEMA_PATH).iter_errors(data)]
    assert errors == []
    material_schema.load_material_file(path)


@pytest.mark.req("LIB-013")
@pytest.mark.req("MAT-019")
@pytest.mark.parametrize(
    ("schema_path", "document"),
    [
        (component_schema.SCHEMA_PATH, {"types": [], "parts": []}),
        (material_schema.SCHEMA_PATH, {"name": "x"}),
    ],
)
def test_json_schema_rejects_invalid_documents(schema_path: Path, document: dict[str, Any]) -> None:
    assert list(validator(schema_path).iter_errors(document)) != []


@pytest.mark.req("MAT-019")
def test_json_schema_rejects_bare_number_quantity() -> None:
    data = material_schema.yaml_loader().load(
        (material_schema.DATA_DIR / "pla_printed.yaml").read_text(encoding="utf-8")
    )
    data["density"]["typ"] = 1240
    assert list(validator(material_schema.SCHEMA_PATH).iter_errors(data)) != []
