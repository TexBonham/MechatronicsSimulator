"""The facade protocol schema is generated, current, and validates the worked examples (FAC-004, L4)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import jsonschema
import pytest

from mechsim.facade import protocol

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_FILE = ROOT / "schemas" / "facade_protocol.json"
EXAMPLES = sorted((ROOT / "schemas" / "examples").glob("facade_*.json"))


def _schema() -> dict[str, Any]:
    data: dict[str, Any] = json.loads(SCHEMA_FILE.read_text(encoding="utf-8"))
    return data


@pytest.mark.req("FAC-004")
def test_schema_lives_in_schemas() -> None:
    assert protocol.SCHEMA_PATH == SCHEMA_FILE
    assert SCHEMA_FILE.is_file()


@pytest.mark.req("FAC-004")
def test_schema_is_current() -> None:
    message = "schemas/facade_protocol.json is stale; run: uv run python -m mechsim.facade.protocol --write-schema"
    assert SCHEMA_FILE.read_text(encoding="utf-8") == protocol.render_schema(), message


@pytest.mark.req("FAC-004")
def test_schema_is_valid_draft_2020_12() -> None:
    jsonschema.Draft202012Validator.check_schema(_schema())


@pytest.mark.req("FAC-004")
def test_examples_present() -> None:
    names = {path.name for path in EXAMPLES}
    expected = {"facade_request.json", "facade_response.json", "facade_event.json", "facade_tick_batch.json"}
    assert expected <= names


@pytest.mark.req("FAC-004")
@pytest.mark.parametrize("path", EXAMPLES, ids=[path.name for path in EXAMPLES])
def test_example_validates_against_schema(path: Path) -> None:
    jsonschema.Draft202012Validator(_schema()).validate(json.loads(path.read_text(encoding="utf-8")))


@pytest.mark.req("FAC-004")
@pytest.mark.parametrize(
    "document",
    [
        {"version": 1, "message_id": "m", "kind": "nope", "payload": {}},
        {"version": 1, "message_id": "m", "kind": "scrub", "payload": {"run_id": "r", "t": -1.0}},
        {"version": 1, "message_id": "m", "kind": "scrub", "payload": {"run_id": "r", "t": 0.0, "x": 1}},
        {"version": 2, "message_id": "m", "kind": "scrub", "payload": {"run_id": "r", "t": 0.0}},
        {"version": 1, "message_id": "m", "kind": "error", "payload": {"request_id": None, "message": "x"}},
    ],
)
def test_schema_rejects_what_the_model_rejects(document: dict[str, Any]) -> None:
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.Draft202012Validator(_schema()).validate(document)
