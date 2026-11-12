"""Invariant tier: the test case and baseline schemas are current, and every shipped file under `test_cases/`
validates (invariant 3)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import jsonschema
import pytest

from mechsim.design.schema import yaml_loader
from mechsim.testing import baseline as baseline_schema
from mechsim.testing import schema as case_schema

ROOT = Path(__file__).resolve().parents[2]
BASELINES = sorted((ROOT / "test_cases").glob("*/baseline.yaml"))
SUITES = sorted((ROOT / "test_cases").glob("*/suite.yaml"))
CASES = sorted(
    path for path in (ROOT / "test_cases").glob("*/*.yaml") if path.name not in ("baseline.yaml", "suite.yaml")
)


@pytest.mark.req("TST-009")
@pytest.mark.parametrize("module", [case_schema, baseline_schema], ids=["test_case", "baseline"])
def test_schema_is_current_and_well_formed(module: Any) -> None:
    path: Path = module.SCHEMA_PATH
    assert path.read_text(encoding="utf-8") == module.schema_text()
    schema = json.loads(path.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert schema["$id"] == path.name


@pytest.mark.req("TST-009")
def test_reference_cases_ship() -> None:
    assert [path.relative_to(ROOT).as_posix() for path in CASES] == [
        "test_cases/gantry/G1.yaml",
        "test_cases/gantry/G10.yaml",
        "test_cases/gantry/G11.yaml",
        "test_cases/gantry/G13.yaml",
        "test_cases/gantry/G14.yaml",
        "test_cases/gantry/G2.yaml",
        "test_cases/gantry/G3.yaml",
        "test_cases/gantry/G4.yaml",
        "test_cases/gantry/G5.yaml",
        "test_cases/gantry/G6.yaml",
        "test_cases/gantry/G7.yaml",
        "test_cases/gantry/G8.yaml",
        "test_cases/gantry/G9.yaml",
        "test_cases/pendulum/P1_period.yaml",
        "test_cases/pendulum/P2_damping_sweep.yaml",
    ]


@pytest.mark.req("TST-009")
@pytest.mark.parametrize("path", CASES, ids=[path.stem for path in CASES])
def test_shipped_case_validates(path: Path) -> None:
    schema = json.loads(case_schema.SCHEMA_PATH.read_text(encoding="utf-8"))
    data = yaml_loader().load(path.read_text(encoding="utf-8"))
    assert [error.message for error in jsonschema.Draft202012Validator(schema).iter_errors(data)] == []
    assert case_schema.validate_test_case_file(path) == []


@pytest.mark.req("SCO-009")
@pytest.mark.parametrize("path", BASELINES, ids=[path.parent.name for path in BASELINES])
def test_shipped_baseline_validates_against_its_suite(path: Path) -> None:
    schema = json.loads(baseline_schema.SCHEMA_PATH.read_text(encoding="utf-8"))
    data = yaml_loader().load(path.read_text(encoding="utf-8"))
    assert [error.message for error in jsonschema.Draft202012Validator(schema).iter_errors(data)] == []
    loaded = baseline_schema.load_baseline(path)
    assert loaded.suite == path.parent.name
    cases = [case_schema.load_test_case(case) for case in CASES if case.parent == path.parent]
    assert baseline_schema.check_against_cases(loaded, cases) == []
    assert baseline_schema.baseline_text(loaded) == path.read_text(encoding="utf-8")


@pytest.mark.req("TST-008")
@pytest.mark.parametrize("path", SUITES, ids=[path.parent.name for path in SUITES])
def test_shipped_suite_validates(path: Path) -> None:
    schema = json.loads(case_schema.SUITE_SCHEMA_PATH.read_text(encoding="utf-8"))
    jsonschema.validate(yaml_loader().load(path.read_text(encoding="utf-8")), schema)
    suite = case_schema.load_suite_file(path)
    for case in suite.cases:
        assert (path.parent / str(case)).resolve() in CASES
    assert (path.parent / str(suite.baseline)).is_file()
