"""The schema validation hook takes a data file's kind from the CLI's `file_kind` (FAC-010, INT-002), so an
`integrity.yaml` beside a design validates against the integrity schema, not the design schema (brief I3 item 5)."""

from __future__ import annotations

import importlib.util
import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

from mechsim.testing.schema import ROOT

HOOK = ROOT / "tools" / "hooks" / "yaml_check_hook.py"


def _hook(path: Path, cwd: Path) -> subprocess.CompletedProcess[str]:
    payload = json.dumps({"tool_name": "Write", "tool_input": {"file_path": str(path)}, "cwd": str(cwd)})
    return subprocess.run(
        [sys.executable, str(HOOK)], input=payload, capture_output=True, text=True, check=False, cwd=cwd
    )


def _repo(tmp_path: Path) -> Path:
    (tmp_path / ".git").mkdir()
    shutil.copytree(ROOT / "schemas", tmp_path / "schemas")
    return tmp_path


@pytest.mark.req("INT-002")
@pytest.mark.parametrize(
    "relative",
    [
        "designs/gantry/integrity.yaml",
        "designs/cantilever_beam/integrity.yaml",
        "designs/gantry/design.yaml",
        "test_cases/gantry/G8.yaml",
        "test_cases/gantry/suite.yaml",
        "test_cases/gantry/baseline.yaml",
    ],
)
def test_shipped_data_files_pass_under_their_own_kind(relative: str) -> None:
    result = _hook(ROOT / relative, ROOT)
    assert result.returncode == 0, result.stderr
    assert result.stderr == ""


@pytest.mark.req("INT-002")
def test_an_invalid_integrity_file_is_reported_against_the_integrity_schema(tmp_path: Path) -> None:
    root = _repo(tmp_path)
    folder = root / "designs" / "machine"
    folder.mkdir(parents=True)
    path = folder / "integrity.yaml"
    path.write_text("schema_version: 1\nchecks:\n  - check: strength\n    tier: 9\n", encoding="utf-8")
    result = _hook(path, root)
    assert result.returncode == 2
    assert "schema integrity:" in result.stderr
    assert "schema design:" not in result.stderr


@pytest.mark.req("INT-002")
def test_a_design_file_without_its_keys_still_validates_as_a_design(tmp_path: Path) -> None:
    root = _repo(tmp_path)
    folder = root / "designs" / "machine"
    folder.mkdir(parents=True)
    path = folder / "design.yaml"
    path.write_text("name: machine\n", encoding="utf-8")
    result = _hook(path, root)
    assert result.returncode == 2
    assert "schema design: <root>: 'bodies' is a required property" in result.stderr


@pytest.mark.req("INT-002")
def test_an_unparsable_file_is_a_parse_error(tmp_path: Path) -> None:
    root = _repo(tmp_path)
    folder = root / "designs" / "machine"
    folder.mkdir(parents=True)
    path = folder / "integrity.yaml"
    path.write_text("checks: [\n", encoding="utf-8")
    result = _hook(path, root)
    assert result.returncode == 2
    assert "YAML parse error" in result.stderr


def _hook_module() -> Any:
    spec = importlib.util.spec_from_file_location("yaml_check_hook", HOOK)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.req("INT-002")
@pytest.mark.parametrize(
    ("text", "encoding"),
    [("checks: [\n", "utf-8"), ("name: é\n", "latin-1")],
)
def test_the_cli_kind_of_an_unreadable_file_is_none(tmp_path: Path, text: str, encoding: str) -> None:
    path = tmp_path / "broken.yaml"
    path.write_bytes(text.encode(encoding))
    assert _hook_module().cli_kind(path) is None
    assert _hook_module().cli_kind(tmp_path / "missing.yaml") is None


@pytest.mark.req("INT-002")
def test_an_unexpected_file_kind_failure_surfaces(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    from mechsim.facade import files

    def broken(path: Path) -> str | None:
        raise RuntimeError("defect in file_kind")

    monkeypatch.setattr(files, "file_kind", broken)
    path = tmp_path / "design.yaml"
    path.write_text("bodies: []\njoints: []\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="defect in file_kind"):
        _hook_module().cli_kind(path)
