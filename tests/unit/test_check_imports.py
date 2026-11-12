"""Unit tests for tools/check_imports.py, the import-boundary checker."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

import check_imports
from check_imports import RULES, Rule, Violation, check_paths, check_source, main

CHECKER = Path(check_imports.__file__).resolve()

QT_MODULES = ["PySide6", "PySide2", "PyQt5", "PyQt6", "OpenGL", "pyqtgraph", "tkinter", "vispy", "glfw", "pygame"]
ORACLE_MODULES = ["hppfcl", "fcl", "pycollision"]


def write(root: Path, relative: str, source: str) -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(source, encoding="utf-8")
    return path


def violations_for(tmp_path: Path, relative: str, source: str) -> list[Violation]:
    write(tmp_path, relative, source)
    return check_paths([tmp_path], tmp_path)


ENGINE_MODULES = ("mechsim._core_ext.engine", "mechsim._core.engine", "mechsim.physics.native._engine")


def test_rule_table_covers_every_boundary() -> None:
    covered = {module for rule in RULES for module in rule.modules}
    assert covered == {"mujoco", "mechsim.viewer", "tests", *QT_MODULES, *ORACLE_MODULES, *ENGINE_MODULES}


# ===== mujoco =========================================================================================================


@pytest.mark.parametrize(
    "location",
    ["mechsim/physics/mujoco/adapter.py", "mechsim/physics/mujoco/__init__.py", "mechsim/physics/mujoco/sub/deep.py"],
)
def test_mujoco_allowed_in_adapter(tmp_path: Path, location: str) -> None:
    assert violations_for(tmp_path, location, "import mujoco\n") == []


@pytest.mark.parametrize(
    "location",
    [
        "mechsim/physics/native/adapter.py",
        "mechsim/physics/__init__.py",
        "mechsim/physics/mujoco_extra/x.py",
        "mechsim/builder/build.py",
        "mechsim/viewer/replay.py",
        "tests/unit/test_x.py",
        "tools/script.py",
        "top_level.py",
        "physics/mujoco/x.py",
    ],
)
def test_mujoco_forbidden_outside_adapter(tmp_path: Path, location: str) -> None:
    result = violations_for(tmp_path, location, "import mujoco\n")
    assert len(result) == 1
    assert result[0].path == location
    assert result[0].line == 1
    assert "'mujoco'" in result[0].message
    assert "mechsim/physics/mujoco/" in result[0].message


# ===== Qt and display =================================================================================================


@pytest.mark.parametrize("module", QT_MODULES)
def test_display_modules_allowed_in_viewer(tmp_path: Path, module: str) -> None:
    assert violations_for(tmp_path, "mechsim/viewer/window.py", f"import {module}\n") == []


@pytest.mark.parametrize("module", QT_MODULES)
@pytest.mark.parametrize(
    "location", ["mechsim/facade/service.py", "mechsim/viewerx/w.py", "tests/unit/test_w.py", "tools/t.py"]
)
def test_display_modules_forbidden_outside_viewer(tmp_path: Path, module: str, location: str) -> None:
    result = violations_for(tmp_path, location, f"import {module}\n")
    assert len(result) == 1
    assert f"'{module}'" in result[0].message
    assert "invariant 9" in result[0].message


# ===== collision oracles ==============================================================================================


@pytest.mark.parametrize("module", ORACLE_MODULES)
def test_oracles_allowed_in_tests(tmp_path: Path, module: str) -> None:
    assert violations_for(tmp_path, "tests/reference/test_collision.py", f"import {module}\n") == []


@pytest.mark.parametrize("module", ORACLE_MODULES)
@pytest.mark.parametrize(
    "location", ["mechsim/physics/native/collide.py", "tests_extra/x.py", "tools/t.py", "unit/tests/x.py"]
)
def test_oracles_forbidden_outside_tests(tmp_path: Path, module: str, location: str) -> None:
    result = violations_for(tmp_path, location, f"import {module}\n")
    assert len(result) == 1
    assert f"'{module}'" in result[0].message
    assert "tests/" in result[0].message


# ===== mechsim.viewer =================================================================================================


@pytest.mark.parametrize(
    "location", ["mechsim/viewer/timeline.py", "mechsim/facade/service.py", "tests/unit/test_viewer.py"]
)
@pytest.mark.parametrize(
    "source",
    ["import mechsim.viewer\n", "from mechsim import viewer\n", "from mechsim.viewer import x\n"],
)
def test_viewer_import_allowed(tmp_path: Path, location: str, source: str) -> None:
    assert violations_for(tmp_path, location, source) == []


@pytest.mark.parametrize("location", ["mechsim/cli.py", "mechsim/__init__.py", "tools/t.py", "web/x.py"])
@pytest.mark.parametrize(
    "source",
    [
        "import mechsim.viewer\n",
        "import mechsim.viewer.timeline\n",
        "import mechsim.viewer as v\n",
        "from mechsim import viewer\n",
        "from mechsim import library, viewer\n",
        "from mechsim.viewer import timeline\n",
        "from mechsim.viewer.timeline import Timeline\n",
    ],
)
def test_viewer_import_forbidden_elsewhere(tmp_path: Path, location: str, source: str) -> None:
    result = violations_for(tmp_path, location, source)
    assert len(result) == 1
    assert "'mechsim.viewer" in result[0].message


@pytest.mark.parametrize(
    "source",
    ["import mechsim\n", "from mechsim import library\n", "import mechsim.viewers\n", "from mechsim import viewers\n"],
)
def test_other_mechsim_imports_allowed(tmp_path: Path, source: str) -> None:
    assert violations_for(tmp_path, "mechsim/facade/service.py", source) == []


# ===== tests package ==================================================================================================


@pytest.mark.parametrize(
    "source",
    ["import tests\n", "import tests.unit\n", "from tests import conftest\n", "from tests.unit.helpers import f\n"],
)
def test_mechsim_must_not_import_tests(tmp_path: Path, source: str) -> None:
    result = violations_for(tmp_path, "mechsim/scoring/report.py", source)
    assert len(result) == 1
    assert "'tests" in result[0].message
    assert "forbidden under mechsim/" in result[0].message


@pytest.mark.parametrize("location", ["tests/unit/test_a.py", "tools/t.py", "conftest.py"])
def test_tests_importable_outside_mechsim(tmp_path: Path, location: str) -> None:
    assert violations_for(tmp_path, location, "from tests.unit import helpers\n") == []


def test_tests_prefix_module_is_not_tests(tmp_path: Path) -> None:
    assert violations_for(tmp_path, "mechsim/a.py", "import testsuite\nimport tests_helpers\n") == []


# ===== import forms ===================================================================================================


@pytest.mark.parametrize(
    "source",
    [
        "import mujoco\n",
        "import mujoco.viewer\n",
        "import mujoco as mj\n",
        "import mujoco.viewer as mv\n",
        "import os, mujoco\n",
        "from mujoco import MjModel\n",
        "from mujoco import MjModel as M, MjData\n",
        "from mujoco import (\n    MjModel,\n    MjData,\n)\n",
        "from mujoco import *\n",
        "from mujoco.viewer import launch\n",
        "def f() -> None:\n    import mujoco\n",
        "def f() -> None:\n    def g() -> None:\n        from mujoco.viewer import launch\n",
        "class C:\n    def m(self) -> None:\n        import mujoco\n",
        "class C:\n    import mujoco\n",
        "if True:\n    import mujoco\n",
        "try:\n    import mujoco\nexcept ImportError:\n    pass\n",
        "with open('x') as f:\n    import mujoco\n",
        "async def f() -> None:\n    import mujoco\n",
        "lambda: __import__('mujoco')\n",
        "__import__('mujoco')\n",
        "import importlib\nimportlib.import_module('mujoco')\n",
        "from importlib import import_module\nimport_module('mujoco.viewer')\n",
    ],
)
def test_every_import_form_is_detected(tmp_path: Path, source: str) -> None:
    result = violations_for(tmp_path, "mechsim/builder/b.py", source)
    assert len(result) == 1
    assert "'mujoco" in result[0].message


@pytest.mark.parametrize(
    "source",
    [
        "from . import mujoco\n",
        "from .mujoco import adapter\n",
        "from .. import viewer\n",
        "from ..viewer import timeline\n",
        "from ...tests import helpers\n",
        "from . import *\n",
    ],
)
def test_relative_imports_are_never_violations(tmp_path: Path, source: str) -> None:
    assert violations_for(tmp_path, "mechsim/physics/native/adapter.py", source) == []


@pytest.mark.parametrize(
    "source",
    [
        "import mujoco_tools\n",
        "import mujocox.core\n",
        "from fclx import y\n",
        "import OpenGLContext\n",
        "import requests\n",
        "name = 'mujoco'\n__import__(name)\n",
        "import importlib\nimportlib.import_module('.mujoco', 'mechsim.physics')\n",
        "__import__()\n",
        "import importlib\nimportlib.import_module(b'mujoco')\n",
        "def mujoco() -> None:\n    pass\n",
        "x = 'import mujoco'\n",
        "# import mujoco\n",
        "getattr(object, 'x')('mujoco')\n",
    ],
)
def test_lookalikes_are_not_violations(tmp_path: Path, source: str) -> None:
    assert violations_for(tmp_path, "mechsim/builder/b.py", source) == []


def test_multi_name_from_import_reports_once(tmp_path: Path) -> None:
    result = violations_for(tmp_path, "mechsim/builder/b.py", "from mujoco import viewer, MjModel\n")
    assert len(result) == 1


def test_multiple_violations_sorted_with_lines(tmp_path: Path) -> None:
    source = "import os\nimport PySide6\n\ndef f() -> None:\n    import mujoco\nimport fcl\n"
    result = violations_for(tmp_path, "mechsim/library/x.py", source)
    assert [v.line for v in result] == [2, 5, 6]


def test_one_import_can_break_two_rules_from_two_statements(tmp_path: Path) -> None:
    result = violations_for(tmp_path, "mechsim/library/x.py", "import mujoco, PySide6\n")
    assert len(result) == 2
    assert all(v.line == 1 for v in result)


# ===== unparseable and empty files ====================================================================================


def test_syntax_error_is_a_violation(tmp_path: Path) -> None:
    result = violations_for(tmp_path, "mechsim/a.py", "import os\ndef broken(:\n")
    assert len(result) == 1
    assert result[0].line == 2
    assert "cannot parse" in result[0].message


def test_syntax_error_does_not_hide_other_files(tmp_path: Path) -> None:
    write(tmp_path, "mechsim/a.py", "def broken(:\n")
    write(tmp_path, "mechsim/b.py", "import mujoco\n")
    result = check_paths([tmp_path], tmp_path)
    assert [v.path for v in result] == ["mechsim/a.py", "mechsim/b.py"]


def test_null_byte_is_a_violation(tmp_path: Path) -> None:
    path = tmp_path / "mechsim" / "a.py"
    path.parent.mkdir(parents=True)
    path.write_bytes(b"import os\x00\n")
    result = check_paths([tmp_path], tmp_path)
    assert len(result) == 1
    assert "cannot parse" in result[0].message


def test_invalid_encoding_is_a_violation(tmp_path: Path) -> None:
    path = tmp_path / "mechsim" / "a.py"
    path.parent.mkdir(parents=True)
    path.write_bytes(b"x = '\xff\xfe'\n")
    result = check_paths([tmp_path], tmp_path)
    assert len(result) == 1
    assert "cannot parse" in result[0].message


def test_encoding_declaration_is_honoured(tmp_path: Path) -> None:
    path = tmp_path / "mechsim" / "a.py"
    path.parent.mkdir(parents=True)
    path.write_bytes(b"# -*- coding: latin-1 -*-\nx = '\xe9'\nimport mujoco\n")
    result = check_paths([tmp_path], tmp_path)
    assert [v.line for v in result] == [3]


@pytest.mark.parametrize(
    "source", ["", "\n", "# only a comment\n", '"""Docstring."""\n', "x = 1\n", "from __future__ import annotations\n"]
)
def test_files_without_boundary_imports_pass(tmp_path: Path, source: str) -> None:
    assert violations_for(tmp_path, "mechsim/a.py", source) == []


def test_check_source_with_custom_rules() -> None:
    rule = Rule(modules=("numpy",), allowed_under=("ok/",), forbidden_under=(), reason="custom")
    assert check_source("import numpy\n", "ok/a.py", [rule]) == []
    assert len(check_source("import numpy\n", "bad/a.py", [rule])) == 1


def test_forbidden_under_beats_allowed_under() -> None:
    rule = Rule(modules=("numpy",), allowed_under=("pkg/",), forbidden_under=("pkg/secret/",), reason="custom")
    assert check_source("import numpy\n", "pkg/a.py", [rule]) == []
    assert len(check_source("import numpy\n", "pkg/secret/a.py", [rule])) == 1


# ===== file discovery =================================================================================================


@pytest.mark.parametrize("directory", [".venv", ".git", ".hidden", "build", "node_modules", "__pycache__", "a/build"])
def test_skipped_directories(tmp_path: Path, directory: str) -> None:
    write(tmp_path, f"{directory}/x.py", "import mujoco\n")
    assert check_paths([tmp_path], tmp_path) == []


def test_non_python_files_are_ignored(tmp_path: Path) -> None:
    write(tmp_path, "mechsim/notes.md", "import mujoco\n")
    write(tmp_path, "mechsim/script", "import mujoco\n")
    write(tmp_path, "mechsim/stub.pyi", "import mujoco\n")
    assert check_paths([tmp_path], tmp_path) == []
    assert check_paths([tmp_path / "mechsim" / "notes.md"], tmp_path) == []


def test_explicit_file_path(tmp_path: Path) -> None:
    path = write(tmp_path, "mechsim/a.py", "import mujoco\n")
    result = check_paths([path], tmp_path)
    assert [v.path for v in result] == ["mechsim/a.py"]


def test_explicit_subdirectory_keeps_root_relative_locations(tmp_path: Path) -> None:
    write(tmp_path, "mechsim/physics/mujoco/a.py", "import mujoco\n")
    write(tmp_path, "mechsim/physics/native/b.py", "import mujoco\n")
    result = check_paths([tmp_path / "mechsim" / "physics"], tmp_path)
    assert [v.path for v in result] == ["mechsim/physics/native/b.py"]


def test_missing_path_is_a_violation(tmp_path: Path) -> None:
    result = check_paths([tmp_path / "missing.py"], tmp_path)
    assert len(result) == 1
    assert result[0].message == "no such file or directory"


def test_file_outside_root_matches_no_allowed_location(tmp_path: Path) -> None:
    outside = write(tmp_path, "outside/mechsim/physics/mujoco/a.py", "import mujoco\n")
    root = tmp_path / "repo"
    root.mkdir()
    result = check_paths([outside], root)
    assert len(result) == 1
    assert result[0].path == outside.resolve().as_posix()


def test_empty_directory(tmp_path: Path) -> None:
    assert check_paths([tmp_path], tmp_path) == []


# ===== command line ===================================================================================================


def test_violation_format() -> None:
    assert str(Violation("mechsim/a.py", 3, "msg")) == "mechsim/a.py:3: msg"


def test_main_returns_zero_when_clean(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    write(tmp_path, "mechsim/physics/mujoco/a.py", "import mujoco\n")
    assert main(["--root", str(tmp_path)]) == 0
    assert capsys.readouterr().out == ""


def test_main_returns_one_and_prints_violations(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    write(tmp_path, "mechsim/a.py", "\nimport mujoco\n")
    assert main(["--root", str(tmp_path), str(tmp_path)]) == 1
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 1
    assert lines[0].startswith("mechsim/a.py:2: import of 'mujoco'")


def test_script_entry_point(tmp_path: Path) -> None:
    write(tmp_path, "mechsim/a.py", "import PyQt6\n")
    completed = subprocess.run(
        [sys.executable, str(CHECKER), "--root", str(tmp_path)], capture_output=True, text=True, check=False
    )
    assert completed.returncode == 1
    assert completed.stdout.startswith("mechsim/a.py:1: import of 'PyQt6'")
    (tmp_path / "mechsim" / "a.py").write_text("import os\n", encoding="utf-8")
    completed = subprocess.run(
        [sys.executable, str(CHECKER), "--root", str(tmp_path)], capture_output=True, text=True, check=False
    )
    assert completed.returncode == 0
    assert completed.stdout == ""
