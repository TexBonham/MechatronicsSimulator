"""Invariant tier: import boundaries hold across the whole repository.

INV-005: engines and emulators stay behind adapters. INV-009: the core is headless.
"""

from __future__ import annotations

import pytest

from check_imports import check_paths, check_source, default_root


@pytest.mark.req("INV-005")
@pytest.mark.req("INV-009")
def test_repository_has_no_import_boundary_violations() -> None:
    root = default_root()
    violations = check_paths([root], root)
    assert [str(violation) for violation in violations] == []


ENGINE_IMPORTS = (
    "import mechsim._core_ext.engine",
    "from mechsim._core_ext import engine",
    "from mechsim._core import engine",
    "from mechsim.physics.native._engine import MultibodyProvider",
    "import importlib\nimportlib.import_module('mechsim._core_ext.engine')",
)


@pytest.mark.req("INV-005")
@pytest.mark.parametrize("source", ENGINE_IMPORTS)
@pytest.mark.parametrize("location", ["mechsim/physics/native/adapter.py", "tests/unit/test_engine.py"])
def test_engine_binding_allowed_behind_native_adapter(source: str, location: str) -> None:
    assert check_source(source, location) == []


@pytest.mark.req("INV-005")
@pytest.mark.parametrize("source", ENGINE_IMPORTS)
@pytest.mark.parametrize(
    "location", ["mechsim/builder/build.py", "mechsim/physics/mujoco/adapter.py", "mechsim/physics/neutral.py"]
)
def test_engine_binding_rejected_elsewhere(source: str, location: str) -> None:
    violations = check_source(source, location)
    assert len(violations) == 1
    assert "invariant 5" in violations[0].message
