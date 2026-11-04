"""Invariant tier stubs for the physics adapter spec (PHY), strict xfail until converted."""

import re
from pathlib import Path

import pytest


@pytest.mark.req("PHY-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_010() -> None:
    """The same neutral model, inputs and seed run twice on the same adapter shall give identical outputs at every
    step.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_026() -> None:
    """After retirement, no runtime code path shall import MuJoCo."""
    raise NotImplementedError


ROOT = Path(__file__).resolve().parents[2]
_MUJOCO_INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]mujoco/', re.MULTILINE)


@pytest.mark.req("PHY-011")
def test_phy_011_mujoco_headers_only_in_the_cpp_adapter() -> None:
    """C++ side of PHY-011 (ADR 0014); the Python side is the import-boundary checker's test."""
    allowed = ROOT / "core" / "physics" / "mujoco"
    offenders = []
    for path in sorted((ROOT / "core").rglob("*")):
        if path.suffix not in {".c", ".cc", ".cpp", ".h", ".hpp"} or allowed in path.parents:
            continue
        if _MUJOCO_INCLUDE.search(path.read_text(encoding="utf-8", errors="replace")):
            offenders.append(path.relative_to(ROOT).as_posix())
    assert offenders == []
    assert _MUJOCO_INCLUDE.search((allowed / "src" / "mujoco_adapter.cpp").read_text(encoding="utf-8"))
