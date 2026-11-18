"""ENG stubs, invariant tier (T3); Catch2 tests replace them with the C++ code."""

import re
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]
_COLLISION_LIBRARIES = re.compile(r"\b(fcl|hppfcl|coal|libccd|ccd|bullet|BulletCollision|ode|jolt)\b", re.IGNORECASE)


@pytest.mark.req("ENG-018")
def test_eng_018() -> None:
    """The engine shall not link the reference collision library; that library shall be used only inside the test
    suite.
    """
    # The engine's link line and includes: no collision library in any CMake target or C++ source under core/
    # (the Python side is held by tools/check_imports.py's reference-collision rule).
    offenders = []
    for path in sorted((_ROOT / "core").rglob("*")):
        if path.suffix not in {".cpp", ".hpp", ".h", ".txt", ".cmake"} or "build" in path.parts:
            continue
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
            is_link = "target_link_libraries" in line or "find_package" in line or "FetchContent_Declare" in line
            is_include = line.lstrip().startswith("#include")
            if (is_link or is_include) and _COLLISION_LIBRARIES.search(line):
                offenders.append(f"{path.relative_to(_ROOT)}:{number}: {line.strip()}")
    assert offenders == []
    assert (_ROOT / "core/engine/src/collision.cpp").is_file()
