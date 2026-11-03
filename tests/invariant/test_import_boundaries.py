"""Invariant tier: import boundaries hold across the whole repository.

INV-005: engines and emulators stay behind adapters. INV-009: the core is headless.
"""

from __future__ import annotations

import pytest

from check_imports import check_paths, default_root


@pytest.mark.req("INV-005")
@pytest.mark.req("INV-009")
def test_repository_has_no_import_boundary_violations() -> None:
    root = default_root()
    violations = check_paths([root], root)
    assert [str(violation) for violation in violations] == []
