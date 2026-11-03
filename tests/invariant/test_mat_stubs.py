"""Invariant tier stubs for the materials spec (MAT), strict xfail until converted."""

import pytest


@pytest.mark.req("MAT-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_mat_018() -> None:
    """Every shipped materials file shall be YAML and shall validate against its JSON Schema, which is generated from
    the pydantic v2 material model.
    """
    raise NotImplementedError
