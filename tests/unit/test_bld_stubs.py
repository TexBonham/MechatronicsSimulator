"""Unit tier stubs for the builder spec (BLD), strict xfail until converted."""

import pytest


@pytest.mark.req("BLD-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_004() -> None:
    """A kinematic loop in the design shall be emitted as a constraint in the neutral model."""
    raise NotImplementedError
