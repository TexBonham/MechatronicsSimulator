"""Strict-xfail stubs for the FAC spec (invariant tier)."""

import pytest


@pytest.mark.req("FAC-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_fac_016() -> None:
    """Client code shall hold no domain logic: every value a client shows comes from the facade."""
    raise NotImplementedError
