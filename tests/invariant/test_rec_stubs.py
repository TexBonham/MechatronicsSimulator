"""Strict-xfail stubs for the REC spec (invariant tier)."""

import pytest


@pytest.mark.req("REC-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_021() -> None:
    """No run archive shall be tracked in git."""
    raise NotImplementedError


@pytest.mark.req("REC-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_rec_022() -> None:
    """Two archives from the same design, test case, seed and controller mode shall be identical in every
    recorded series and attribute.
    """
    raise NotImplementedError
