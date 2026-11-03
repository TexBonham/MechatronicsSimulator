"""Reference tier stubs for the design spec (DES), strict xfail until converted."""

import pytest


@pytest.mark.req("DES-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_des_018() -> None:
    """STEP import shall compute volume, center of mass and inertia of each body from its geometry and its material's
    density.
    """
    raise NotImplementedError
