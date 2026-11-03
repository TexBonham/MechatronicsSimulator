"""Reference tier stubs for the physics adapter spec (PHY), strict xfail until converted."""

import pytest


@pytest.mark.req("PHY-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_021() -> None:
    """Two adapters run on the same neutral model, inputs and seed shall agree on body poses, velocities and forces
    within the differential tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_022() -> None:
    """A new adapter shall pass the analytical reference suite before it is accepted."""
    raise NotImplementedError


@pytest.mark.req("PHY-028")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_028() -> None:
    """A rigid pendulum run on an adapter shall match the closed-form small-angle period."""
    raise NotImplementedError


@pytest.mark.req("PHY-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_029() -> None:
    """A body resting on the floor under gravity shall stay at rest on an adapter, with penetration and drift within
    tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-030")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_030() -> None:
    """A body on a prismatic joint with Coulomb friction shall decelerate at the closed-form rate on an adapter."""
    raise NotImplementedError
