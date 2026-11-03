"""Acceptance tier stubs for the physics adapter spec (PHY), strict xfail until converted."""

import pytest


@pytest.mark.req("PHY-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_015() -> None:
    """The MuJoCo adapter shall carry the milestone 1 acceptance cases, G1 to G4 and the pendulum."""
    raise NotImplementedError


@pytest.mark.req("PHY-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_016() -> None:
    """The MuJoCo adapter shall carry the milestone 2 acceptance cases, G5 to G11, G13 and G14."""
    raise NotImplementedError


@pytest.mark.req("PHY-023")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_023() -> None:
    """The MuJoCo retirement check shall pass only when the native adapter passes the whole analytical suite."""
    raise NotImplementedError


@pytest.mark.req("PHY-024")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_024() -> None:
    """The MuJoCo retirement check shall pass only when the native adapter matches the reference-design baselines within
    the differential tolerance.
    """
    raise NotImplementedError
