"""Invariant tier stubs for the physics adapter spec (PHY), strict xfail until converted."""

import pytest


@pytest.mark.req("PHY-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_010() -> None:
    """The same neutral model, inputs and seed run twice on the same adapter shall give identical outputs at every
    step.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-011")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_011() -> None:
    """No MuJoCo or other engine type shall be imported or referenced outside `mechsim/physics/<name>/`, as enforced by
    the import-boundary checker.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_026() -> None:
    """After retirement, no runtime code path shall import MuJoCo."""
    raise NotImplementedError
