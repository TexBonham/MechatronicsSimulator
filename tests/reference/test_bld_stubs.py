"""Reference tier stubs for the builder spec (BLD), strict xfail until converted."""

import pytest


@pytest.mark.req("BLD-010")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_010() -> None:
    """The builder shall compute mass, center of mass and inertia from a body's geometry and its material's density,
    matching closed-form values for primitive solids.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_015() -> None:
    """A lumped cantilever beam built from a known section and material shall match the closed-form static tip
    deflection.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-016")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_016() -> None:
    """A lumped cantilever beam built from a known section and material shall match the closed-form first natural
    frequency.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_017() -> None:
    """A lumped simply supported beam shall match the closed-form midspan deflection and first natural frequency."""
    raise NotImplementedError


@pytest.mark.req("BLD-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_019() -> None:
    """The builder shall calibrate each flexible lumped beam so that it reproduces the deflection and first natural
    frequency recorded for that member in the design file."""
    raise NotImplementedError
