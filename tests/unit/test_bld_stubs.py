"""Unit tier stubs for the builder spec (BLD), strict xfail until converted."""

import pytest


@pytest.mark.req("BLD-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_004() -> None:
    """A kinematic loop in the design shall be emitted as a constraint in the neutral model."""
    raise NotImplementedError


@pytest.mark.req("BLD-008")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_008() -> None:
    """The builder shall emit a structural member list naming every structural member and mount of the design."""
    raise NotImplementedError


@pytest.mark.req("BLD-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_031() -> None:
    """Every fastener and mount shall be emitted as an individually addressable constraint, so that a modeled failure
    can disable it.
    """
    raise NotImplementedError


@pytest.mark.req("BLD-032")
@pytest.mark.xfail(strict=True, reason="stub")
def test_bld_032() -> None:
    """A rope shall be emitted as a lumped chain in the neutral form."""
    raise NotImplementedError
