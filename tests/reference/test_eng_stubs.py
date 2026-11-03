"""ENG stubs, reference tier (T3); Catch2 tests replace them with the C++ code."""

import pytest


@pytest.mark.req("ENG-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_019() -> None:
    """The engine's collision detection shall agree with the reference collision library on contact existence, normal
    and penetration depth over a shared case set.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_021() -> None:
    """The engine shall bring a body dropped onto a plane to rest with steady penetration within tolerance, and shall
    stop a body driven into a fixed hard stop.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-042")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_042() -> None:
    """A lumped pinned-pinned column under axial load shall lose stability at the Euler load pi^2*E*I/L^2."""
    raise NotImplementedError
