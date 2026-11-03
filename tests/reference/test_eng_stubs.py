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


@pytest.mark.req("ENG-036")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_036() -> None:
    """A rigid-link pendulum's small-amplitude period shall match the compound pendulum closed form
    2*pi*sqrt(I/(m*g*d)).
    """
    raise NotImplementedError


@pytest.mark.req("ENG-037")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_037() -> None:
    """A pendulum with viscous joint damping shall decay with the logarithmic decrement of the linear damped
    oscillator closed form.
    """
    raise NotImplementedError


@pytest.mark.req("ENG-038")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_038() -> None:
    """A lumped cantilever under a static tip load shall deflect by P*L^3/(3*E*I) at the tip."""
    raise NotImplementedError


@pytest.mark.req("ENG-039")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_039() -> None:
    """A lumped cantilever's first natural frequency shall match (1.875^2/(2*pi))*sqrt(E*I/(rho*A*L^4))."""
    raise NotImplementedError


@pytest.mark.req("ENG-040")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_040() -> None:
    """A lumped simply supported beam under a static midspan load shall deflect by P*L^3/(48*E*I) at midspan."""
    raise NotImplementedError


@pytest.mark.req("ENG-041")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_041() -> None:
    """A lumped simply supported beam's first natural frequency shall match (pi/(2*L^2))*sqrt(E*I/(rho*A))."""
    raise NotImplementedError


@pytest.mark.req("ENG-042")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_042() -> None:
    """A lumped pinned-pinned column under axial load shall lose stability at the Euler load pi^2*E*I/L^2."""
    raise NotImplementedError
