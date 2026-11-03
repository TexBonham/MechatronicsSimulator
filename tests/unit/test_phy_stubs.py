"""Unit tier stubs for the physics adapter spec (PHY), strict xfail until converted."""

import pytest


@pytest.mark.req("PHY-003")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_003() -> None:
    """The adapter interface shall report body poses, velocities, joint forces and contact forces after every step."""
    raise NotImplementedError


@pytest.mark.req("PHY-004")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_004() -> None:
    """The adapter interface shall report rope tension after every step for every rope in the model."""
    raise NotImplementedError


@pytest.mark.req("PHY-006")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_006() -> None:
    """The adapter interface shall accept injected physical faults: transmission slip and payload change."""
    raise NotImplementedError


@pytest.mark.req("PHY-007")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_007() -> None:
    """Every adapter that carries milestone 3 or later shall publish live member end-loads every tick."""
    raise NotImplementedError


@pytest.mark.req("PHY-009")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_009() -> None:
    """An adapter shall report constraint drift beyond its threshold and solver failure as run errors that end the run
    with a report, never as returned numbers.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-012")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_012() -> None:
    """The only model type that crosses into an adapter shall be the engine-neutral physics model, and the only values
    that cross out shall be neutral types.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-013")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_013() -> None:
    """The MuJoCo adapter shall translate the neutral model into MJCF."""
    raise NotImplementedError


@pytest.mark.req("PHY-014")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_014() -> None:
    """The MuJoCo adapter shall run with the implicitfast integrator at a 1 ms step."""
    raise NotImplementedError


@pytest.mark.req("PHY-017")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_017() -> None:
    """The MuJoCo adapter shall map only elements of the neutral model and shall add no element type the neutral model
    lacks.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_018() -> None:
    """Every adapter shall map the neutral model's transmissions and lumped members onto its engine."""
    raise NotImplementedError


@pytest.mark.req("PHY-019")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_019() -> None:
    """An adapter shall declare the neutral elements it can map and shall refuse to load a model containing any other
    element, ending with a report.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-020")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_020() -> None:
    """The native adapter shall bind the own engine through the core runtime's single Python binding."""
    raise NotImplementedError


@pytest.mark.req("PHY-025")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_025() -> None:
    """The comparison recordings used for retirement shall be kept as fixed baselines, stored as YAML scoring summaries
    in git.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-027")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_027() -> None:
    """Every adapter shall apply the design's gravity, joint friction and joint limits."""
    raise NotImplementedError


@pytest.mark.req("PHY-031")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_031() -> None:
    """Integrator choice and step size shall be per-adapter configuration, and no adapter shall change another
    adapter's configuration.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-032")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_032() -> None:
    """An adapter shall advance simulated time only on a step call and shall not read wall-clock time."""
    raise NotImplementedError
