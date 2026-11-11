"""Unit tier tests and stubs for the physics adapter spec (PHY); stubs are strict xfail until converted."""

import math
from pathlib import Path
from typing import Any

import pytest

from mechsim import _core as core


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
def test_phy_006() -> None:
    """The adapter interface shall accept injected physical faults: transmission slip and payload change.

    Through the binding on the MuJoCo adapter, with the committed translation fixtures. Slip: the lead screw
    (c = 0.008 / 2 pi) under a shaft drive whose rest advances 0.01 rad per step for 10 steps with s = 0.5 loses
    s * 9 * 0.01 = 0.045 rad (the first rest only starts tracking). Payload: 2 kg welded to a fixed hook carries a
    force near m g; doubled, it carries near 2 m g; released, it falls and reports no carrier; a wrong carrier or a
    fraction outside [0, 1] is an AdapterError. The exact closed-form checks are the C++ reference tests [PHY-006].
    """
    fixtures = Path(__file__).resolve().parents[2] / "core" / "physics" / "mujoco" / "tests" / "fixtures"

    def load(name: str) -> Any:
        adapter = core.make_physics_adapter("mujoco")
        adapter.load(core.PreparedModel("", (fixtures / name).read_text(encoding="utf-8"), "0.1.0"), core.Rate(1000))
        return adapter

    screw = load("lead_screw.xml")
    screw.set_transmission_slip(0, 0.5)
    for k in range(10):
        inputs = core.PhysicsInputs()
        inputs.joint_drives = [core.JointDrive(0, 0.01 * k, 20.0, 0.01, True)]
        screw.set_inputs(inputs)
        screw.step()
    state = screw.outputs().transmissions[0]
    assert state.slipping
    assert state.slip == pytest.approx(0.045, rel=1e-12)
    for fraction in (-0.1, 1.5, math.nan):
        with pytest.raises(core.AdapterError):
            screw.set_transmission_slip(0, fraction)
    with pytest.raises(core.AdapterError):
        screw.set_transmission_slip(1, 0.1)

    payload = load("payload.xml")
    weight = 2.0 * 9.81
    for _ in range(200):
        payload.step()
    held = payload.outputs().payloads[0]
    assert (held.body, held.carrier, held.attached) == (1, 0, True)
    assert held.force[2] == pytest.approx(weight, rel=1e-6)
    payload.set_payload_mass(1, 4.0)
    for _ in range(200):
        payload.step()
    assert payload.outputs().payloads[0].force[2] == pytest.approx(2.0 * weight, rel=1e-6)
    with pytest.raises(core.AdapterError):
        payload.set_payload_attachment(1, 1)
    height = payload.outputs().bodies[1].position[2]
    payload.release_load(1)
    for _ in range(10):
        payload.step()
    released = payload.outputs().payloads[0]
    assert not released.attached
    assert released.force == [0.0, 0.0, 0.0]
    assert payload.outputs().bodies[1].position[2] < height
    payload.set_payload_attachment(1, 0)
    payload.step()
    assert payload.outputs().payloads[0].attached


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


@pytest.mark.req("PHY-018")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_018() -> None:
    """Every adapter shall map the neutral model's transmissions and lumped members onto its engine."""
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
