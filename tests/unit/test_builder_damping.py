"""Structural damping as design data (DES-047, BLD-040, ruling 78): a flexible member's modal damping ratio of its
first bending mode becomes stiffness-proportional node damping c = 2 zeta k / omega_1; an undamped member is always
noted. The decrement oracles are in `tests/reference/test_member_damping.py`."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from mechsim.builder import structure
from mechsim.design.schema import FlexibleMember, OpenValue
from tests.unit.test_builder_structure import CALIBRATION, EI, RHO_A, _bar, _build
from tests.unit.test_builder_support import gantry

UNDAMPED = (
    "flexible body 'bar': undamped (damping_ratio absent); its ringing is removed only by the integration scheme's "
    "own decay, which depends on the step, so its dynamic response does not converge as the step is refined (DES-047, "
    "BLD-040, ruling 78)"
)


@pytest.mark.req("DES-047")
@pytest.mark.parametrize(("written", "value"), [("0.01", 0.01), ("5 percent", 0.05), ("0", 0.0), ("0.999", 0.999)])
def test_damping_ratio_is_a_dimensionless_field(written: str, value: float) -> None:
    member = FlexibleMember.model_validate({"supports": ["clamped", "free"], "damping_ratio": written})
    assert member.damping_ratio == pytest.approx(value, rel=1e-15)


@pytest.mark.req("DES-047")
def test_damping_ratio_defaults_to_absent_and_may_be_open() -> None:
    assert FlexibleMember.model_validate({"supports": ["clamped", "free"]}).damping_ratio is None
    member = FlexibleMember.model_validate({"supports": ["clamped", "free"], "damping_ratio": "${rail_damping}"})
    assert member.damping_ratio == OpenValue("rail_damping", "dimensionless")


@pytest.mark.req("DES-047")
@pytest.mark.parametrize("written", ["-0.01", "1", "1.5", "0.01 m"])
def test_damping_ratio_outside_zero_to_one_or_with_a_unit_is_refused(written: str) -> None:
    with pytest.raises(ValidationError):
        FlexibleMember.model_validate({"supports": ["clamped", "free"], "damping_ratio": written})


@pytest.mark.req("BLD-040")
@pytest.mark.parametrize("supports", [["clamped", "free"], ["pinned", "roller"], ["clamped", "clamped"]])
def test_node_damping_is_two_zeta_k_over_the_chains_first_mode(supports: list[str]) -> None:
    result = _build(_bar(supports, damping_ratio="0.02"))
    assert result.physics is not None
    member = result.physics.lumped_members[0]
    segments = member.segments
    omega = structure.loaded_first_frequency(supports[0], supports[1], segments, 1.0, EI, RHO_A)
    stiffness = EI * segments
    assert member.node_damping == pytest.approx(2.0 * 0.02 * stiffness / omega, rel=1e-12)
    record = result.lumped[0]
    assert (record.damping_ratio, record.node_damping) == (0.02, member.node_damping)
    assert record.first_frequency == pytest.approx(omega, rel=1e-12)
    assert UNDAMPED not in result.notes
    line = next(note for note in result.notes if "stiffness-proportional" in note)
    assert f"time constant 1 / (zeta omega_1) = {1.0 / (0.02 * omega):.6g} s" in line
    assert "carried mass" not in line


@pytest.mark.req("BLD-040")
def test_first_modes_of_the_three_chains_lie_near_their_closed_forms() -> None:
    """The conversion's omega_1 is the chain's; at N = 8 (16 clamped-clamped) it lies within the 2 percent of BLD-017
    of beta^2 sqrt(E I / (rho A L^4)), beta^2 = 3.516, pi^2 and 22.373."""
    for root, end, segments, beta in (
        ("clamped", "free", 8, 3.516015),
        ("pinned", "roller", 8, math.pi**2),
        ("clamped", "clamped", 16, 22.37329),
    ):
        omega = structure.loaded_first_frequency(root, end, segments, 1.0, EI, RHO_A)
        assert omega == pytest.approx(beta * math.sqrt(EI / RHO_A), rel=0.02)


@pytest.mark.req("BLD-040", "BLD-019")
def test_a_calibrated_member_is_damped_on_its_recorded_frequency() -> None:
    """Calibration makes the chain's first mode the recorded 8 Hz exactly, so omega_1 = 2 pi 8 rad/s."""
    result = _build(_bar(["clamped", "free"], calibration=CALIBRATION, damping_ratio="0.01"))
    record = result.lumped[0]
    assert record.first_frequency == pytest.approx(2.0 * math.pi * 8.0, rel=1e-9)
    assert record.node_damping == pytest.approx(2.0 * 0.01 * record.node_stiffness / (2.0 * math.pi * 8.0), rel=1e-9)


@pytest.mark.req("BLD-040")
def test_an_undamped_member_gets_its_line() -> None:
    result = _build(_bar(["clamped", "free"]))
    assert result.physics is not None
    assert result.physics.lumped_members[0].node_damping == 0.0
    assert UNDAMPED in result.notes
    assert result.lumped[0].damping_ratio is None


@pytest.mark.req("BLD-040")
def test_a_zero_ratio_is_undamped_and_says_so() -> None:
    result = _build(_bar(["clamped", "free"], damping_ratio="0"))
    assert result.physics is not None
    assert result.physics.lumped_members[0].node_damping == 0.0
    assert UNDAMPED.replace("absent", "0") in result.notes


@pytest.mark.req("BLD-040", "BLD-033")
def test_a_rigid_build_has_no_damping_lines() -> None:
    result = _build(_bar(["clamped", "free"]), rigid_members=True)
    assert not any("damping" in note for note in result.notes)


@pytest.mark.req("BLD-040")
def test_a_carried_mass_lowers_the_first_mode_most_at_the_cantilever_tip() -> None:
    lowest, at = structure.lowest_carried_frequency("clamped", "free", 8, 1.0, EI, RHO_A, 0.5)
    assert at == 1.0
    assert lowest == structure.loaded_first_frequency("clamped", "free", 8, 1.0, EI, RHO_A, 0.5, 1.0)
    assert lowest < structure.loaded_first_frequency("clamped", "free", 8, 1.0, EI, RHO_A)
    lowest, at = structure.lowest_carried_frequency("pinned", "roller", 8, 1.0, EI, RHO_A, 0.5)
    # The pinned chain is not mirror-symmetric (a pin at x = 0, springs at (j + 1/2) l, a half span at the roller), so
    # its mode peaks near, not at, midspan: within one segment of it.
    assert abs(at - 0.5) <= 1.0 / 8
    assert lowest <= structure.loaded_first_frequency("pinned", "roller", 8, 1.0, EI, RHO_A, 0.5, 0.5)
    assert structure.loaded_first_frequency("pinned", "roller", 8, 1.0, EI, RHO_A, 0.5, 0.0) == pytest.approx(
        structure.loaded_first_frequency("pinned", "roller", 8, 1.0, EI, RHO_A), rel=1e-12
    )


@pytest.mark.req("DES-047")
def test_the_gantry_rails_state_the_placeholder_ratio_fixed_in_advance() -> None:
    """Ruling 78: 0.01 on both rails (loss factor 0.02, the low end for printed PETG), owed by the gantry team."""
    design = gantry()
    ratios = {body.name: body.flexible.damping_ratio for body in design.bodies if body.flexible is not None}
    assert ratios == {"rail_x": 0.01, "rail_y": 0.01}
