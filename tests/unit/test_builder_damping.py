"""Structural damping as design data (DES-047, BLD-040, ruling 78): a flexible member's modal damping ratio of its
first bending mode becomes stiffness-proportional node damping c = 2 zeta k / omega_1; an undamped member is always
noted. The decrement oracles are in `tests/reference/test_member_damping.py`."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from mechsim.builder import BuildError, BuildResult, structure
from mechsim.design.schema import Design, FlexibleMember, OpenValue
from mechsim.library.registry import default_registry
from mechsim.physics.mujoco.prepare import prepare_model as prepare_bridge
from mechsim.physics.native.translate import _subtree_mass, translate
from mechsim.physics.neutral import PhysicsModel
from tests.reference.test_transmission_damping import belt_model
from tests.unit.test_builder_structure import CALIBRATION, EI, RHO_A, _bar, _bar_data, _build
from tests.unit.test_builder_support import filled_gantry, gantry, gantry_data

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


# ===== Transmissions (LIB-035, BLD-041, PHY-037) ========================


def _belt_build(ratio: str | None) -> BuildResult:
    data = gantry_data()
    for component in data["components"]:
        if component["type"] == "belt_pulley":
            overrides = {key: value for key, value in component.get("overrides", {}).items() if key != "damping_ratio"}
            if ratio is not None:
                overrides["damping_ratio"] = ratio
            component["overrides"] = overrides
    return _build(filled_gantry(data))


def _build_physics(result: BuildResult) -> PhysicsModel:
    assert result.physics is not None
    return result.physics


@pytest.mark.req("LIB-035")
@pytest.mark.parametrize("kind", ["belt_pulley", "lead_screw"])
def test_compliant_transmission_types_take_a_damping_ratio_defaulting_to_zero(kind: str) -> None:
    parameter = default_registry().component_type(kind).parameter("damping_ratio")
    assert parameter.default == 0.0


@pytest.mark.req("BLD-041", "LIB-035")
def test_a_stated_transmission_ratio_becomes_two_zeta_sqrt_k_m_on_the_driven_subtree() -> None:
    result = _build_physics(_belt_build("0.02"))
    assert len(result.transmissions) == 3
    for transmission in result.transmissions:
        driven = next(
            body for body in result.bodies if body.joint is not None and body.joint.name == transmission.driven_joint
        )
        expected = 2.0 * 0.02 * math.sqrt(float(transmission.stiffness) * _subtree_mass(result, driven.name))
        assert float(transmission.damping) == pytest.approx(expected, rel=1e-12)


@pytest.mark.req("BLD-041", "PHY-037")
def test_an_absent_transmission_ratio_stays_undamped() -> None:
    result = _build_physics(_belt_build(None))
    assert all(float(transmission.damping) == 0.0 for transmission in result.transmissions)


@pytest.mark.req("PHY-037")
def test_the_native_line_prints_zeta_d_and_the_time_constant_of_a_damped_belt() -> None:
    """The reference belt: m = 2 kg, k = 1e5 N/m; D = 2 (0.02) sqrt(k m), time constant 2 m / D."""
    damping = 2.0 * 0.02 * math.sqrt(1e5 * 2.0)
    translation = translate(belt_model(damping), 1e-3)
    line = next(item for item in translation.report if item.startswith("transmission 'belt': compliant"))
    assert f"damping {damping:.6g} at the output (ruling 78), zeta = D / (2 sqrt(k m)) = 0.02 on the driven" in line
    assert f"time constant 2 m / D = {4.0 / damping:.6g} s" in line
    assert translation.document["transmissions"][0]["damping"] == damping
    undamped = translate(belt_model(0.0), 1e-3)
    assert "no damping (ruling 10)" in next(
        item for item in undamped.report if item.startswith("transmission 'belt': compliant")
    )
    assert undamped.document["transmissions"][0]["damping"] == 0.0


@pytest.mark.req("PHY-037")
def test_the_bridge_keeps_its_rigid_belt_line_and_names_an_ignored_damping() -> None:
    damped = prepare_bridge(belt_model(17.0), None, 1e-3).report
    plain = prepare_bridge(belt_model(0.0), None, 1e-3).report
    assert any("treated as rigid (milestone 1); its damping " in line for line in damped)
    assert any(line.endswith("treated as rigid (milestone 1)") for line in plain)
    assert not any("its damping" in line for line in plain)


@pytest.mark.req("BLD-041")
def test_the_gantry_belts_state_the_placeholder_ratio_fixed_in_advance() -> None:
    belts = {item.name: item.overrides for item in gantry().components if item.type == "belt_pulley"}
    assert belts == {name: {"damping_ratio": "0.01"} for name in ("x_belt", "y_belt", "lift_belt")}


@pytest.mark.req("BLD-040", "BLD-019")
@pytest.mark.parametrize(
    ("supports", "segments"),
    [(["clamped", "free"], 8), (["pinned", "roller"], 8), (["clamped", "roller"], 8), (["clamped", "clamped"], 16)],
)
def test_the_damping_frequency_is_the_calibrated_chains_recorded_first_frequency(
    supports: list[str], segments: int
) -> None:
    """One chain implementation (BLD-040, ruling 80): calibration scales the mass so the chain's first mode is the
    recorded 8 Hz, and the damping conversion's omega_1 is that same chain's, so they agree to rounding (1e-9)."""
    result = _build(_bar(supports, calibration=CALIBRATION, damping_ratio="0.01", segments=segments))
    record = result.lumped[0]
    assert record.first_frequency == pytest.approx(2.0 * math.pi * 8.0, rel=1e-9)
    assert record.node_damping == pytest.approx(2.0 * 0.01 * record.node_stiffness / (2.0 * math.pi * 8.0), rel=1e-9)


@pytest.mark.req("BLD-041", "LIB-035")
def test_a_resolved_transmission_ratio_outside_zero_to_one_is_refused() -> None:
    """[0, 1) holds for the catalog's damping_ratio however it was given, open parameters included (resolved before
    the build): above 1 the catalog's maximum refuses it at validation; 1 itself, which that inclusive maximum admits,
    is a build error."""
    with pytest.raises(BuildError, match="damping_ratio"):
        _belt_build("1")
    with pytest.raises(ValidationError, match="damping_ratio"):
        _belt_build("1.5")


@pytest.mark.req("DES-047")
@pytest.mark.parametrize(("value", "valid"), [("0.02", True), ("1", False), ("-0.5", False)])
def test_an_open_member_ratio_is_range_checked_when_resolved(value: str, valid: bool) -> None:
    from mechsim.testing.runner import _substitute

    data = _bar_data(["clamped", "free"], damping_ratio="${rail_damping}")
    resolved = _substitute(data, {"rail_damping": value}, frozenset())
    if valid:
        assert Design.model_validate(resolved).bodies[0].flexible.damping_ratio == pytest.approx(0.02)  # type: ignore[union-attr]
    else:
        with pytest.raises(ValidationError, match="damping_ratio"):
            Design.model_validate(resolved)
