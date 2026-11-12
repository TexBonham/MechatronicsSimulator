"""Analytical references for the integrity checks (INT-004 to INT-008, INT-010 to INT-012).

Loads follow round 5 ruling 24: member frame, x from start to end, each end's wrench the load applied to the member at
that end. Each test prints the closed-form target and the measured value and asserts against the tolerance the INT
row states (docs/spec/int.md)."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
import pytest

from mechsim.integrity.beam import first_frequency
from mechsim.integrity.checks.buckling import Buckling, BucklingParameters
from mechsim.integrity.checks.deflection import Deflection
from mechsim.integrity.checks.fastener import Fastener, FastenerParameters
from mechsim.integrity.checks.fatigue import Fatigue, FatigueParameters
from mechsim.integrity.checks.impact import Impact, ImpactParameters
from mechsim.integrity.checks.resonance import Resonance, ResonanceParameters
from mechsim.integrity.checks.strength import Strength
from mechsim.integrity.plugin import (
    JointLoads,
    MemberInputs,
    NoParameters,
    RunInputs,
    SectionProperties,
    Spread,
    Wrenches,
)
from mechsim.materials.database import MaterialSet

# Closed forms evaluated in double precision: the check and the reference perform the same few operations, so they
# agree to rounding; 1e-9 relative is the proposed tolerance (INT-004, T4) for strength, buckling, fatigue, fastener,
# impact and resonance.
EXACT = 1e-9
# Deflection is maximised over a 201-point grid (spacing h = L / 200). Near the maximum v' = 0, so sampling misses it by
# at most h^2 max|v''| / 8 with max|v''| = M_max / E I. The references here have v_max / (M_max L^2 / E I) of 1/3
# (cantilever), 1/12 (simply supported midspan), above 1/24 (off-centre) and 1/24 (clamped-clamped), so the relative
# miss is at most 24 / 8 (h / L)^2 = 7.5e-5; 1e-3 relative is the proposed tolerance (INT-006).
DEFLECTION = 1e-3

LENGTH = 0.5
WIDTH = 0.02
HEIGHT = 0.04
AREA = WIDTH * HEIGHT
I_Z = WIDTH * HEIGHT**3 / 12.0
I_Y = HEIGHT * WIDTH**3 / 12.0
C_Y = HEIGHT / 2.0
C_Z = WIDTH / 2.0
MODULUS = 2.0e9
DENSITY = 1270.0
SECTION = SectionProperties(AREA, MODULUS, DENSITY, LENGTH, I_Y, I_Z, C_Y, C_Z)
TEMPERATURE = 293.15


class FakeMaterial:
    """Constant strengths with spreads and a stated S-N curve."""

    def __init__(self, curve: tuple[tuple[float, float], ...] = ((1e3, 100e6), (1e6, 40e6))) -> None:
        self.curve = curve
        self.strengths = {
            "yield": Spread(40e6, 50e6, 60e6),
            "ultimate": Spread(45e6, 55e6, 65e6),
            "shear": Spread(20e6, 25e6, 30e6),
        }

    def strength_spread(
        self, kind: str, temperature: float, direction_body: Sequence[float] | None = None, process: object = None
    ) -> Spread:
        return self.strengths[kind]

    def sn_curve(self) -> tuple[tuple[float, float], ...]:
        return self.curve


def constant(
    rows: int,
    start_force=(0.0, 0.0, 0.0),
    start_moment=(0.0, 0.0, 0.0),
    end_force=(0.0, 0.0, 0.0),
    end_moment=(0.0, 0.0, 0.0),
) -> Wrenches:
    def tile(vector: tuple[float, float, float]) -> np.ndarray:
        return np.tile(np.asarray(vector, dtype=np.float64), (rows, 1))

    return Wrenches(tile(start_force), tile(start_moment), tile(end_force), tile(end_moment))


def member(
    loads: Wrenches, supports=((0.0, "clamped"),), name: str = "rail", frequency: float | None = None
) -> MemberInputs:
    return MemberInputs(name, "fake", SECTION, tuple(supports), loads, natural_frequency=frequency)


def inputs(members: Sequence[MemberInputs], rows: int, dt: float = 1e-3, **extra: object) -> RunInputs:
    times = np.arange(rows, dtype=np.float64) * dt
    return RunInputs(times, tuple(members), {"fake": FakeMaterial()}, TEMPERATURE, **extra)  # type: ignore[arg-type]


def cantilever(load: float, rows: int = 3) -> Wrenches:
    """Tip load -P y at x = L on a member clamped at x = 0: the root reaction is +P y and, balancing the tip load's
    moment L x (-P y) = -P L z, +P L z."""
    return constant(
        rows, start_force=(0.0, load, 0.0), start_moment=(0.0, 0.0, load * LENGTH), end_force=(0.0, -load, 0.0)
    )


def report(name: str, target: float, measured: float) -> None:
    print(f"{name}: closed form {target!r}, measured {measured!r}")


@pytest.mark.req("INT-004", "INT-005")
def test_strength_cantilever_root_stress_is_m_c_over_i() -> None:
    # sigma = M c / I at the root, M = P L about z, c = c_y (ruling 25), no axial force.
    load = 100.0
    target = load * LENGTH * C_Y / I_Z
    result = Strength().evaluate(inputs([member(cantilever(load))], 3), NoParameters(), 1)
    (row,) = result.rows
    report("cantilever root stress", target, row.value)
    assert (row.subject, row.quantity, row.tier) == ("rail", "normal_stress", 1)
    assert row.value == pytest.approx(target, rel=EXACT)
    # Safety factor against the yield spread (min, typ, max) of the material.
    assert row.margin == Spread(40e6 / row.value, 50e6 / row.value, 60e6 / row.value)
    assert row.limit == 40e6


@pytest.mark.req("INT-005")
def test_strength_sums_axial_and_bending_about_both_axes() -> None:
    # Ruling 25: |N| / A + |M_y| c_z / I_y + |M_z| c_y / I_z at the end with the largest value.
    axial, moment_y, moment_z = 500.0, 3.0, 7.0
    loads = constant(
        2, end_force=(axial, 0.0, 0.0), end_moment=(0.0, moment_y, moment_z), start_force=(-axial, 0.0, 0.0)
    )
    target = axial / AREA + moment_y * C_Z / I_Y + moment_z * C_Y / I_Z
    (row,) = Strength().evaluate(inputs([member(loads)], 2), NoParameters(), 1).rows
    report("combined stress", target, row.value)
    assert row.value == pytest.approx(target, rel=EXACT)


@pytest.mark.req("INT-005")
def test_strength_uses_the_materials_direction_dependent_strength_at_ambient_temperature() -> None:
    view = MaterialSet.from_files().integrity_view("petg_printed")
    load = 20.0
    stress = load * LENGTH * C_Y / I_Z
    run = RunInputs(
        np.arange(3) * 1e-3,
        (MemberInputs("rail", "petg_printed", SECTION, ((0.0, "clamped"),), cantilever(load)),),
        {"petg_printed": view},
        TEMPERATURE,
    )
    (row,) = Strength().evaluate(run, NoParameters(), 1).rows
    strength = view.strength_spread("yield", TEMPERATURE, (1.0, 0.0, 0.0))
    assert row.margin is not None
    assert row.margin.min == pytest.approx(strength.min / stress, rel=EXACT)
    assert row.margin.max == pytest.approx(strength.max / stress, rel=EXACT)


@pytest.mark.req("INT-005")
def test_strength_mount_uses_the_joint_reaction_on_the_held_members_section() -> None:
    # Conservative mount stress |F| / A + |M| max(c) / min(I) from the world-aligned reaction.
    rows = 2
    force = np.tile([30.0, 40.0, 0.0], (rows, 1))
    moment = np.tile([0.0, 0.0, 2.0], (rows, 1))
    mount = JointLoads("base_mount", "mount", "base", force, moment, member="rail")
    run = inputs([member(constant(rows))], rows, joints=(mount,))
    rows_out = Strength().evaluate(run, NoParameters(), 1).rows
    target = 50.0 / AREA + 2.0 * C_Y / I_Y
    assert rows_out[1].subject == "base_mount"
    assert rows_out[1].value == pytest.approx(target, rel=EXACT)


@pytest.mark.req("INT-005")
def test_strength_tier_two_adds_shear_and_von_mises() -> None:
    load = 100.0
    shear = load / AREA
    sigma = load * LENGTH * C_Y / I_Z
    rows = Strength().evaluate(inputs([member(cantilever(load))], 3), NoParameters(), 2).rows
    assert [row.quantity for row in rows] == ["normal_stress", "shear_stress", "von_mises_stress"]
    assert rows[1].value == pytest.approx(shear, rel=EXACT)
    assert rows[2].value == pytest.approx(math.sqrt(sigma**2 + 3.0 * shear**2), rel=EXACT)
    assert rows[1].margin is not None
    assert rows[1].margin.typ == pytest.approx(25e6 / shear, rel=EXACT)


def _deflection(loads: Wrenches, supports) -> float:  # type: ignore[no-untyped-def]
    (row,) = (
        Deflection().evaluate(inputs([member(loads, supports)], loads.start_force.shape[0]), NoParameters(), 1).rows
    )
    return row.value


@pytest.mark.req("INT-004", "INT-006")
def test_deflection_cantilever_tip_is_p_l3_over_3_e_i() -> None:
    load = 50.0
    target = load * LENGTH**3 / (3.0 * MODULUS * I_Z)
    measured = _deflection(cantilever(load), ((0.0, "clamped"),))
    report("cantilever tip deflection", target, measured)
    assert measured == pytest.approx(target, rel=DEFLECTION)


@pytest.mark.req("INT-006")
def test_deflection_cantilever_out_of_plane_uses_i_y() -> None:
    # Tip load +P z: root reaction -P z and, balancing L x (P z) = -P L y, +P L y.
    load = 5.0
    loads = constant(
        2, start_force=(0.0, 0.0, -load), start_moment=(0.0, load * LENGTH, 0.0), end_force=(0.0, 0.0, load)
    )
    target = load * LENGTH**3 / (3.0 * MODULUS * I_Y)
    measured = _deflection(loads, ((0.0, "clamped"),))
    report("cantilever z deflection", target, measured)
    assert measured == pytest.approx(target, rel=DEFLECTION)


@pytest.mark.req("INT-006")
def test_deflection_simply_supported_midspan_is_p_l3_over_48_e_i() -> None:
    load = 80.0
    loads = constant(2, start_force=(0.0, load / 2.0, 0.0), end_force=(0.0, load / 2.0, 0.0))
    target = load * LENGTH**3 / (48.0 * MODULUS * I_Z)
    measured = _deflection(loads, ((0.0, "pinned"), (LENGTH, "roller")))
    report("simply supported midspan", target, measured)
    assert measured == pytest.approx(target, rel=DEFLECTION)


@pytest.mark.req("INT-006")
def test_deflection_simply_supported_off_centre_load() -> None:
    # Load P at a from the start, b = L - a: v_max = P b (L^2 - b^2)^(3/2) / (9 sqrt(3) L E I) (a > b).
    load, a = 60.0, 0.35
    b = LENGTH - a
    loads = constant(2, start_force=(0.0, load * b / LENGTH, 0.0), end_force=(0.0, load * a / LENGTH, 0.0))
    target = load * b * (LENGTH**2 - b**2) ** 1.5 / (9.0 * math.sqrt(3.0) * LENGTH * MODULUS * I_Z)
    measured = _deflection(loads, ((0.0, "pinned"), (LENGTH, "roller")))
    report("simply supported off-centre", target, measured)
    assert measured == pytest.approx(target, rel=DEFLECTION)


@pytest.mark.req("INT-006")
def test_deflection_clamped_clamped_midspan_is_p_l3_over_192_e_i() -> None:
    # Midspan load P down: reactions P / 2 up, fixed-end moments +P L / 8 (start) and -P L / 8 (end) about z.
    load = 120.0
    loads = constant(
        2,
        start_force=(0.0, load / 2.0, 0.0),
        start_moment=(0.0, 0.0, load * LENGTH / 8.0),
        end_force=(0.0, load / 2.0, 0.0),
        end_moment=(0.0, 0.0, -load * LENGTH / 8.0),
    )
    target = load * LENGTH**3 / (192.0 * MODULUS * I_Z)
    measured = _deflection(loads, ((0.0, "clamped"), (LENGTH, "clamped")))
    report("clamped-clamped midspan", target, measured)
    assert measured == pytest.approx(target, rel=DEFLECTION)


@pytest.mark.req("INT-004", "INT-007")
def test_buckling_pinned_pinned_column_margin_is_euler_over_load() -> None:
    # P_cr = pi^2 E I_min / L^2 (K = 1); compression P at both ends pushing inwards.
    load = 40.0
    loads = constant(2, start_force=(load, 0.0, 0.0), end_force=(-load, 0.0, 0.0))
    critical = math.pi**2 * MODULUS * I_Y / LENGTH**2
    run = inputs([member(loads, ((0.0, "pinned"), (LENGTH, "roller")))], 2)
    (row,) = Buckling().evaluate(run, BucklingParameters(), 1).rows
    assert row.margin is not None
    report("pinned-pinned buckling margin", critical / load, row.margin.typ)
    assert row.value == pytest.approx(load, rel=EXACT)
    assert row.margin.typ == pytest.approx(critical / load, rel=EXACT)


@pytest.mark.req("INT-007")
@pytest.mark.parametrize(
    ("supports", "factor"),
    [
        (((0.0, "clamped"),), 2.0),
        (((0.0, "clamped"), (LENGTH, "clamped")), 0.5),
        (((0.0, "clamped"), (LENGTH, "pinned")), 0.6992),
    ],
)
def test_buckling_effective_length_factor_follows_the_end_conditions(supports, factor) -> None:  # type: ignore[no-untyped-def]
    load = 10.0
    loads = constant(2, start_force=(load, 0.0, 0.0), end_force=(-load, 0.0, 0.0))
    (row,) = Buckling().evaluate(inputs([member(loads, supports)], 2), BucklingParameters(), 1).rows
    assert dict(row.detail)["effective_length_factor"] == pytest.approx(factor, abs=1e-4)


@pytest.mark.req("INT-004", "INT-008", "INT-037")
@pytest.mark.parametrize(("amplitude", "life"), [(100e6, 1e3), (math.sqrt(100e6 * 40e6), math.sqrt(1e3 * 1e6))])
def test_fatigue_sinusoidal_bending_damage_is_cycles_over_life(amplitude: float, life: float) -> None:
    # M_z = -M0 cos(2 pi t / T) over n periods: the corner fibre carries S_a = M0 c_y / I_z; the turning points
    # -S, +S, ..., -S rainflow to 2n half cycles of range 2 S = n cycles. At an S-N point N is that point's cycles; at
    # the geometric mean of two points log-log interpolation gives the geometric mean of their cycles. D = n / N.
    periods, per_period = 10, 40
    moment0 = amplitude * I_Z / C_Y
    phase = 2.0 * math.pi * np.arange(periods * per_period + 1) / per_period
    rows = phase.shape[0]
    moment = np.zeros((rows, 3))
    moment[:, 2] = -moment0 * np.cos(phase)
    zero = np.zeros((rows, 3))
    loads = Wrenches(zero, moment, zero, -moment)
    result = Fatigue().evaluate(inputs([member(loads)], rows), FatigueParameters(), 1)
    (row,) = result.rows
    report("fatigue damage", periods / life, row.value)
    assert row.value == pytest.approx(periods / life, rel=EXACT)
    assert dict(row.detail)["cycles"] == periods
    assert result.cycle_counts == (("rail", float(periods)),)


@pytest.mark.req("INT-004", "INT-010")
def test_fastener_static_load_margin_is_rating_over_load() -> None:
    rows = 3
    force = np.tile([300.0, 400.0, 0.0], (rows, 1))
    joint = JointLoads("bolts", "fastener", "bracket", force, np.zeros((rows, 3)))
    parameters = FastenerParameters.model_validate({"ratings": {"bolts": "2 kN"}})
    (row,) = Fastener().evaluate(inputs([], rows, joints=(joint,)), parameters, 1).rows
    assert row.margin is not None
    report("fastener margin", 4.0, row.margin.typ)
    assert row.value == pytest.approx(500.0, rel=EXACT)
    assert row.margin.typ == pytest.approx(4.0, rel=EXACT)
    assert row.flags == ()


@pytest.mark.req("INT-004", "INT-011")
def test_impact_step_load_peak_is_factor_times_static_stress() -> None:
    # Tip load steps from 0 to P at row 5; peak stress k P L c_y / I_z over the window, judged against ultimate.
    rows, step, load, factor = 20, 5, 30.0, 2.0
    loads = cantilever(load, rows)
    for array in (loads.start_force, loads.start_moment, loads.end_force):
        array[:step] = 0.0
    parameters = ImpactParameters.model_validate({"impact_factor": factor, "step_threshold": "1 N", "window": "5 ms"})
    (row,) = Impact().evaluate(inputs([member(loads)], rows), parameters, 1).rows
    target = factor * load * LENGTH * C_Y / I_Z
    report("impact stress", target, row.value)
    assert row.value == pytest.approx(target, rel=EXACT)
    assert dict(row.detail)["event_time"] == pytest.approx(step * 1e-3)
    assert row.margin is not None
    assert row.margin.min == pytest.approx(45e6 / target, rel=EXACT)


@pytest.mark.req("INT-004", "INT-012")
def test_resonance_flags_a_member_tuned_to_the_excitation() -> None:
    # Pinned-pinned first mode f = pi / (2 L^2) sqrt(E I_min / (rho A)); an excitation at f is flagged, one at
    # f (1 + 2 band) is not.
    natural = math.pi / (2.0 * LENGTH**2) * math.sqrt(MODULUS * I_Y / (DENSITY * AREA))
    supports = ((0.0, "pinned"), (LENGTH, "roller"))
    probe = member(constant(1), supports)
    report("pinned-pinned first frequency", natural, first_frequency(probe))
    assert first_frequency(probe) == pytest.approx(natural, rel=EXACT)
    band = 0.1
    times = np.arange(4) * 1e-3
    series = {
        "joint:x.velocity": (times, np.asarray([0.0, 0.5, 1.0, 0.5]) * natural / 100.0),
        "joint:y.velocity": (times, np.full(4, natural * (1.0 + 2.0 * band) / 100.0)),
    }
    run = RunInputs(
        times, (member(constant(4), supports),), {"fake": FakeMaterial()}, TEMPERATURE, series=series.__getitem__
    )
    parameters = ResonanceParameters.model_validate(
        {"band": band, "excitations": [{"series": name, "hz_per_unit": 100.0} for name in series]}
    )
    tuned, detuned = Resonance().evaluate(run, parameters, 1).rows
    assert tuned.flags == ("resonance",)
    assert tuned.value == pytest.approx(natural, rel=EXACT)
    assert tuned.time == pytest.approx(2e-3)
    assert detuned.flags == ()


@pytest.mark.req("INT-012")
def test_resonance_uses_the_calibrated_frequency_when_the_design_records_one() -> None:
    times = np.arange(2) * 1e-3
    run = RunInputs(
        times, (member(constant(2), frequency=42.0),), {}, TEMPERATURE, series=lambda _: (times, np.full(2, 42.0))
    )
    parameters = ResonanceParameters.model_validate(
        {"band": 0.05, "excitations": [{"series": "s", "hz_per_unit": 1.0}]}
    )
    (row,) = Resonance().evaluate(run, parameters, 1).rows
    assert row.limit == 42.0
    assert row.flags == ("resonance",)
