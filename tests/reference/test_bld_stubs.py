"""Lumped-beam references through the builder on the own engine binding (BLD-015 to BLD-017, BLD-019).

Bounds: the spec tolerance (proposed 2 percent at N = 8, T4) on the raw quantity; the chain's own closed forms at
1e-6 for statics (the load gives 1e-4 L, geometric nonlinearity O(1e-8)); the step's dispersion atan(omega h) / h
within 10 percent of its correction for frequencies (ENG-039, ENG-041). Derivations in `lumped_beam_support`.
"""

import math

import pytest

from mechsim.builder import structure
from mechsim.physics.neutral import LumpedMember
from tests.reference.lumped_beam_support import (
    CANTILEVER_ROOT,
    SEGMENTS,
    TOLERANCE,
    beam_design,
    built_member,
    dispersion_expected,
    flexural_rigidity,
    frequency_scale,
    settled_deflection,
    simulated_frequency,
)


def _chain(member: LumpedMember) -> structure.Chain:
    return structure.chain_layout(member.root, member.end, member.segments, member.section.length)


@pytest.mark.req("BLD-015")
def test_cantilever_tip_deflection_through_the_builder() -> None:
    # Chain value P L^3 / (3 E I) (1 - 1 / (4 N^2)): -0.390625 % at N = 8, inside the 2 % tolerance.
    _result, member = built_member(beam_design(("clamped", "free")))
    length = member.section.length
    rigidity = flexural_rigidity(member)
    load = 3.0 * rigidity * 1e-4 / length**2
    closed_form = load * length**3 / (3.0 * rigidity)
    chain = closed_form * (1.0 - 1.0 / (4.0 * SEGMENTS**2))
    omega = CANTILEVER_ROOT**2 * frequency_scale(member)
    final, last_change = settled_deflection(member, omega, length, -load)
    measured = -final
    print(
        f"BLD-015: closed form {closed_form:.9e} m, measured {measured:.9e} m, error {measured / closed_form - 1:.3e}"
    )
    assert last_change <= 1e-9 * closed_form
    assert abs(measured / chain - 1.0) <= 1e-6
    assert abs(measured / closed_form - 1.0) <= TOLERANCE


@pytest.mark.req("BLD-016")
def test_cantilever_first_frequency_through_the_builder() -> None:
    _result, member = built_member(beam_design(("clamped", "free")))
    closed_form = CANTILEVER_ROOT**2 * frequency_scale(member)
    chain = structure.chain_first_frequency(
        _chain(member), flexural_rigidity(member), member.section.density * member.section.area
    )
    measured = simulated_frequency(member, chain)
    expected, correction = dispersion_expected(chain)
    print(
        f"BLD-016: closed form {closed_form:.6f} rad/s, chain {chain:.6f}, measured {measured:.6f} rad/s, "
        f"error {measured / closed_form - 1:.3e}"
    )
    assert abs(measured / expected - 1.0) <= 0.1 * correction
    assert abs(measured / closed_form - 1.0) <= TOLERANCE


@pytest.mark.req("BLD-017")
def test_simply_supported_midspan_deflection_and_frequency_through_the_builder() -> None:
    # Chain value P L^3 / (48 E I) (1 - 1 / N^2): -1.5625 % at N = 8 (ENG-040), inside the 2 % tolerance.
    _result, member = built_member(beam_design(("pinned", "roller")))
    assert member.segments == SEGMENTS
    length = member.section.length
    rigidity = flexural_rigidity(member)
    load = 48.0 * rigidity * 1e-4 / length**2
    closed_form = load * length**3 / (48.0 * rigidity)
    chain_deflection = closed_form * (1.0 - 1.0 / SEGMENTS**2)
    omega = math.pi**2 * frequency_scale(member)
    final, last_change = settled_deflection(member, omega, 0.5 * length, -load)
    measured = -final
    print(
        f"BLD-017: closed form {closed_form:.9e} m, measured {measured:.9e} m, error {measured / closed_form - 1:.3e}"
    )
    assert last_change <= 1e-9 * closed_form
    assert abs(measured / chain_deflection - 1.0) <= 1e-6
    assert abs(measured / closed_form - 1.0) <= TOLERANCE

    chain = structure.chain_first_frequency(_chain(member), rigidity, member.section.density * member.section.area)
    frequency = simulated_frequency(member, chain)
    expected, correction = dispersion_expected(chain)
    print(
        f"BLD-017: closed form {omega:.6f} rad/s, chain {chain:.6f}, measured {frequency:.6f} rad/s, "
        f"error {frequency / omega - 1:.3e}"
    )
    assert abs(frequency / expected - 1.0) <= 0.1 * correction
    assert abs(frequency / omega - 1.0) <= TOLERANCE


@pytest.mark.req("BLD-019")
@pytest.mark.parametrize("supports", [("clamped", "free"), ("pinned", "roller")])
def test_calibrated_beam_reproduces_the_recorded_deflection_and_frequency(supports: tuple[str, str]) -> None:
    # Recorded values deliberately off the section's closed forms (deflection 1.1 x, frequency 0.9 x, as an FEA of
    # a member with softer joints would give). Calibration is exact on the chain's linear statics and first mode, so
    # the engine reproduces the recorded deflection at the chain's 1e-6 and the frequency at its step dispersion.
    _plain, raw = built_member(beam_design(supports))
    length = raw.section.length
    rigidity = flexural_rigidity(raw)
    at = structure.load_point(supports[1], length)
    if supports[1] == "free":
        load = 3.0 * rigidity * 1e-4 / length**2
        closed_deflection = load * length**3 / (3.0 * rigidity)
        closed_omega = CANTILEVER_ROOT**2 * frequency_scale(raw)
    else:
        load = 48.0 * rigidity * 1e-4 / length**2
        closed_deflection = load * length**3 / (48.0 * rigidity)
        closed_omega = math.pi**2 * frequency_scale(raw)
    recorded_deflection = 1.1 * closed_deflection
    recorded_omega = 0.9 * closed_omega
    calibration = {
        "source": "fea",
        "design_version": "1.0.0",
        "load": f"{load!r} N",
        "deflection": f"{recorded_deflection!r} m",
        "first_natural_frequency": f"{recorded_omega / (2.0 * math.pi)!r} Hz",
    }
    result, member = built_member(beam_design(supports, calibration))
    record = result.lumped[0]
    assert record.calibrated
    print(f"BLD-019 {supports}: stiffness scale {record.stiffness_scale}, mass scale {record.mass_scale}")
    final, last_change = settled_deflection(member, recorded_omega, at, -load)
    measured = -final
    print(f"BLD-019 {supports}: recorded {recorded_deflection:.9e} m, measured {measured:.9e} m")
    assert last_change <= 1e-9 * recorded_deflection
    assert abs(measured / recorded_deflection - 1.0) <= 1e-6
    assert abs(measured / recorded_deflection - 1.0) <= TOLERANCE

    frequency = simulated_frequency(member, recorded_omega)
    expected, correction = dispersion_expected(recorded_omega)
    print(f"BLD-019 {supports}: recorded {recorded_omega:.6f} rad/s, measured {frequency:.6f} rad/s")
    assert abs(frequency / expected - 1.0) <= 0.1 * correction
    assert abs(frequency / recorded_omega - 1.0) <= TOLERANCE
