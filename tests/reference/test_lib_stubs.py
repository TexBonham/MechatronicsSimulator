"""Reference tier tests and stubs for the component library spec (LIB); stubs are strict xfail until converted."""

import math

import pytest

from mechsim.builder import structure
from tests.reference.lumped_beam_support import (
    CANTILEVER_ROOT,
    TOLERANCE,
    beam_design,
    built_member,
    flexural_rigidity,
    frequency_scale,
    settled_deflection,
    simulated_frequency,
)


@pytest.mark.req("LIB-022")
@pytest.mark.xfail(strict=True, reason="stub")
def test_lib_022() -> None:
    """The behavioral stepper type shall reproduce the pull-out torque of its catalog part against the published
    torque-speed curve.
    """
    raise NotImplementedError


@pytest.mark.req("LIB-032")
@pytest.mark.parametrize("supports", [("clamped", "free"), ("pinned", "roller")])
def test_flexible_beam_part_reproduces_the_closed_forms(supports: tuple[str, str]) -> None:
    """The flexible beam type shall reproduce cantilever and simply supported beam deflection and first natural
    frequency against their closed-form results.

    The steel_bar_10mm_1m part, lumped at N = 8 through the builder and stepped on the engine binding. Chain errors
    at N = 8 (derivations in tests/reference/lumped_beam_support.py): tip deflection -1/(4 N^2) = -0.39 %, midspan
    deflection -1/N^2 = -1.56 %, first modes from the chain eigenproblem +0.36 % and -0.002 %, step dispersion
    -(omega h)^2 / 3 (0.09 % and 0.69 % at h = 1 ms): all inside the proposed 2 % (docs/spec/lib.md, T4)."""
    _result, member = built_member(beam_design(supports))
    length = member.section.length
    rigidity = flexural_rigidity(member)
    if supports[1] == "free":
        load = 3.0 * rigidity * 1e-4 / length**2
        deflection = load * length**3 / (3.0 * rigidity)
        omega = CANTILEVER_ROOT**2 * frequency_scale(member)
    else:
        load = 48.0 * rigidity * 1e-4 / length**2
        deflection = load * length**3 / (48.0 * rigidity)
        omega = math.pi**2 * frequency_scale(member)
    at = structure.load_point(supports[1], length)
    final, _change = settled_deflection(member, omega, at, -load)
    chain = structure.chain_layout(member.root, member.end, member.segments, length)
    chain_omega = structure.chain_first_frequency(chain, rigidity, member.section.density * member.section.area)
    frequency = simulated_frequency(member, chain_omega)
    print(f"LIB-032 {supports}: deflection closed form {deflection:.9e} m, measured {-final:.9e} m")
    print(f"LIB-032 {supports}: frequency closed form {omega:.6f} rad/s, measured {frequency:.6f} rad/s")
    assert abs(-final / deflection - 1.0) <= TOLERANCE
    assert abs(frequency / omega - 1.0) <= TOLERANCE
