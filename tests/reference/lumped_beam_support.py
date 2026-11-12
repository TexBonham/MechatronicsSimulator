"""Shared machinery for the lumped-beam references (LIB-032, BLD-015 to BLD-017, BLD-019): a one-member design built
from a flexible_beam catalog part through the builder, its neutral lumped member stepped on the own engine binding.

Closed forms: cantilever tip deflection P L^3 / (3 E I), first mode (1.8751^2 / (2 pi)) sqrt(E I / (rho A L^4));
simply supported midspan deflection P L^3 / (48 E I), first mode (pi / (2 L^2)) sqrt(E I / (rho A)). The chain's own
values (ENG-038 to ENG-041, `core/engine` lumped_beam): tip deflection times (1 - 1 / (4 N^2)), midspan deflection
times (1 - 1 / N^2) for even N; its first mode is the eigenvalue of the linearized chain (`builder.structure`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray

from mechsim.builder import BuildResult, build
from mechsim.design.schema import Design, shipped_materials
from mechsim.library.registry import default_registry
from mechsim.physics.native import _engine as eng
from mechsim.physics.neutral import LumpedMember

PART = "steel_bar_10mm_1m"
CANTILEVER_ROOT = 1.8751040687119611
STEP = 1e-3
# Proposed LIB-032 and BLD-015 to BLD-019 tolerance at N = 8 (docs/spec/lib.md, bld.md; T4).
TOLERANCE = 0.02
SEGMENTS = 8


def beam_design(supports: tuple[str, str], calibration: dict[str, Any] | None = None) -> Design:
    flexible: dict[str, Any] = {"supports": list(supports), "segments": SEGMENTS}
    if calibration is not None:
        flexible["calibration"] = calibration
    return Design.model_validate(
        {
            "name": "reference_beam",
            "version": "1.0.0",
            "source_hash": "unset",
            "status": "release",
            "environment": {"gravity": ["0 m/s**2", "0 m/s**2", "0 m/s**2"], "ambient_temperature": "293.15 K"},
            "bodies": [
                {
                    "name": "bar",
                    "material": "aluminium_6063_t5_extrusion",
                    "process": {"kind": "extrusion"},
                    "geometry": {
                        "kind": "primitives",
                        "shapes": [
                            {"shape": "box", "position": ["0.5 m", "0 m", "0 m"], "size": ["1 m", "0.01 m", "0.01 m"]}
                        ],
                    },
                    "catalog_part": {"type": "flexible_beam", "part": PART},
                    "flexible": flexible,
                }
            ],
            "joints": [
                {
                    "name": "bar_mount",
                    "type": "fixed",
                    "parent": "world",
                    "child": "bar",
                    "placement": {"position": ["0 m", "0 m", "0 m"]},
                }
            ],
        }
    )


def built_member(design: Design) -> tuple[BuildResult, LumpedMember]:
    result = build(design, default_registry(), shipped_materials())
    assert result.physics is not None
    assert len(result.physics.lumped_members) == 1
    return result, result.physics.lumped_members[0]


@dataclass(frozen=True)
class EngineBeam:
    model: Any
    beam: Any


def engine_beam(member: LumpedMember, node_damping: float = 0.0) -> EngineBeam:
    """The neutral lumped member as the engine's BeamSpec, on the world, gravity off."""
    spec = eng.BeamSpec()
    spec.youngs_modulus = member.section.elastic_modulus
    spec.area_moment = member.section.second_moment_of_area
    spec.density = member.section.density
    spec.area = member.section.area
    spec.length = member.section.length
    spec.segments = member.segments
    spec.roller_mass_fraction = member.roller_mass_fraction
    spec.node_damping = node_damping
    if member.root == "clamped" and member.end == "free":
        spec.ends = eng.BeamEnds.Cantilever
    elif member.root == "pinned" and member.end == "roller":
        spec.ends = eng.BeamEnds.SimplySupported
    else:
        raise AssertionError(f"no engine beam for ({member.root}, {member.end})")
    model = eng.MultibodyModel()
    model.gravity = np.zeros(3)
    beam = eng.add_lumped_beam(model, spec)
    return EngineBeam(model, beam)


def flexural_rigidity(member: LumpedMember) -> float:
    return member.section.elastic_modulus * member.section.second_moment_of_area


def frequency_scale(member: LumpedMember) -> float:
    """sqrt(E I / (rho A L^4)), rad/s."""
    section = member.section
    return math.sqrt(flexural_rigidity(member) / (section.density * section.area * section.length**4))


def _rotate(wxyz: Any, vector: NDArray[np.float64]) -> NDArray[np.float64]:
    w, x, y, z = (float(value) for value in wxyz)
    axis = np.array([x, y, z])
    return np.asarray(vector + 2.0 * np.cross(axis, np.cross(axis, vector) + w * vector), dtype=np.float64)


def _world(provider: Any, state: Any, point: Any) -> NDArray[np.float64]:
    if point.body < 0:
        return np.asarray(point.point, dtype=np.float64)
    body = provider.body_states(state.q, state.v)[point.body]
    return np.asarray(body.position, dtype=np.float64) + _rotate(body.orientation, np.asarray(point.point))


def settled_deflection(
    member: LumpedMember, closed_form_omega: float, x_load: float, force: float
) -> tuple[float, float]:
    """World y of the load point after 3000 steps under a held transverse point force, and its change over the
    last 100 steps. Node damping D = (2 / omega_c) K gives every mode zeta_i = omega_i / omega_c, about critical for
    the first and overdamped above it, so all modes decay at least at omega_c / 2: 3 s at omega_c >= 50 rad/s leave
    exp(-75) of the transient, and the fixed point of the step is the chain's static solution (as ENG-038)."""
    segment = member.section.length / member.segments
    damping = 2.0 / closed_form_omega * flexural_rigidity(member) / segment
    built = engine_beam(member, damping)
    provider = eng.MultibodyProvider(built.model)
    integrator = eng.SemiImplicitEuler(eng.StepConfig())
    state = eng.State(provider.neutral_positions(), np.zeros(provider.velocity_size()), 0.0)
    point = eng.beam_point(built.beam, x_load)
    com = np.asarray(built.model.bodies[point.body].com, dtype=np.float64)
    steps = 3000
    marker = 0.0
    load = np.array([0.0, force, 0.0])
    for n in range(steps):
        if n == steps - 100:
            marker = float(_world(provider, state, point)[1])
        body = provider.body_states(state.q, state.v)[point.body]
        origin = np.asarray(body.position, dtype=np.float64)
        at = origin + _rotate(body.orientation, np.asarray(point.point))
        centre = origin + _rotate(body.orientation, com)
        provider.apply_body_wrench(point.body, np.cross(at - centre, load), load)
        provider.step(integrator, state)
    final = float(_world(provider, state, point)[1])
    return final, abs(final - marker)


def simulated_frequency(member: LumpedMember, chain_omega: float, periods: int = 12) -> float:
    """First-mode frequency, rad/s, from upward zero crossings of the free vibration out of the chain's static
    point-load shape (load at the tip, or at midspan for the pinned span), no damping. The first two crossings are
    dropped: under the implicit step a mode at omega_k contracts by (1 + (omega_k h)^2)^(-1/2) per step, so the
    cantilever's mode 2 (6.27 omega_1) and the pinned span's mode 3 (9 omega_1; the symmetric shape holds no mode 2)
    fall below 1e-4 of their start within two periods of mode one (as ENG-039, ENG-041)."""
    built = engine_beam(member)
    provider = eng.MultibodyProvider(built.model)
    beam = built.beam
    length = member.section.length
    stiffness = beam.node_stiffness
    q = np.array(provider.neutral_positions(), dtype=np.float64)
    load = 1e-3 * stiffness
    measure = length
    if member.end == "free":
        for j, body in enumerate(beam.bodies):
            q[provider.position_offset(body)] = -load * (length - beam.starts[j]) / stiffness
    else:
        measure = 0.5 * length
        end_offset = 0.0
        for j in range(1, len(beam.bodies)):
            x = beam.starts[j]
            angle = -0.5 * load * min(x, length - x) / stiffness
            q[provider.position_offset(beam.bodies[j])] = angle
            end_offset += angle * (length - x)
        q[provider.position_offset(beam.bodies[0])] = -end_offset / length
    integrator = eng.SemiImplicitEuler(eng.StepConfig())
    state = eng.State(q, np.zeros(provider.velocity_size()), 0.0)
    point = eng.beam_point(beam, measure)
    steps = math.ceil((periods + 3.0) * 2.0 * math.pi / chain_omega / STEP)
    crossings: list[float] = []
    before = float(_world(provider, state, point)[1])
    for _ in range(steps):
        t0 = state.t
        provider.step(integrator, state)
        after = float(_world(provider, state, point)[1])
        if before < 0.0 <= after:
            crossings.append(t0 + STEP * before / (before - after))
        before = after
    dropped = 2
    assert len(crossings) >= periods + dropped
    counted = len(crossings) - dropped - 1
    return 2.0 * math.pi * counted / (crossings[-1] - crossings[dropped])


def dispersion_expected(chain_omega: float) -> tuple[float, float]:
    """K implicit, no damping: the step rotates each mode by atan(omega h), so the simulated frequency is
    atan(omega h) / h, low by about (omega h)^2 / 3 (package 1). Returns that value and the correction."""
    omega_h = chain_omega * STEP
    return math.atan(omega_h) / STEP, omega_h * omega_h / 3.0
