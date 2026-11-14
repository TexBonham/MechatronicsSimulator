"""Analytical references for the integrity checks (INT-004 to INT-008, INT-010 to INT-012).

Loads follow round 5 ruling 24: member frame, x from start to end, each end's wrench the load applied to the member at
that end. Each test prints the closed-form target and the measured value and asserts against the tolerance the INT
row states (docs/spec/int.md)."""

from __future__ import annotations

import json
import math
from collections.abc import Callable, Sequence
from dataclasses import replace

import numpy as np
import pytest

from mechsim.integrity.beam import first_frequency, node_moments
from mechsim.integrity.checks.buckling import Buckling, BucklingParameters
from mechsim.integrity.checks.deflection import Deflection
from mechsim.integrity.checks.fastener import Fastener, FastenerParameters
from mechsim.integrity.checks.fatigue import Fatigue, FatigueParameters
from mechsim.integrity.checks.impact import Impact, ImpactParameters
from mechsim.integrity.checks.resonance import Resonance, ResonanceParameters
from mechsim.integrity.checks.strength import Strength
from mechsim.integrity.plugin import (
    BodyPoses,
    CarriedLoad,
    CheckResult,
    JointLoads,
    MemberInputs,
    NodeChain,
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
    loads: Wrenches,
    supports=((0.0, "clamped"),),
    name: str = "rail",
    frequency: float | None = None,
    nodes: NodeChain | None = None,
) -> MemberInputs:
    return MemberInputs(name, "fake", SECTION, tuple(supports), loads, natural_frequency=frequency, nodes=nodes)


def inputs(members: Sequence[MemberInputs], rows: int, dt: float = 1e-3, **extra: object) -> RunInputs:
    times = np.arange(rows, dtype=np.float64) * dt
    return RunInputs(times, tuple(members), {"fake": FakeMaterial()}, TEMPERATURE, **extra)  # type: ignore[arg-type]


def cantilever(load: float, rows: int = 3) -> Wrenches:
    """Tip load -P y at x = L on a member clamped at x = 0: the root reaction is +P y and, balancing the tip load's
    moment L x (-P y) = -P L z, +P L z."""
    return constant(
        rows, start_force=(0.0, load, 0.0), start_moment=(0.0, 0.0, load * LENGTH), end_force=(0.0, -load, 0.0)
    )


def cantilever_poses(tips: Sequence[Sequence[float]]) -> tuple[tuple[NodeChain, ...], BodyPoses]:
    """Node poses of cantilevers along world x clamped at the origin, one chain of 8 nodes per entry of `tips` (its
    tip deflection per row, along -y): interior nodes on the tip-load shape v(x) = v_L x^2 (3 L - x) / (2 L^3) at
    x_j = (j + 1/2) l, the last node level at (L - l/2, -v_L), so its far end is (L, -v_L) and the node-pose
    deflection (distance from the root tangent, world x) is abs(v_L) to rounding."""
    count = 8
    segment = LENGTH / count
    rows = len(tips[0])
    positions = np.zeros((rows, count * len(tips), 3))
    orientations = np.zeros((rows, count * len(tips), 4))
    orientations[:, :, 0] = 1.0
    chains: list[NodeChain] = []
    for k, chain_tips in enumerate(tips):
        for row, tip in enumerate(chain_tips):
            for j in range(count - 1):
                x = (j + 0.5) * segment
                positions[row, k * count + j] = (x, -tip * x * x * (3.0 * LENGTH - x) / (2.0 * LENGTH**3), 0.0)
            positions[row, k * count + count - 1] = (LENGTH - 0.5 * segment, -tip, 0.0)
        chains.append(NodeChain(k * count, count, "clamped", None, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), segment))
    return tuple(chains), BodyPoses(positions, orientations)


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


# ===== Deflection from node poses (INT-006, ruling 28) ===================
#
# A lumped chain (ENG-022: N rigid segments, spring k = E I / l at x_j = (j + 1/2) l) is settled on the own engine under
# a held load and its body poses go to the check as the recorder would publish them. Node damping D = (2 / omega_c) K
# (omega_c the closed-form first mode) puts every mode at zeta_i = omega_i / omega_c >= 1, so 3 s at omega_c >= 50
# rad/s leave exp(-75) of the transient (lumped_beam_support.settled_deflection). Reference lines (INT-006 row): the
# clamped root's tangent for a member with a clamped root, the chord from the pin for a pinned root.
#
# Chain closed forms (core/engine lumped_beam.hpp, unit-load method, the chain being the midpoint rule of
# integral M m / E I):
#   cantilever tip load:          P L^3 / (3 E I)   (1 - 1 / (4 N^2)), N = 8: -0.39 %
#   simply supported centre load: P L^3 / (48 E I)  (1 - 1 / N^2),     N = 8: -1.56 %
#   clamped-clamped centre load:  P L^3 / (192 E I) (1 - 4 / N^2),     N = 16: -1.56 %
# For the pinned and clamped spans the central segment straddles midspan and stays level by symmetry, so its two
# nodes carry the midspan value. Cantilever under its own weight w = rho A g: the chain's tip deflection is
# sum_j theta_j (L - x_j), theta_j = M(x_j) l / (E I) with M(x_j) = sum_{k >= j} w s_k (c_k - x_j) over the segment
# bodies (span s_k, centre c_k; the root stub [0, l/2] belongs to the clamp), against w L^4 / (8 E I); the end-load
# formula read this 17 percent low (5/48 against 1/8). Each value asserts against the continuum within the lumped
# chain's 2 percent (rulings 31, 32), and against the chain's own value within 1e-6: the settled residual is below
# exp(-75) and the large-rotation terms are of order theta^2 < 1e-6 at these loads (theta below 6e-4 rad: tip
# deflections near 3e-4 m on the 1 m bar).
CHAIN = 0.02
SETTLED = 1e-6
BEAM_MODULUS = 70e9
BEAM_SIDE = 0.01
BEAM_AREA = BEAM_SIDE**2
BEAM_I = BEAM_SIDE**4 / 12.0
BEAM_DENSITY = 2700.0
BEAM_LENGTH = 1.0
BETA = {"Cantilever": 1.8751040687119611, "SimplySupported": math.pi, "Clamped": 4.730040744862704}


def _rotate_q(q: Sequence[float], v: np.ndarray) -> np.ndarray:
    w = q[0]
    u = np.asarray(q[1:], dtype=np.float64)
    t = 2.0 * np.cross(u, v)
    return np.asarray(v + w * t + np.cross(u, t), dtype=np.float64)


def _settled_chain(
    ends: str, segments: int, point: tuple[float, float] | None, gravity: float = 0.0
) -> tuple[MemberInputs, BodyPoses]:
    """The chain settled under a transverse force -P y at x (point = (x, P)) or its own weight (gravity), as the check's
    member with its node chain and one row of body poses."""
    from mechsim.physics.native import _engine as eng

    spec = eng.BeamSpec()
    spec.youngs_modulus = BEAM_MODULUS
    spec.area_moment = BEAM_I
    spec.density = BEAM_DENSITY
    spec.area = BEAM_AREA
    spec.length = BEAM_LENGTH
    spec.segments = segments
    spec.ends = getattr(eng.BeamEnds, ends)
    rigidity = BEAM_MODULUS * BEAM_I
    omega = (BETA[ends] / BEAM_LENGTH) ** 2 * math.sqrt(rigidity / (BEAM_DENSITY * BEAM_AREA))
    spec.node_damping = 2.0 / omega * rigidity / (BEAM_LENGTH / segments)
    model = eng.MultibodyModel()
    model.gravity = np.array([0.0, -gravity, 0.0])
    beam = eng.add_lumped_beam(model, spec)
    provider = eng.MultibodyProvider(model)
    integrator = eng.SemiImplicitEuler(eng.StepConfig())
    state = eng.State(provider.neutral_positions(), np.zeros(provider.velocity_size()), 0.0)
    for _ in range(3000):
        if point is not None:
            at = eng.beam_point(beam, point[0])
            body = provider.body_states(state.q, state.v)[at.body]
            origin = np.asarray(body.position, dtype=np.float64)
            where = origin + _rotate_q(body.orientation, np.asarray(at.point, dtype=np.float64))
            centre = origin + _rotate_q(body.orientation, np.asarray(model.bodies[at.body].com, dtype=np.float64))
            load = np.array([0.0, -point[1], 0.0])
            provider.apply_body_wrench(at.body, np.cross(where - centre, load), load)
        provider.step(integrator, state)
    states = provider.body_states(state.q, state.v)
    positions = np.asarray([[s.position for s in states]], dtype=np.float64)
    orientations = np.asarray([[s.orientation for s in states]], dtype=np.float64)
    bodies = list(beam.bodies)
    assert bodies == list(range(bodies[0], bodies[0] + len(bodies)))
    root = "clamped"
    supports: tuple[tuple[float, str], ...] = ((0.0, "clamped"),)
    if ends == "SimplySupported":
        root = "pinned"
        supports = ((0.0, "pinned"), (BEAM_LENGTH, "roller"))
    if ends == "Clamped":
        supports = ((0.0, "clamped"), (BEAM_LENGTH, "clamped"))
    segment = BEAM_LENGTH / segments
    chain = NodeChain(
        bodies[0], len(bodies), root, None, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), segment, rigidity / segment
    )
    section = SectionProperties(
        BEAM_AREA, BEAM_MODULUS, BEAM_DENSITY, BEAM_LENGTH, BEAM_I, BEAM_I, BEAM_SIDE / 2, BEAM_SIDE / 2
    )
    ends = provider.outputs.members[0]
    loads = constant(
        1,
        start_force=tuple(ends.start_force),
        start_moment=tuple(ends.start_moment),
        end_force=tuple(ends.end_force),
        end_moment=tuple(ends.end_moment),
    )
    return MemberInputs("beam", "fake", section, supports, loads, nodes=chain), BodyPoses(positions, orientations)


def _node_deflection(member_inputs: MemberInputs, bodies: BodyPoses) -> float:
    run = RunInputs(np.zeros(1), (member_inputs,), {"fake": FakeMaterial()}, TEMPERATURE, bodies=bodies)
    (row,) = Deflection().evaluate(run, NoParameters(), 1).rows
    return row.value


def _assert_chain(name: str, continuum: float, chain: float, measured: float) -> None:
    report(f"{name} (continuum)", continuum, measured)
    report(f"{name} (chain)", chain, measured)
    assert abs(measured / continuum - 1.0) <= CHAIN
    assert measured == pytest.approx(chain, rel=SETTLED)


@pytest.mark.req("INT-004", "INT-006")
def test_deflection_cantilever_tip_load_from_node_poses_is_p_l3_over_3_e_i() -> None:
    load, n = 0.05, 8
    member_inputs, bodies = _settled_chain("Cantilever", n, (BEAM_LENGTH, load))
    continuum = load * BEAM_LENGTH**3 / (3.0 * BEAM_MODULUS * BEAM_I)
    _assert_chain(
        "cantilever tip load",
        continuum,
        continuum * (1.0 - 1.0 / (4.0 * n * n)),
        _node_deflection(member_inputs, bodies),
    )


@pytest.mark.req("INT-006")
def test_deflection_cantilever_own_weight_from_node_poses_is_w_l4_over_8_e_i() -> None:
    n, g = 8, 0.5
    member_inputs, bodies = _settled_chain("Cantilever", n, None, gravity=g)
    w = BEAM_DENSITY * BEAM_AREA * g
    segment = BEAM_LENGTH / n
    spans = [segment] * (n - 1) + [0.5 * segment]
    starts = [(j + 0.5) * segment for j in range(n)]
    centres = [start + 0.5 * span for start, span in zip(starts, spans, strict=True)]
    chain = 0.0
    for j, x in enumerate(starts):
        moment = sum(w * spans[k] * (centres[k] - x) for k in range(j, n))
        chain += moment * segment / (BEAM_MODULUS * BEAM_I) * (BEAM_LENGTH - x)
    continuum = w * BEAM_LENGTH**4 / (8.0 * BEAM_MODULUS * BEAM_I)
    _assert_chain("cantilever own weight", continuum, chain, _node_deflection(member_inputs, bodies))


@pytest.mark.req("INT-006")
def test_deflection_simply_supported_centre_load_from_node_poses_is_p_l3_over_48_e_i() -> None:
    load, n = 0.2, 8
    member_inputs, bodies = _settled_chain("SimplySupported", n, (0.5 * BEAM_LENGTH, load))
    continuum = load * BEAM_LENGTH**3 / (48.0 * BEAM_MODULUS * BEAM_I)
    _assert_chain(
        "simply supported centre load",
        continuum,
        continuum * (1.0 - 1.0 / (n * n)),
        _node_deflection(member_inputs, bodies),
    )


@pytest.mark.req("INT-006")
def test_deflection_clamped_clamped_centre_load_from_node_poses_is_p_l3_over_192_e_i() -> None:
    load, n = 0.8, 16
    member_inputs, bodies = _settled_chain("Clamped", n, (0.5 * BEAM_LENGTH, load))
    continuum = load * BEAM_LENGTH**3 / (192.0 * BEAM_MODULUS * BEAM_I)
    _assert_chain(
        "clamped-clamped centre load",
        continuum,
        continuum * (1.0 - 4.0 / (n * n)),
        _node_deflection(member_inputs, bodies),
    )


@pytest.mark.req("INT-006")
def test_deflection_of_a_member_built_rigid_is_not_evaluated_with_a_note() -> None:
    result = Deflection().evaluate(inputs([member(cantilever(5.0))], 3), NoParameters(), 1)
    (row,) = result.rows
    assert math.isnan(row.value)
    assert row.flags == ("deflection_not_evaluated",)
    assert result.notes == (
        "deflection of 'rail': not evaluated, the member is built rigid and has no node poses (INT-006)",
    )


# ===== Peak moment along the member (INT-005, INT-023) =====================
#
# The bending moment at every node spring is k times the relative rotation of its two node bodies (k = E I / l), plus
# the recorded end moments; stress M c / I. On the square section with no axial force (pinned and clamped far ends
# slide freely, a cantilever's tip is free) the peak stress is abs(M) c / I exactly, so M = sigma I / c. In a settled
# chain each spring's moment is the exact static moment at x_j = (j + 1/2) l (a joint transmits its spring torque),
# so the chain errs only where the closed-form peak lies off a spring or where a support moment is statically
# indeterminate. The segment counts below put each peak on a spring or an end:
#   pinned centre load P L / 4, N = 9: spring 4 at 4.5 l = L / 2, exact (statically determinate).
#   pinned load at a = 2.5 l, N = 8: P a b / L under the load at spring 2, exact.
#   cantilever tip load P L at the root: the end moment, exact by statics.
#   pinned own weight w L^2 / 8, N = 9: each segment body lies wholly on one side of the centre spring with its weight
#     at its centre, so the left part's statics reproduce the integral: exact.
#   clamped-clamped centre load P L / 8, N = 17 (centre spring 8 at L / 2): the support moment from the chain's
#     compatibility sum_j M(x_j) = 0 with M(x) = M_0 + (P / 2) min(x, L - x) is
#     M_0 = -(P / (2 N)) sum_j min(x_j, L - x_j) = -(72.5 / 578) P L = -0.12543 P L (+0.35 %), the centre
#     M_0 + P L / 4 = 0.12457 P L (-0.35 %).
#   clamped-clamped own weight w L^2 / 12 at the ends, N = 16: the midpoint sum of M over the springs gives
#     -(w L^2 / 12)(1 + 1 / (2 N^2)) (+0.2 %); the root stub [0, l / 2] is carried by the clamp, not the member, a
#     load w l / 2 at l / 4 whose fixed-end moment is at most w l^2 / 8, 1.5 / N^2 = 0.6 % of w L^2 / 12.
# The three statically determinate cases (pinned centre load, pinned load at a, pinned own weight) are exact by statics
# and asserted at what the settled run leaves (ruling 72): the transient below exp(-75), quaternion rounding 1e-16 over
# rotations near 1e-4 rad (1e-12), and the geometry, the one second-order term: the loads act at the deflected points
# and the roller slides, so every arm is its undeformed length times cos(theta) >= 1 - theta^2 / 2, theta the largest
# slope; reactions and loads each carry it, so the moment errs by at most theta^2 relative. theta: P L^2 / (16 E I)
# (centre load), P a b (L + b) / (6 L E I) at the support near the load (load at a), w L^3 / (24 E I) (own weight).
# The indeterminate clamped cases sit within the lumped chain's 2 percent row (rulings 31, 32). Tier 1
# (`core/integrity` through `_core.integrity_member_stress`) on the same end-loads and poses agrees within 1e-9.


def _limits_json(member_inputs: MemberInputs, bodies: int) -> str:
    chain = member_inputs.nodes
    assert chain is not None
    section = member_inputs.section
    supports = [{"at": at, "condition": condition} for at, condition in member_inputs.supports]
    for support in supports:
        if support["condition"] == "roller":
            support["condition"] = "pinned"
    return json.dumps(
        {
            "format": "mechsim-integrity-limits-1",
            "minimum_safety_factor": 1.0,
            "bodies": bodies,
            "checks": {
                "stress": True,
                "buckling": False,
                "deflection": False,
                "fastener": False,
                "yield_offset": False,
            },
            "members": [
                {
                    "name": member_inputs.name,
                    "index": 0,
                    "area": section.area,
                    "i_y": section.i_y,
                    "i_z": section.i_z,
                    "c_y": section.c_y,
                    "c_z": section.c_z,
                    "length": section.length,
                    "elastic_modulus": section.elastic_modulus,
                    "effective_length_factor": 1.0,
                    "allowable_stress": 1e12,
                    "yield_strength": 1e12,
                    "deflection_limit": None,
                    "supports": supports,
                    "first_spring": 0,
                    "segments": chain.count,
                    "segment_length": chain.segment_length,
                    "node_stiffness": chain.stiffness,
                    "nodes": chain.to_limits(),
                }
            ],
            "fastenings": [],
        }
    )


def _tier1(
    member_inputs: MemberInputs, bodies: BodyPoses, carried: Sequence[tuple[float, float, float]] = ()
) -> tuple[float, list[float]]:
    from mechsim import _core as core

    assert member_inputs.loads is not None
    loads = core.MemberEndLoads()
    start = core.Wrench()
    start.force = list(member_inputs.loads.start_force[-1])
    start.moment = list(member_inputs.loads.start_moment[-1])
    end = core.Wrench()
    end.force = list(member_inputs.loads.end_force[-1])
    end.moment = list(member_inputs.loads.end_moment[-1])
    loads.start = start
    loads.end = end
    states = []
    for position, orientation in zip(bodies.positions[-1], bodies.orientations[-1], strict=True):
        state = core.BodyState()
        state.position = list(position)
        state.orientation = list(orientation)
        states.append(state)
    stress, moments = core.integrity_member_stress(
        _limits_json(member_inputs, len(states)), 0, loads, states, [list(item) for item in carried]
    )
    return float(stress), [float(moment) for moment in moments]


def _peak_moment(
    name: str, continuum: float, member_inputs: MemberInputs, bodies: BodyPoses, tolerance: float = CHAIN
) -> list[float]:
    """Both tiers' peak moment against the continuum peak; returns tier 2's node moments for location checks."""
    run = RunInputs(np.zeros(1), (member_inputs,), {"fake": FakeMaterial()}, TEMPERATURE, bodies=bodies)
    (row,) = Strength().evaluate(run, NoParameters(), 1).rows
    section = member_inputs.section
    tier2 = row.value * section.i_z / section.c_y
    stress, tier1_moments = _tier1(member_inputs, bodies)
    tier1 = stress * section.i_z / section.c_y
    report(f"{name} (tier 2)", continuum, tier2)
    report(f"{name} (tier 1)", continuum, tier1)
    assert abs(tier2 / continuum - 1.0) <= tolerance
    assert tier1 == pytest.approx(tier2, rel=EXACT)
    moments, _ = node_moments(member_inputs, bodies)
    assert tier1_moments == pytest.approx(list(moments[0]), rel=EXACT, abs=EXACT * abs(continuum))
    return [float(value) for value in moments[0]]


@pytest.mark.req("INT-004", "INT-005", "INT-023")
def test_strength_pinned_centre_load_peak_moment_is_p_l_over_4() -> None:
    load, n = 0.2, 9
    member_inputs, bodies = _settled_chain("SimplySupported", n, (0.5 * BEAM_LENGTH, load))
    slope = load * BEAM_LENGTH**2 / (16.0 * BEAM_MODULUS * BEAM_I)
    moments = _peak_moment("pinned centre load", load * BEAM_LENGTH / 4.0, member_inputs, bodies, slope**2)
    assert int(np.argmax(np.abs(moments))) == 4


@pytest.mark.req("INT-005", "INT-023")
def test_strength_pinned_load_at_a_peak_moment_is_p_a_b_over_l_under_the_load() -> None:
    load, n = 0.2, 8
    a = 2.5 * BEAM_LENGTH / n
    member_inputs, bodies = _settled_chain("SimplySupported", n, (a, load))
    b = BEAM_LENGTH - a
    target = load * a * b / BEAM_LENGTH
    slope = load * a * b * (BEAM_LENGTH + b) / (6.0 * BEAM_LENGTH * BEAM_MODULUS * BEAM_I)
    moments = _peak_moment("pinned load at a", target, member_inputs, bodies, slope**2)
    assert int(np.argmax(np.abs(moments))) == 2


@pytest.mark.req("INT-005", "INT-023")
def test_strength_cantilever_tip_load_peak_moment_is_p_l_at_the_root() -> None:
    load, n = 0.05, 8
    member_inputs, bodies = _settled_chain("Cantilever", n, (BEAM_LENGTH, load))
    _peak_moment("cantilever tip load", load * BEAM_LENGTH, member_inputs, bodies)
    assert member_inputs.loads is not None
    root = abs(float(member_inputs.loads.start_moment[0, 2]))
    report("cantilever root end moment", load * BEAM_LENGTH, root)
    assert root == pytest.approx(load * BEAM_LENGTH, rel=SETTLED)


@pytest.mark.req("INT-005", "INT-023")
def test_strength_clamped_centre_load_peak_moment_is_p_l_over_8_at_the_ends_and_centre() -> None:
    load, n = 0.8, 17
    member_inputs, bodies = _settled_chain("Clamped", n, (0.5 * BEAM_LENGTH, load))
    target = load * BEAM_LENGTH / 8.0
    moments = _peak_moment("clamped centre load", target, member_inputs, bodies)
    assert member_inputs.loads is not None
    for label, value in (
        ("clamped centre load, start", float(member_inputs.loads.start_moment[0, 2])),
        ("clamped centre load, end", float(member_inputs.loads.end_moment[0, 2])),
        ("clamped centre load, centre spring", moments[n // 2]),
    ):
        report(label, target, abs(value))
        assert abs(abs(value) / target - 1.0) <= CHAIN


@pytest.mark.req("INT-005", "INT-023")
def test_strength_pinned_own_weight_peak_moment_is_w_l2_over_8() -> None:
    n, g = 9, 0.5
    member_inputs, bodies = _settled_chain("SimplySupported", n, None, gravity=g)
    w = BEAM_DENSITY * BEAM_AREA * g
    slope = w * BEAM_LENGTH**3 / (24.0 * BEAM_MODULUS * BEAM_I)
    moments = _peak_moment("pinned own weight", w * BEAM_LENGTH**2 / 8.0, member_inputs, bodies, slope**2)
    assert int(np.argmax(np.abs(moments))) == 4


@pytest.mark.req("INT-005", "INT-023")
def test_strength_clamped_own_weight_peak_moment_is_w_l2_over_12_at_the_ends() -> None:
    n, g = 16, 0.5
    member_inputs, bodies = _settled_chain("Clamped", n, None, gravity=g)
    w = BEAM_DENSITY * BEAM_AREA * g
    target = w * BEAM_LENGTH**2 / 12.0
    _peak_moment("clamped own weight", target, member_inputs, bodies)
    assert member_inputs.loads is not None
    for label, value in (
        ("clamped own weight, start", float(member_inputs.loads.start_moment[0, 2])),
        ("clamped own weight, end", float(member_inputs.loads.end_moment[0, 2])),
    ):
        report(label, target, abs(value))
        assert abs(abs(value) / target - 1.0) <= CHAIN


# ===== Moment under a carried load (rulings 71, 75) ========================
#
# The engine puts a carried load on the two lever nodes around its foot point s (`mechsim/integrity/beam.py`, section
# "Moment under a carried load"): a force F and a couple C at s; every sample but the spring above s is clean, and the
# moments just left and right of s are the clean lines A and B at s. The runs: the data-only bar of
# `tests/unit/test_carried_members.py` (1 m, 20 mm square aluminium, pinned-roller or clamped-clamped, N = 8,
# l = 0.125 m: springs at 0.0625 + 0.125 j) with its 40 mm cube carriage carried at a, the cube's centre offset by
# e = 0.1 m along the member from the carriage's origin (its joint): the weight P = m g acts at a + e, which the engine
# applies as F = -P at s = a and the couple of the weight about the foot point, a step K = P e in the spring-sense
# moment at a (statically equivalent to P at a + e; sign: M(L) = 0 on the pinned span). Settling: the chain's node
# damping D = 2 k / omega (k = E I / l, omega the first bending frequency) damps the first mode critically, the higher
# ones overdamped; omega is about 290 rad/s (pinned) and 660 (clamped), so 600 ticks leave e^(-170) of the start
# transient. Superposition isolates the carriage: carried run minus the same design carrying nothing (its carriage
# then loads the rigid parent, not the beam), both under their own weight. Gravity 0.0981 m/s^2: the deflection under
# own weight is 5 w L^4 / (384 E I) = 1.5e-6 m, and the geometric (second-order) part of the statics, the roller's
# slide and the arms' change, is a few (delta / L)^2 = 2e-12 each, while the spring rotations (about 6e-7 rad) carry
# quaternion rounding of 1e-16 / 6e-7 = 2e-10 relative: both below the 1e-9 of the oracle. Oracles:
#   pinned-pinned, a in {0.1, 0.3125, 0.5, 0.77} (0.3125 on spring 2, 0.5 mid-segment, 0.77 in the last segment but
#     one, so line B is the end's line): M(a-) = P a b / L - K a / L and M(a+) = P a b / L + K b / L within 1e-9
#     relative to P a b / L (the statics of the applied force and couple, R0 = (P b - K) / L);
#   clamped-clamped, a = 0.5: M(a-) and M(a+) within 1e-9 of the chain's statics (statically indeterminate: M0, R0
#     from the clamps' compatibility over the springs, sum_j M_j = 0 and sum_j x_j M_j = 0, the spring above a short of
#     its torque share w K), and M(a+) - M(a-) = K; the continuum's M0, R0 (integral M dx = 0, integral x M dx = 0) is
#     printed: the chain sits w K / N from it (K / 16 here), the couple share its spring above the load lacks;
#   the two tiers on the total fields (own weight and carriage) within 1e-9, and tier 1's (s, F, C) from the poses
#     and reactions against tier 2's within 1e-12.

_CARRIED_TICKS = 600
_CARRIED_GRAVITY = 0.0981
_OFFSET = 0.1


def _carried_bar(carries: bool, a: float, supports: tuple[str, str]) -> tuple[MemberInputs, BodyPoses, float]:
    """The bar after settling, as the check's member with its last two rows of poses (the last row's carried loads
    are read from the one before), and the carriage's weight P."""
    from mechsim.builder import build
    from mechsim.design.schema import Design, shipped_materials
    from mechsim.integrity.inputs import (
        BODY_ORIENTATION,
        BODY_POSITION,
        MEMBER_SERIES,
        carried_loads,
        node_chains,
    )
    from mechsim.library.registry import default_registry
    from tests.unit.test_carried_members import _design_data, _record

    carried: list[str] = []
    if carries:
        carried = ["slide"]
    travel = min(a, 1.0 - a, 0.05)
    data = _design_data(carried, position=a, limits=(-travel, travel))
    data["environment"]["gravity"] = ["0 m/s**2", f"-{_CARRIED_GRAVITY} m/s**2", "0 m/s**2"]
    data["bodies"][0]["flexible"]["segments"] = 8
    data["bodies"][0]["flexible"]["supports"] = list(supports)
    data["bodies"][1]["geometry"]["shapes"][0]["position"] = [f"{_OFFSET} m", "0 m", "0 m"]
    result = build(Design.model_validate(data), default_registry(), shipped_materials())
    model = result.physics
    assert model is not None
    lumped = model.lumped_members[0]
    section = lumped.section
    ei = float(section.elastic_modulus) * float(section.second_moment_of_area)
    rho_a = float(section.density) * float(section.area)
    ends = "SimplySupported"
    if supports[0] == "clamped":
        ends = "Clamped"
    omega = (BETA[ends] / float(section.length)) ** 2 * math.sqrt(ei / rho_a)
    stiffness = ei * 8 / float(section.length)
    damped = lumped.model_copy(update={"node_damping": 2.0 / omega * stiffness})
    model = model.model_copy(update={"lumped_members": (damped,)})
    recording = _record(model, _CARRIED_TICKS)
    chain = node_chains(model)["bar"]
    count = len(model.bodies) + chain.count
    positions = np.asarray(recording.series[BODY_POSITION].values, dtype=np.float64).reshape(-1, count, 3)
    orientations = np.asarray(recording.series[BODY_ORIENTATION].values, dtype=np.float64).reshape(-1, count, 4)
    rows = positions.shape[0]
    wrenches = [
        np.asarray(recording.series[path].values, dtype=np.float64).reshape(-1, 3)[-2:] for path in MEMBER_SERIES
    ]
    loads = carried_loads(model, recording.series, rows).get("bar", ())
    loads = tuple(replace(item, force=item.force[-2:], moment=item.moment[-2:]) for item in loads)
    properties = SectionProperties(
        float(section.area),
        float(section.elastic_modulus),
        float(section.density),
        float(section.length),
        float(section.second_moment_of_area),
        float(section.second_moment_of_area),
        0.01,
        0.01,
    )
    supports_at = ((0.0, supports[0]), (float(section.length), supports[1]))
    member_inputs = MemberInputs(
        "bar", "fake", properties, supports_at, Wrenches(*wrenches), nodes=chain, carried=loads
    )
    weight = float(next(body.mass for body in model.bodies if body.name == "carriage")) * _CARRIED_GRAVITY
    return member_inputs, BodyPoses(positions[-2:], orientations[-2:]), weight


def _carriage_sides(
    carried: tuple[MemberInputs, BodyPoses, float], plain: tuple[MemberInputs, BodyPoses, float]
) -> tuple[float, float]:
    """The rule's M(a-) and M(a+) on the carriage's own field (carried minus plain, moments and end loads) at the last
    row."""
    from mechsim.integrity.beam import carried_reads

    loaded, plain_inputs = carried[0], plain[0]
    assert loaded.loads is not None
    assert plain_inputs.loads is not None
    moments = node_moments(loaded, carried[1])[0] - node_moments(plain_inputs, plain[1])[0]
    loads = Wrenches(
        loaded.loads.start_force - plain_inputs.loads.start_force,
        loaded.loads.start_moment - plain_inputs.loads.start_moment,
        loaded.loads.end_force - plain_inputs.loads.end_force,
        loaded.loads.end_moment - plain_inputs.loads.end_moment,
    )
    reads = carried_reads(loaded, loads, moments, carried[1])
    assert bool(reads.found[-1, 0]) and bool(reads.found[-1, 1])
    assert float(reads.bound[-1]) == 0.0
    return float(reads.values[-1, 0]), float(reads.values[-1, 1])


def _tiers_agree(member_inputs: MemberInputs, bodies: BodyPoses) -> None:
    """Tier 2 on the last row (the strength check's per-row read: the largest section plus the bound) against tier 1
    on the same row with the carried loads tier 2 applies (from the row before), and tier 1's own rebuild of those
    loads from that row's poses and reactions."""
    from mechsim import _core as core
    from mechsim.integrity.beam import carried_at, carried_reads, member_normal_stress

    assert member_inputs.loads is not None
    section = member_inputs.section
    normal = member_normal_stress(member_inputs.loads, section, member_inputs, bodies)
    tier2 = float(normal[-1].max())
    if member_inputs.nodes is not None and member_inputs.nodes.carries:
        moments = node_moments(member_inputs, bodies)[0]
        bound = carried_reads(member_inputs, member_inputs.loads, moments, bodies).bound
        tier2 = tier2 + float(bound[-1]) * section.c_y / section.i_z
    s, force, couple = carried_at(member_inputs, bodies)
    items = [(float(s[-1, j]), float(force[-1, j]), float(couple[-1, j])) for j in range(s.shape[1])]
    stress, _ = _tier1(member_inputs, bodies, items)
    assert stress == pytest.approx(tier2, rel=EXACT)
    if len(member_inputs.carried) == 0:
        return
    assert member_inputs.nodes is not None
    states = []
    for position, orientation in zip(bodies.positions[0], bodies.orientations[0], strict=True):
        state = core.BodyState()
        state.position = list(position)
        state.orientation = list(orientation)
        states.append(state)
    places = [joint for _, joint in member_inputs.nodes.carried]
    joints = [core.JointForce() for _ in range(max(places) + 1)]
    for place, item in zip(places, member_inputs.carried, strict=True):
        reaction = core.Wrench()
        reaction.force = list(-item.force[0])
        reaction.moment = list(-item.moment[0])
        joints[place].reaction = reaction
    rebuilt = core.integrity_carried_at(_limits_json(member_inputs, len(states)), 0, states, joints)
    report("tier 1 rebuild of the carried couple", items[0][2], rebuilt[0][2])
    assert len(rebuilt) == len(items)
    for mine, theirs in zip(rebuilt, items, strict=True):
        assert list(mine) == pytest.approx(list(theirs), rel=1e-12, abs=1e-15)


@pytest.mark.req("INT-005", "INT-023")
@pytest.mark.parametrize("fraction", [0.1, 0.3125, 0.5, 0.77])
def test_strength_carried_force_and_couple_read_both_sides_of_the_load_exactly(fraction: float) -> None:
    carried = _carried_bar(True, fraction, ("pinned", "roller"))
    plain = _carried_bar(False, fraction, ("pinned", "roller"))
    length = carried[0].section.length
    a = fraction * length
    b = length - a
    load = carried[2]
    step = load * _OFFSET
    left, right = _carriage_sides(carried, plain)
    report(f"pinned carried load at {fraction} L, M(a-)", load * a * b / length - step * a / length, left)
    report(f"pinned carried load at {fraction} L, M(a+)", load * a * b / length + step * b / length, right)
    scale = load * a * b / length
    assert left == pytest.approx(load * a * b / length - step * a / length, abs=EXACT * scale)
    assert right == pytest.approx(load * a * b / length + step * b / length, abs=EXACT * scale)
    _tiers_agree(*carried[:2])


def _clamped_chain(load: float, step: float, a: float, length: float, n: int) -> tuple[float, float]:
    """The clamped-clamped chain's statics: M_j = M0 + R0 x_j - P (x_j - a)+ + K H(x_j - a) at the springs, the spring
    above a short of its torque share w K (beam.py header); the clamps hold the chain's end slope sum_j M_j l / (E I)
    and end deflection sum_j (L - x_j) M_j l / (E I) at zero, so sum_j M_j = 0 and sum_j x_j M_j = 0 give M0, R0."""
    l = length / n
    x = (np.arange(n) + 0.5) * l
    nodes = np.concatenate(([0.0], x, [length]))
    k = 0
    while k + 2 < nodes.size and nodes[k + 1] <= a:
        k += 1
    known = -load * np.clip(x - a, 0.0, None) + step * (x > a)
    if k + 1 <= n:
        known[k] = known[k] - (a - nodes[k]) / (nodes[k + 1] - nodes[k]) * step
    matrix = np.asarray([[n, x.sum()], [x.sum(), (x * x).sum()]])
    start, slope = np.linalg.solve(matrix, -np.asarray([known.sum(), (x * known).sum()]))
    before = float(start + slope * a)
    return before, before + step


def _clamped_continuum(load: float, step: float, a: float, length: float) -> tuple[float, float]:
    b = length - a
    matrix = np.asarray([[length, length**2 / 2.0], [length**2 / 2.0, length**3 / 3.0]])
    right_side = np.asarray(
        [
            load * b**2 / 2.0 - step * b,
            load * (length**3 / 3.0 - a * length**2 / 2.0 + a**3 / 6.0) - step * (length**2 - a**2) / 2.0,
        ]
    )
    start, slope = np.linalg.solve(matrix, right_side)
    before = float(start + slope * a)
    return before, before + step


@pytest.mark.req("INT-005", "INT-023")
def test_strength_clamped_carried_force_and_couple_step_by_the_couple_exactly() -> None:
    carried = _carried_bar(True, 0.5, ("clamped", "clamped"))
    plain = _carried_bar(False, 0.5, ("clamped", "clamped"))
    length = carried[0].section.length
    load = carried[2]
    step = load * _OFFSET
    left, right = _carriage_sides(carried, plain)
    before, after = _clamped_chain(load, step, 0.5 * length, length, 8)
    report("clamped carried load, M(a-) (chain statics)", before, left)
    report("clamped carried load, M(a+) (chain statics)", after, right)
    report("clamped carried load, step", step, right - left)
    continuum = _clamped_continuum(load, step, 0.5 * length, length)
    report("clamped carried load, M(a-) (continuum; the chain is off by w K / N, the share its spring lacks)", continuum[0], left)
    scale = max(abs(before), abs(after))
    assert left == pytest.approx(before, abs=EXACT * scale)
    assert right == pytest.approx(after, abs=EXACT * scale)
    assert right - left == pytest.approx(step, rel=EXACT)
    _tiers_agree(*carried[:2])


@pytest.mark.req("INT-005", "INT-023")
def test_strength_of_a_member_carrying_nothing_is_unchanged_to_the_last_bit() -> None:
    """The I4 formula (ends, then every spring) on the same inputs, written out: bit for bit."""
    from mechsim.integrity.beam import member_normal_stress, normal_stress

    member_inputs, bodies, _ = _carried_bar(False, 0.5, ("pinned", "roller"))
    assert member_inputs.loads is not None
    assert member_inputs.nodes is not None
    assert not member_inputs.nodes.carries
    loads, section = member_inputs.loads, member_inputs.section
    moments, where = node_moments(member_inputs, bodies)
    axial = np.maximum(np.abs(loads.start_force[:, 0]), np.abs(loads.end_force[:, 0])) / section.area
    share = where / section.length
    moment_y = np.outer(loads.start_moment[:, 1], 1.0 - share) - np.outer(loads.end_moment[:, 1], share)
    interior = axial[:, None] + np.abs(moment_y) * section.c_z / section.i_y
    interior = interior + np.abs(moments * 1.0) * section.c_y / section.i_z
    expected = np.concatenate((normal_stress(loads, section), interior), axis=1)
    measured = member_normal_stress(loads, section, member_inputs, bodies)
    assert measured.tobytes() == expected.tobytes()
    _tiers_agree(member_inputs, bodies)


@pytest.mark.req("INT-005", "INT-023")
def test_strength_own_weight_under_a_carried_load_errs_by_at_most_three_w_l_squared() -> None:
    """Own weight w, a carried load of zero size at mid-segment s = 0.5 L (N = 16, pinned): line B extrapolates the
    parabola from x_{k+2}, x_{k+3} back to s across 2 l, departing from it by (w / 2)(2 l)(3 l) = 3 w l^2 at most
    (beam.py header); here d = s - x_{k+2} = -1.5 l, so B(s) - M(s) = (w / 2)(1.5 l)(2.5 l) = 1.875 w l^2 above the
    sagging moment (conservative), and line A from x_k = s - l / 2 departs by (w / 2)(l / 2)(3 l / 2) = 0.375 w l^2.
    The settled springs sit on the parabola (each node's weight at its centre) within the settled residual."""
    from mechsim.integrity.beam import _root_frame

    n, g = 16, 0.5
    member_inputs, bodies = _settled_chain("SimplySupported", n, None, gravity=g)
    assert member_inputs.nodes is not None
    rows = bodies.positions.shape[0]
    positions = np.concatenate((bodies.positions, np.zeros((rows, 1, 3))), axis=1)
    orientations = np.concatenate((bodies.orientations, np.tile([1.0, 0.0, 0.0, 0.0], (rows, 1, 1))), axis=1)
    origin, axis, _, _ = _root_frame(member_inputs, BodyPoses(positions, orientations))
    positions[:, -1, :] = origin + 0.5 * BEAM_LENGTH * axis
    poses = BodyPoses(positions, orientations)
    zero = np.zeros((rows, 3))
    item = CarriedLoad("slide", positions.shape[1] - 1, zero, zero.copy())
    member_inputs = replace(
        member_inputs, nodes=replace(member_inputs.nodes, carries=True), carried=(item,)
    )
    w = BEAM_DENSITY * BEAM_AREA * g
    l = BEAM_LENGTH / n
    continuum = w * BEAM_LENGTH**2 / 8.0
    run = RunInputs(np.zeros(rows), (member_inputs,), {"fake": FakeMaterial()}, TEMPERATURE, bodies=poses)
    (row,) = Strength().evaluate(run, NoParameters(), 1).rows
    measured = row.value * member_inputs.section.i_z / member_inputs.section.c_y
    report("pinned own weight under a carried point, line B", continuum + 1.875 * w * l**2, measured)
    assert measured >= continuum * (1.0 - SETTLED)
    assert measured - continuum <= 3.0 * w * l**2 + SETTLED * continuum
    assert measured == pytest.approx(continuum + 1.875 * w * l**2, rel=SETTLED)


@pytest.mark.req("INT-005")
def test_strength_of_a_member_without_node_poses_reads_its_ends_with_a_note() -> None:
    result = Strength().evaluate(inputs([member(cantilever(5.0))], 3), NoParameters(), 1)
    assert result.notes == ("strength of 'rail': end moments only, the member has no node poses (INT-005)",)


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


# ===== Fatigue and impact along the member (INT-008, INT-011, ruling 72) =====
#
# A pinned-pinned member (the test SECTION, L = 0.5 m, N = 9, l = L / 9) under a centre load P: by statics the end
# moments are zero, the reactions P / 2 each, and the moment at spring x_j = (j + 1/2) l is (P / 2) min(x_j, L - x_j);
# spring 4 sits at 4.5 l = L / 2 and carries P L / 4. The node poses are built from these moments exactly (node 0 the
# pin segment, node j + 1 turned about z by the sum of M_i / k up to spring j, k = E I_z / l), so the springs read
# the statics to quaternion rounding (1e-16 over rotations near 1e-3 rad: 1e-13 relative, below 1e-9). An end reading
# sees no bending at all here: zero damage, zero cycles, zero impact stress.
#   Fatigue: P(t) = -P0 cos(2 pi t / T) over n periods; the centre fibre carries S_a = (P0 L / 4) c_y / I_z, and the
#     turning points -S, +S, ..., -S rainflow to exactly n cycles of range 2 S (the existing fatigue oracle's
#     argument); P0 is set so S_a = 100 MPa, the curve's first point, life 1e3: D = n / 1e3.
#   Impact: P steps from 0 to P1 at row 5; the event is the end force's rise P1 / 2; the peak over the window is
#     factor (P1 L / 4) c_y / I_z at mid-span.
_SPAN = 9


def _pinned_centre(loads_p: np.ndarray) -> tuple[MemberInputs, BodyPoses]:
    rows = loads_p.shape[0]
    segment = LENGTH / _SPAN
    stiffness = MODULUS * I_Z / segment
    where = (np.arange(_SPAN) + 0.5) * segment
    moments = np.outer(loads_p / 2.0, np.minimum(where, LENGTH - where))
    angles = np.cumsum(moments / stiffness, axis=1)
    orientations = np.zeros((rows, _SPAN + 1, 4))
    orientations[:, :, 0] = 1.0
    orientations[:, 1:, 0] = np.cos(angles / 2.0)
    orientations[:, 1:, 3] = np.sin(angles / 2.0)
    positions = np.zeros((rows, _SPAN + 1, 3))
    zero = np.zeros((rows, 3))
    reaction = np.zeros((rows, 3))
    reaction[:, 1] = loads_p / 2.0
    chain = NodeChain(0, _SPAN + 1, "pinned", None, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), segment, stiffness)
    supports = ((0.0, "pinned"), (LENGTH, "roller"))
    member_inputs = MemberInputs(
        "rail", "fake", SECTION, supports, Wrenches(reaction, zero, reaction.copy(), zero.copy()), nodes=chain
    )
    return member_inputs, BodyPoses(positions, orientations)


@pytest.mark.req("INT-004", "INT-008")
def test_fatigue_of_a_pinned_member_reads_the_centre_bending_amplitude_and_counts_every_cycle() -> None:
    periods, per_period = 12, 40
    amplitude = 100e6
    load0 = amplitude * I_Z / C_Y * 4.0 / LENGTH
    phase = 2.0 * math.pi * np.arange(periods * per_period + 1) / per_period
    member_inputs, bodies = _pinned_centre(-load0 * np.cos(phase))
    run = inputs([member_inputs], phase.shape[0], bodies=bodies)
    result = Fatigue().evaluate(run, FatigueParameters(), 1)
    (row,) = result.rows
    report("pinned centre fatigue damage", periods / 1e3, row.value)
    report("pinned centre fatigue cycles", float(periods), dict(row.detail)["cycles"])
    assert row.value == pytest.approx(periods / 1e3, rel=EXACT)
    assert dict(row.detail)["cycles"] == periods
    assert result.cycle_counts == (("rail", float(periods)),)


@pytest.mark.req("INT-004", "INT-011")
def test_impact_on_a_pinned_member_reads_the_mid_span_peak() -> None:
    rows, step, load, factor = 20, 5, 30.0, 2.0
    loads_p = np.full(rows, load)
    loads_p[:step] = 0.0
    member_inputs, bodies = _pinned_centre(loads_p)
    parameters = ImpactParameters.model_validate({"impact_factor": factor, "step_threshold": "1 N", "window": "5 ms"})
    (row,) = Impact().evaluate(inputs([member_inputs], rows, bodies=bodies), parameters, 1).rows
    target = factor * load * LENGTH / 4.0 * C_Y / I_Z
    report("pinned centre impact stress", target, row.value)
    assert row.value == pytest.approx(target, rel=EXACT)
    assert dict(row.detail)["event_time"] == pytest.approx(step * 1e-3)


# ===== Carried couples and clusters (INT-005, rulings 72, 75) ================
#
# A pinned-pinned member (SECTION, L = 0.5 m, N = 8, l = 0.0625 m, samples x_0 = 0, x_i = (i - 1/2) l, x_9 = L)
# carries a body at a = 0.2 m (between x_3 = 0.15625 and x_4 = 0.21875) at a height d = 0.02 m above its axis, loaded
# by its weight P = 30 N and an axial push H at that height: the engine reroutes F . y = -P and the couple
# C = ((r_c - p) x F) . z = -d H. Statics with a step +C in the spring-sense moment at a (the convention these
# synthetic fields use): R_0 = P b / L - C / L, M(x) = R_0 x left of a, R_0 x - P (x - a) + C right of it; the spring
# moments and end wrenches are these exactly except the spring above a (x_4), which lacks the share w_4 C (beam.py
# header), and the node poses are built from them (quaternion rounding 1e-13 relative). The rule reads M(a-) from the
# clean samples x_2, x_3 and M(a+) from x_5, x_6: exact, no bound. For the 400 N push the read is P a b / L + C b / L
# = 7.2 + 1.2 = 8.4 N m (the I5 bound read 15.875, the plain samples 7.875). Two loads of opposite sign in one segment
# (0.225 and 0.27 m, both in [x_4, x_5]) or in neighbouring segments (0.195 and 0.24 m): the moment between them is
# linear, so lines A and B at the two loads give the peak exactly. Bounded clusters, the read at or above the true
# peak and within U of it: a couple with a second load within two segments, U = sum abs(F) (b - a) / 4 + sum abs(C);
# three loads; a load in a clamped root's stub.

_SPRINGS = 8
_SEGMENT = LENGTH / _SPRINGS


def _carried_member(
    moment_at: Callable[[np.ndarray], np.ndarray],
    start_fy: float,
    end_fy: float,
    carried: Sequence[tuple],
    root: str = "pinned",
    start_mz: float = 0.0,
) -> tuple[MemberInputs, BodyPoses]:
    """The chain whose springs carry `moment_at` (each carried couple's missing share at the spring above its load
    taken off, the chain's own reading), with the carried bodies at their positions and loads; two identical rows."""
    n = _SPRINGS
    stiffness = MODULUS * I_Z / _SEGMENT
    where = (np.arange(n) + 0.5) * _SEGMENT
    samples = moment_at(where)
    nodes = np.concatenate(([0.0], where, [LENGTH]))
    for position, force in carried:
        a = position[0]
        couple = position[1] * -force[0]
        k = 0
        while k + 2 < nodes.size and nodes[k + 1] <= a:
            k += 1
        if k + 1 <= n:
            samples[k] = samples[k] - (a - nodes[k]) / (nodes[k + 1] - nodes[k]) * couple
    angles = np.cumsum(samples / stiffness)
    count = n + 1 + len(carried)
    first = 1
    if root == "clamped":
        count = n + len(carried)
        first = 0
    orientations = np.zeros((2, count, 4))
    orientations[:, :, 0] = 1.0
    orientations[:, first : first + n, 0] = np.cos(angles / 2.0)
    orientations[:, first : first + n, 3] = np.sin(angles / 2.0)
    positions = np.zeros((2, count, 3))
    loads = []
    base = count - len(carried)
    for k, (position, force) in enumerate(carried):
        positions[:, base + k] = position
        loads.append(CarriedLoad(f"slide{k}", base + k, np.tile(force, (2, 1)), np.zeros((2, 3))))
    zero = np.zeros((2, 3))
    joints = tuple((base + k, k) for k in range(len(carried)))
    chain = NodeChain(0, base, root, None, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), _SEGMENT, stiffness, True, joints)
    end_condition = "roller"
    if root == "clamped":
        end_condition = "clamped"
    member_inputs = MemberInputs(
        "rail",
        "fake",
        SECTION,
        ((0.0, root), (LENGTH, end_condition)),
        Wrenches(
            np.tile([0.0, start_fy, 0.0], (2, 1)),
            np.tile([0.0, 0.0, start_mz], (2, 1)),
            np.tile([0.0, end_fy, 0.0], (2, 1)),
            zero.copy(),
        ),
        nodes=chain,
        carried=tuple(loads),
    )
    return replace(member_inputs, spring_offsets=None), BodyPoses(positions, orientations)


def _true_peak(moment_at: Callable[[np.ndarray], np.ndarray], points: Sequence[float]) -> float:
    xs = np.concatenate((np.linspace(0.0, LENGTH, 20001), np.asarray(points)))
    return float(np.max(np.abs(moment_at(xs))))


def _strength_moment(member_inputs: MemberInputs, bodies: BodyPoses) -> tuple[float, CheckResult]:
    result = Strength().evaluate(inputs([member_inputs], 2, bodies=bodies), NoParameters(), 1)
    return result.rows[0].value * I_Z / C_Y, result


def _pinned_field(loads: Sequence[tuple[float, float, float]]) -> tuple[Callable[[np.ndarray], np.ndarray], float, float]:
    """M(x) of point loads (a, P down, step C) on the pinned span, and the start and end shear F_y0, F_y1."""
    start = sum(p * (LENGTH - a) / LENGTH - c / LENGTH for a, p, c in loads)
    end = sum(p * a / LENGTH + c / LENGTH for a, p, c in loads)

    def moment_at(x: np.ndarray) -> np.ndarray:
        value = start * x
        for a, p, c in loads:
            value = value - p * np.clip(x - a, 0.0, None) + c * (x > a)
        return value

    return moment_at, start, end


@pytest.mark.req("INT-005", "INT-023")
@pytest.mark.parametrize("push", [400.0, -400.0])
def test_strength_carried_couple_reads_exactly_without_a_bound(push: float) -> None:
    a, d, load = 0.2, 0.02, 30.0
    b = LENGTH - a
    step = d * push
    moment_at, start_fy, end_fy = _pinned_field([(a, load, step)])
    carried = (((a, d, 0.0), (-push, -load, 0.0)),)
    member_inputs, bodies = _carried_member(moment_at, start_fy, end_fy, carried)
    true = max(abs(load * a * b / LENGTH - step * a / LENGTH), abs(load * a * b / LENGTH + step * b / LENGTH))
    plain, _ = _strength_moment(replace(member_inputs, carried=()), bodies)
    measured, result = _strength_moment(member_inputs, bodies)
    report(f"carried couple, push {push} N: true peak", true, measured)
    report(f"carried couple, push {push} N: plain read", true, plain)
    assert plain < true
    assert measured == pytest.approx(true, rel=EXACT)
    if push > 0.0:
        assert measured == pytest.approx(8.4, rel=EXACT)
    assert dict(result.rows[0].detail) == {}
    assert not any("conservative" in note for note in result.notes)
    _tiers_agree(member_inputs, bodies)


@pytest.mark.req("INT-005", "INT-023")
@pytest.mark.parametrize(("first", "second"), [(0.225, 0.27), (0.195, 0.24)])
def test_strength_two_opposite_loads_read_exactly_by_the_chord(first: float, second: float) -> None:
    load1, load2 = 50.0, -20.0
    moment_at, start_fy, end_fy = _pinned_field([(first, load1, 0.0), (second, load2, 0.0)])
    carried = (((first, 0.0, 0.0), (0.0, -load1, 0.0)), ((second, 0.0, 0.0), (0.0, -load2, 0.0)))
    member_inputs, bodies = _carried_member(moment_at, start_fy, end_fy, carried)
    true = _true_peak(moment_at, [first, second])
    measured, result = _strength_moment(member_inputs, bodies)
    report(f"opposite loads at {first}, {second}: true peak", true, measured)
    assert measured == pytest.approx(true, rel=EXACT)
    assert dict(result.rows[0].detail) == {}


@pytest.mark.req("INT-005", "INT-023")
@pytest.mark.parametrize(
    ("cluster", "kind"),
    [
        (((0.2, 30.0, 8.0), (0.27, -20.0, 0.0)), "couple"),
        (((0.2, 30.0, 0.0), (0.24, -20.0, 0.0), (0.3, 10.0, 0.0)), "three"),
    ],
)
def test_strength_unresolved_clusters_read_conservatively_within_the_bound(
    cluster: tuple[tuple[float, float, float], ...], kind: str
) -> None:
    # Segments k: 0.2 -> 3, 0.24 -> 4, 0.27 -> 4, 0.3 -> 5; [a, b] = [x_3, x_{k_last + 2}] = [0.15625, 0.34375] (couple)
    # and [0.15625, 0.40625] (three).
    from mechsim.integrity.beam import COUPLED, CROWDED

    d = 0.02
    moment_at, start_fy, end_fy = _pinned_field(list(cluster))
    carried = tuple(((a, d, 0.0), (-c / d, -p, 0.0)) for a, p, c in cluster)
    member_inputs, bodies = _carried_member(moment_at, start_fy, end_fy, carried)
    true = _true_peak(moment_at, [a for a, _, _ in cluster])
    bound = 50.0 * 0.1875 / 4.0 + 8.0
    cause = COUPLED
    if kind == "three":
        bound = 60.0 * 0.25 / 4.0
        cause = CROWDED
    measured, result = _strength_moment(member_inputs, bodies)
    report(f"cluster ({kind}): true peak", true, measured)
    assert measured >= true * (1.0 - EXACT)
    assert measured <= (true + bound) * (1.0 + EXACT)
    assert dict(result.rows[0].detail)["unresolved_moment"] == pytest.approx(bound, rel=EXACT)
    assert any("conservative" in note and cause in note for note in result.notes)
    _tiers_agree(member_inputs, bodies)


@pytest.mark.req("INT-005", "INT-023")
def test_strength_a_load_in_a_clamped_roots_stub_is_bounded_with_the_stub_share() -> None:
    """k = 0 under a clamped root: no clean line left of the load; one read (line B), U = abs(F) (x_2 - 0) / 4 +
    abs(C) + abs(F) l / 8 + abs(C) (beam.py header)."""
    from mechsim.integrity.beam import STUB, row_reads

    x = np.concatenate(([0.0], (np.arange(_SPRINGS) + 0.5) * _SEGMENT, [LENGTH]))
    reads, bound, causes = row_reads(x, np.zeros(x.size), 0.0, 0.0, False, _SEGMENT, [(0.01, -50.0, 1.5)])
    assert len(reads) == 1
    assert bound == pytest.approx(50.0 * x[2] / 4.0 + 1.5 + 50.0 * _SEGMENT / 8.0 + 1.5, rel=1e-15)
    assert causes == {STUB}
    reads, bound, causes = row_reads(x, np.zeros(x.size), 0.0, 0.0, True, _SEGMENT, [(0.01, -50.0, 1.5)])
    assert len(reads) == 2
    assert bound == 0.0


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
