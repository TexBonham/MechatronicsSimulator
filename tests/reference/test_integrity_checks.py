"""Analytical references for the integrity checks (INT-004 to INT-008, INT-010 to INT-012).

Loads follow round 5 ruling 24: member frame, x from start to end, each end's wrench the load applied to the member at
that end. Each test prints the closed-form target and the measured value and asserts against the tolerance the INT
row states (docs/spec/int.md)."""

from __future__ import annotations

import json
import math
from collections.abc import Sequence

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
# Each sits within the lumped chain's 2 percent row (rulings 31, 32); the settled residual is below exp(-75). Tier 1
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


def _tier1(member_inputs: MemberInputs, bodies: BodyPoses) -> tuple[float, list[float]]:
    from mechsim import _core as core

    assert member_inputs.loads is not None
    loads = core.MemberEndLoads()
    start = core.Wrench()
    start.force = list(member_inputs.loads.start_force[0])
    start.moment = list(member_inputs.loads.start_moment[0])
    end = core.Wrench()
    end.force = list(member_inputs.loads.end_force[0])
    end.moment = list(member_inputs.loads.end_moment[0])
    loads.start = start
    loads.end = end
    states = []
    for position, orientation in zip(bodies.positions[0], bodies.orientations[0], strict=True):
        state = core.BodyState()
        state.position = list(position)
        state.orientation = list(orientation)
        states.append(state)
    stress, moments = core.integrity_member_stress(_limits_json(member_inputs, len(states)), 0, loads, states)
    return float(stress), [float(moment) for moment in moments]


def _peak_moment(name: str, continuum: float, member_inputs: MemberInputs, bodies: BodyPoses) -> list[float]:
    """Both tiers' peak moment against the continuum peak; returns tier 2's node moments for location checks."""
    run = RunInputs(np.zeros(1), (member_inputs,), {"fake": FakeMaterial()}, TEMPERATURE, bodies=bodies)
    (row,) = Strength().evaluate(run, NoParameters(), 1).rows
    section = member_inputs.section
    tier2 = row.value * section.i_z / section.c_y
    stress, tier1_moments = _tier1(member_inputs, bodies)
    tier1 = stress * section.i_z / section.c_y
    report(f"{name} (tier 2)", continuum, tier2)
    report(f"{name} (tier 1)", continuum, tier1)
    assert abs(tier2 / continuum - 1.0) <= CHAIN
    assert tier1 == pytest.approx(tier2, rel=EXACT)
    moments, _ = node_moments(member_inputs, bodies)
    assert tier1_moments == pytest.approx(list(moments[0]), rel=EXACT, abs=EXACT * abs(continuum))
    return [float(value) for value in moments[0]]


@pytest.mark.req("INT-004", "INT-005", "INT-023")
def test_strength_pinned_centre_load_peak_moment_is_p_l_over_4() -> None:
    load, n = 0.2, 9
    member_inputs, bodies = _settled_chain("SimplySupported", n, (0.5 * BEAM_LENGTH, load))
    moments = _peak_moment("pinned centre load", load * BEAM_LENGTH / 4.0, member_inputs, bodies)
    assert int(np.argmax(np.abs(moments))) == 4


@pytest.mark.req("INT-005", "INT-023")
def test_strength_pinned_load_at_a_peak_moment_is_p_a_b_over_l_under_the_load() -> None:
    load, n = 0.2, 8
    a = 2.5 * BEAM_LENGTH / n
    member_inputs, bodies = _settled_chain("SimplySupported", n, (a, load))
    target = load * a * (BEAM_LENGTH - a) / BEAM_LENGTH
    moments = _peak_moment("pinned load at a", target, member_inputs, bodies)
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
    moments = _peak_moment("pinned own weight", w * BEAM_LENGTH**2 / 8.0, member_inputs, bodies)
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
