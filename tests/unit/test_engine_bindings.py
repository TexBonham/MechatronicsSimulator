"""Bindings of the milestone 3 engine elements (brief M3-E3, section 2) through `_core_ext.engine`."""

import math
from typing import Any

import numpy as np
import pytest

from mechsim import _core as core
from mechsim.physics.native import _engine as eng

G = 9.81
H = 1e-3


def _bob(position: tuple[float, float, float], mass: float) -> Any:
    bob = eng.Body()
    bob.name = "bob"
    bob.joint.type = eng.JointType.Translation
    bob.joint.position = list(position)
    bob.mass = mass
    bob.inertia = (1e-3 * mass * np.eye(3)).tolist()
    return bob


def _rest(provider: Any) -> Any:
    return eng.State(provider.neutral_positions(), [0.0] * provider.velocity_size())


def _steel_beam(ends: Any, segments: int) -> Any:
    spec = eng.BeamSpec()
    spec.youngs_modulus = 2.0e11
    spec.area_moment = 1e-8 / 12.0
    spec.density = 7850.0
    spec.area = 1e-4
    spec.length = 1.5
    spec.segments = segments
    spec.ends = ends
    return spec


@pytest.mark.req("ENG-047")
def test_translation_joint_falls_one_semi_implicit_step() -> None:
    # Semi-implicit Euler from rest: v = -g h, then q = h v = -g h^2, exact in floating point to rounding.
    model = eng.MultibodyModel()
    model.bodies = [_bob((0.0, 0.0, 0.0), 1.0)]
    provider = eng.MultibodyProvider(model)
    assert provider.position_size() == 3
    assert provider.velocity_size() == 3
    state = _rest(provider)
    provider.step(eng.SemiImplicitEuler(), state)
    assert state.v[2] == pytest.approx(-G * H, rel=1e-15)
    assert state.q[2] == pytest.approx(-G * H * H, rel=1e-15)
    assert state.q[0] == 0.0
    assert state.q[1] == 0.0


def _two_segment_rope() -> tuple[Any, int]:
    model = eng.MultibodyModel()
    model.bodies = [_bob((0.0, 0.0, -1.0), 1.0)]
    rope = eng.Rope()
    rope.from_body = -1
    rope.to_body = 0
    rope.segments = 2
    rope.axial_rigidity = 1e4
    rope.damping = 50.0
    rope.mass_per_length = 0.1
    rope.rest_length = 1.0
    index = eng.add_rope(model, rope, [0.0, 0.0, 0.0], [0.0, 0.0, -1.0])
    return model, index


@pytest.mark.req("ENG-047")
def test_rope_binding_builds_and_hangs_at_static_tension() -> None:
    # ADR 0016: one interior node of mu l = 0.05 kg, the bob gains mu l / 2 = 0.025 kg, the world end none. At rest
    # the lower segment carries the bob, the upper the bob and the node: T = 1.025 g and 1.075 g. Segment damping
    # c / l = 100 N s/m against k = EA / l = 2e4 N/m and 1.075 kg decays the axial modes by e^-70 or more in 3 s,
    # and the implicit step's fixed point is the static solution, so 1e-9 relative.
    model, index = _two_segment_rope()
    assert index == 0
    assert len(model.ropes) == 1
    assert len(model.ropes[0].nodes) == 1
    node = model.bodies[model.ropes[0].nodes[0]]
    assert node.joint.type == eng.JointType.Translation
    assert node.mass == pytest.approx(0.05, rel=1e-15)
    assert model.bodies[0].mass == pytest.approx(1.025, rel=1e-15)
    provider = eng.MultibodyProvider(model)
    state = _rest(provider)
    integrator = eng.SemiImplicitEuler()
    for _ in range(3000):
        provider.step(integrator, state)
    out = provider.outputs.ropes[0]
    assert len(out.tension) == 2
    assert out.tension[0] == pytest.approx(1.075 * G, rel=1e-9)
    assert out.tension[1] == pytest.approx(1.025 * G, rel=1e-9)
    assert out.max_tension == pytest.approx(1.075 * G, rel=1e-9)
    assert out.load_end_tension == pytest.approx(1.025 * G, rel=1e-9)
    assert out.attached
    assert out.rest_length == 1.0
    # snatch_omega_h = sqrt(EA / (L M)) h with M the bob's built mass (ruling 22): 0.0988, under the 0.2 flag.
    assert out.snatch_omega_h == pytest.approx(math.sqrt(1e4 / 1.025) * H, rel=1e-12)
    assert not out.snatch_underestimated
    # Ruling 21: pi sqrt(g / length) h over the loaded length 1 + (1.075 + 1.025) g / 2e4.
    length = 1.0 + (1.075 + 1.025) * G / 2e4
    assert out.scheme_decay == pytest.approx(math.pi * math.sqrt(G / length) * H, rel=1e-9)
    assert provider.rope_states(state.q, state.v)[0].max_tension == pytest.approx(out.max_tension, rel=1e-9)


@pytest.mark.req("ENG-047")
@pytest.mark.req("ENG-034")
def test_rope_binding_winch_release_and_snatch_flag() -> None:
    model, _ = _two_segment_rope()
    provider = eng.MultibodyProvider(model)
    state = _rest(provider)
    integrator = eng.SemiImplicitEuler()
    provider.set_rope_rest_length(0, 0.9)
    provider.step(integrator, state)
    assert provider.outputs.ropes[0].rest_length == 0.9
    assert provider.outputs.events == []
    assert provider.rope_attached(0)
    provider.release_rope(0)
    provider.step(integrator, state)
    events = provider.outputs.events
    assert len(events) == 1
    assert events[0].kind == eng.ModelChange.LoadReleased
    assert events[0].source == eng.LoadSource.Rope
    assert events[0].target == 0
    assert events[0].time == pytest.approx(H, rel=1e-15)
    assert not provider.rope_attached(0)
    assert not provider.outputs.ropes[0].attached
    assert provider.outputs.ropes[0].load_end_tension == 0.0
    with pytest.raises(core.InvalidModelError):
        provider.release_rope(0)
    with pytest.raises(core.InvalidModelError):
        provider.set_rope_rest_length(0, 0.0)


@pytest.mark.req("ENG-047")
def test_rope_snap_lists_rope_snapped_then_load_released() -> None:
    # Breaking tension below the bob's weight: the first taut step's end tension exceeds it.
    model, _ = _two_segment_rope()
    ropes = model.ropes
    ropes[0].breaking_tension = 0.5 * G
    model.ropes = ropes
    provider = eng.MultibodyProvider(model)
    state = _rest(provider)
    integrator = eng.SemiImplicitEuler()
    kinds: list[Any] = []
    for _ in range(200):
        provider.step(integrator, state)
        kinds.extend(event.kind for event in provider.outputs.events)
        if kinds:
            break
    assert kinds == [eng.ModelChange.RopeSnapped, eng.ModelChange.LoadReleased]
    snapped = provider.outputs.events[0]
    assert snapped.limit == 0.5 * G
    assert snapped.value > snapped.limit
    assert not provider.rope_attached(0)


@pytest.mark.req("ENG-022")
@pytest.mark.req("ENG-030")
def test_clamped_beam_and_member_end_loads_binding() -> None:
    # Gravity -y lies in the bending plane (root z bending): the clamped-clamped chain sags and, with node damping,
    # settles to statics (first mode about 23 Hz, damped within 3 s). The member's segments outside the clamped root
    # stub weigh W = sum m_k g; statics of the whole member (support wrenches on it, moments about x = 0, member frame =
    # world): start_force_y + end_force_y = W and start_moment_z + end_moment_z + L end_force_y = sum m_k g x_k, with
    # x_k each segment's centre. The far lock carries a real end moment, read through ConstraintOutput.torque.
    spec = _steel_beam(eng.BeamEnds.Clamped, 8)
    spec.node_damping = 2.0
    model = eng.MultibodyModel()
    model.gravity = [0.0, -G, 0.0]
    beam = eng.add_lumped_beam(model, spec)
    assert not beam.pinned_root
    assert eng.member_nodes(beam) == beam.bodies
    assert len(model.members) == 1
    assert model.constraints[0].type == eng.ConstraintType.Hinge
    provider = eng.MultibodyProvider(model)
    state = _rest(provider)
    integrator = eng.SemiImplicitEuler()
    for _ in range(3000):
        provider.step(integrator, state)
    member = provider.outputs.members[0]
    segment = 1.5 / 8
    masses = [model.bodies[b].mass for b in beam.bodies]
    spans = [segment] * 7 + [0.5 * segment]
    centres = [start + 0.5 * span for start, span in zip(beam.starts, spans, strict=True)]
    weight = sum(masses) * G
    moment = sum(m * x for m, x in zip(masses, centres, strict=True)) * G
    assert member.start_force[1] + member.end_force[1] == pytest.approx(weight, rel=1e-6)
    assert member.start_moment[2] + member.end_moment[2] + 1.5 * member.end_force[1] == pytest.approx(moment, rel=1e-6)
    assert abs(member.end_moment[2]) > 0.01 * weight * 1.5
    # The hinge rows' torque axes are u x axis_b with axis_b the beam x axis, so the lock's torque has no x (twist)
    # part; with the root frame the world frame, MemberOutput.end_moment is that torque unchanged.
    torque = np.asarray(provider.outputs.constraints[0].torque)
    assert abs(torque[0]) <= 1e-12 * float(np.abs(torque).max())
    assert np.array_equal(np.asarray(member.end_moment), torque)


@pytest.mark.req("ENG-022")
def test_clamped_beam_floor_and_point_roller_torque() -> None:
    with pytest.raises(core.InvalidModelError, match="at least 8 segments, got 7"):
        eng.add_lumped_beam(eng.MultibodyModel(), _steel_beam(eng.BeamEnds.Clamped, 7))
    model = eng.MultibodyModel()
    eng.add_lumped_beam(model, _steel_beam(eng.BeamEnds.SimplySupported, 4))
    provider = eng.MultibodyProvider(model)
    provider.step(eng.SemiImplicitEuler(), _rest(provider))
    assert list(provider.outputs.constraints[0].torque) == [0.0, 0.0, 0.0]


@pytest.mark.req("ENG-030")
def test_cantilever_end_loads_carry_the_weight_at_the_root() -> None:
    # Free far end: the root carries the whole out-of-plane weight W = sum m_k g, and its moment about x = 0
    # balances sum x_k x (-m_k g z) = +sum m_k g x_k y, so the support moment is -sum m_k g x_k about y.
    spec = _steel_beam(eng.BeamEnds.Cantilever, 4)
    model = eng.MultibodyModel()
    beam = eng.add_lumped_beam(model, spec)
    provider = eng.MultibodyProvider(model)
    state = _rest(provider)
    provider.step(eng.SemiImplicitEuler(), state)
    member = provider.outputs.members[0]
    segment = 1.5 / 4
    masses = [model.bodies[b].mass for b in beam.bodies]
    centres = [start + 0.5 * (segment if k < 3 else 0.5 * segment) for k, start in enumerate(beam.starts)]
    weight = sum(masses) * G
    moment = sum(m * x for m, x in zip(masses, centres, strict=True)) * G
    assert member.start_force[2] == pytest.approx(weight, rel=1e-9)
    assert member.start_moment[1] == pytest.approx(-moment, rel=1e-9)
    assert list(member.end_force) == [0.0, 0.0, 0.0]
    assert list(member.end_moment) == [0.0, 0.0, 0.0]


@pytest.mark.req("ENG-031")
@pytest.mark.req("ENG-032")
@pytest.mark.req("ENG-033")
@pytest.mark.req("ENG-034")
def test_mid_run_change_bindings_list_events() -> None:
    spec = _steel_beam(eng.BeamEnds.SimplySupported, 4)
    model = eng.MultibodyModel()
    model.gravity = [0.0, 0.0, 0.0]
    eng.add_lumped_beam(model, spec)
    provider = eng.MultibodyProvider(model)
    state = _rest(provider)
    integrator = eng.SemiImplicitEuler()
    provider.set_spring_offset(0, 1, 0.01)
    provider.set_spring_stiffness(0, 2, 5.0)
    provider.step(integrator, state)
    events = provider.outputs.events
    assert [e.kind for e in events] == [eng.ModelChange.SpringOffset, eng.ModelChange.SpringStiffness]
    assert (events[0].target, events[0].index, events[0].value) == (0, 1, 0.01)
    assert (events[1].target, events[1].index, events[1].value) == (0, 2, 5.0)
    assert events[0].time == 0.0
    assert provider.constraint_enabled(0)
    provider.release_load(0)
    provider.step(integrator, state)
    released = provider.outputs.events
    assert len(released) == 1
    assert released[0].kind == eng.ModelChange.LoadReleased
    assert released[0].source == eng.LoadSource.Constraint
    assert not provider.constraint_enabled(0)
    with pytest.raises(core.InvalidModelError):
        provider.disable_constraint(0)
    for member, node, offset in ((1, 0, 0.1), (-1, 0, 0.1), (0, 4, 0.1), (0, 0, math.nan), (0, 0, math.inf)):
        with pytest.raises(core.InvalidModelError):
            provider.set_spring_offset(member, node, offset)
    for member, node, stiffness in ((1, 0, 1.0), (0, -1, 1.0), (0, 0, -1.0), (0, 0, math.nan)):
        with pytest.raises(core.InvalidModelError):
            provider.set_spring_stiffness(member, node, stiffness)
    other = eng.MultibodyProvider(model)
    other.disable_constraint(0)
    other.step(integrator, _rest(other))
    assert [e.kind for e in other.outputs.events] == [eng.ModelChange.ConstraintDisabled]


@pytest.mark.req("ENG-018")
@pytest.mark.req("ENG-020")
@pytest.mark.req("ENG-035")
def test_contact_limits_config_and_payload_release_bindings() -> None:
    limits = eng.CollisionLimits()
    assert (limits.gjk_iterations, limits.epa_iterations, limits.tolerance) == (64, 128, 1e-12)
    config = eng.MultibodyConfig()
    assert config.friction_slip_threshold == 1e-4
    config.friction_slip_threshold = 2e-4
    assert config.friction_slip_threshold == 2e-4

    floor = eng.Geometry()
    floor.kind = "plane"
    ball = eng.Geometry()
    ball.kind = "sphere"
    ball.radius = 0.05
    ball.position = [0.0, 0.0, 0.04]
    points = eng.collide(floor, ball, limits)
    assert len(points) == 1
    assert points[0].depth == pytest.approx(0.01, rel=1e-12)

    # Two unit cubes as convex hulls reach GJK and EPA; overlapping by 0.5 m along x, the penetration is 0.5 m.
    corners = [[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)]
    hull_a = eng.Geometry()
    hull_a.kind = "convex_hull"
    hull_a.vertices = corners
    hull_b = eng.Geometry()
    hull_b.kind = "convex_hull"
    hull_b.vertices = corners
    hull_b.position = [0.5, 0.1, 0.0]
    deep = eng.collide(hull_a, hull_b, limits)
    assert len(deep) == 1
    assert deep[0].depth == pytest.approx(0.5, rel=1e-9)
    capped = eng.CollisionLimits()
    capped.gjk_iterations = 1
    capped.epa_iterations = 1
    with pytest.raises(core.SolverNonConvergenceError):
        eng.collide(hull_a, hull_b, capped)

    carrier = eng.Body()
    carrier.joint.type = eng.JointType.Fixed
    carrier.joint.position = [0.0, 0.0, 1.0]
    payload = eng.Body()
    payload.joint.type = eng.JointType.Free
    payload.mass = 2.0
    payload.inertia = (0.0133 * np.eye(3)).tolist()
    model = eng.MultibodyModel()
    model.bodies = [carrier, payload]
    provider = eng.MultibodyProvider(model, config)
    q = provider.neutral_positions()
    q[2] = 0.8
    state = eng.State(q, [0.0] * 6)
    integrator = eng.SemiImplicitEuler()
    provider.set_attachment(1, 0, True, state.q)
    provider.step(integrator, state)
    assert [e.kind for e in provider.outputs.events] == [eng.ModelChange.PayloadAttached]
    provider.set_body_mass(1, 3.0)
    provider.set_attachment(1, 0, False, state.q)
    provider.step(integrator, state)
    events = provider.outputs.events
    assert [e.kind for e in events] == [eng.ModelChange.PayloadMass, eng.ModelChange.LoadReleased]
    assert events[1].source == eng.LoadSource.Payload
    assert (events[1].target, events[1].index) == (1, 0)
    with pytest.raises(core.InvalidModelError):
        provider.set_attachment(1, 0, False, state.q)
    for body, mass in ((1, -1.0), (1, 0.0), (5, 1.0), (-1, 1.0), (1, math.nan)):
        with pytest.raises(core.InvalidModelError):
            provider.set_body_mass(body, mass)
    for body, carrier in ((5, 0), (1, 7), (1, -2), (1, 1)):
        with pytest.raises(core.InvalidModelError):
            provider.set_attachment(body, carrier, True, state.q)
    provider.set_attachment(1, 0, True, state.q)
    with pytest.raises(core.InvalidModelError):
        provider.set_attachment(1, -1, False, state.q)
    provider.set_attachment(1, 0, False, state.q)


def _prismatic(axis: tuple[float, float, float], mass: float) -> Any:
    body = eng.Body()
    body.name = "slider"
    body.joint.type = eng.JointType.Prismatic
    body.joint.axis = list(axis)
    body.mass = mass
    return body


def _provider(*bodies: Any) -> Any:
    model = eng.MultibodyModel()
    model.bodies = list(bodies)
    return eng.MultibodyProvider(model)


@pytest.mark.req("ENG-013")
@pytest.mark.req("ENG-021")
def test_joint_limit_binding_stops_and_rejects_bad_limits() -> None:
    # Horizontal slider at 0.3 m/s toward a lower stop at 0, 0.05 m away: the passing step overshoots by
    # h V - gap <= h V (removed by the projection), the next one stops it with impulse m V, then v stays zero.
    body = _prismatic((1.0, 0.0, 0.0), 1.5)
    body.joint.lower = [0.0]
    body.joint.upper = [math.inf]
    assert list(body.joint.lower) == [0.0]
    provider = _provider(body)
    state = eng.State([0.05], [-0.3])
    integrator = eng.SemiImplicitEuler()
    overshoot = 0.0
    for _ in range(400):
        provider.step(integrator, state)
        overshoot = max(overshoot, provider.outputs.joints[0].limit_overshoot[0])
    assert 0.0 < overshoot <= H * 0.3
    assert state.q[0] == 0.0
    assert state.v[0] == 0.0
    for lower, upper in (([0.2], [0.1]), ([math.nan], []), ([math.inf], []), ([], [-math.inf]), ([0.0, 1.0], [])):
        bad = _prismatic((1.0, 0.0, 0.0), 1.0)
        bad.joint.lower = lower
        bad.joint.upper = upper
        with pytest.raises(core.InvalidModelError, match="body 0"):
            _provider(bad)
    ball = eng.Body()
    ball.joint.type = eng.JointType.Spherical
    ball.joint.lower = [0.0]
    with pytest.raises(core.InvalidModelError, match="body 0: joint travel limits need a Revolute or Prismatic"):
        _provider(ball)


@pytest.mark.req("ENG-014")
def test_joint_friction_binding_slides_to_rest_and_rejects_bad_values() -> None:
    # 2 kg at 1 m/s against F = 4 N: the frozen-rate scheme reaches v_s after t* = (v0 - v_s) / a, a = 2 m/s^2,
    # within h (1 + ln(v0 (v_s + a h) / v_s^2)) (spec row), and the joint output reports no creep while sliding.
    body = _prismatic((1.0, 0.0, 0.0), 2.0)
    body.joint.friction = [4.0]
    provider = _provider(body)
    state = eng.State([0.0], [1.0])
    integrator = eng.SemiImplicitEuler()
    vs = eng.MultibodyConfig().friction_slip_threshold
    steps = 0
    while state.v[0] > vs:
        provider.step(integrator, state)
        steps += 1
        assert steps < 10000
    t_star = (1.0 - vs) / 2.0
    assert t_star <= steps * H <= t_star + H * (1.0 + math.log((vs + 2.0 * H) / (vs * vs)))
    for friction in ([-1.0], [math.nan], [math.inf], [1.0, 1.0]):
        bad = _prismatic((1.0, 0.0, 0.0), 1.0)
        bad.joint.friction = friction
        with pytest.raises(core.InvalidModelError, match="body 0: joint friction"):
            _provider(bad)


@pytest.mark.req("ENG-051")
def test_joint_drive_binding_sags_tau_over_k_and_rejects_bad_arguments() -> None:
    # Drive K = 400 N/m, D = 40 N s/m (critical at omega 20) under 2 N: sag tau / K = 5 mm (1e-9 after 3 s,
    # e^-60 (1 + 60) of the start error); no event (an input, not a model change); scheme decay pi omega h.
    provider = _provider(_prismatic((1.0, 0.0, 0.0), 1.0))
    provider.set_joint_drive(0, 0, 0.1, 400.0, 40.0)
    state = eng.State([0.0], [0.0])
    integrator = eng.SemiImplicitEuler()
    for _ in range(3000):
        provider.set_generalized_forces([2.0])
        provider.step(integrator, state)
    assert state.q[0] - 0.1 == pytest.approx(2.0 / 400.0, rel=1e-9)
    assert provider.outputs.events == []
    assert provider.outputs.joints[0].scheme_decay[0] == pytest.approx(math.pi * 20.0 * H, rel=1e-12)
    for joint, dof, rest, stiffness, damping in (
        (1, 0, 0.0, 1.0, 1.0),
        (-1, 0, 0.0, 1.0, 1.0),
        (0, 1, 0.0, 1.0, 1.0),
        (0, 0, math.nan, 1.0, 1.0),
        (0, 0, 0.0, -1.0, 1.0),
        (0, 0, 0.0, math.inf, 1.0),
        (0, 0, 0.0, 1.0, -1.0),
        (0, 0, 0.0, 1.0, math.nan),
    ):
        with pytest.raises(core.InvalidModelError):
            provider.set_joint_drive(joint, dof, rest, stiffness, damping)
    ball = eng.Body()
    ball.joint.type = eng.JointType.Spherical
    with pytest.raises(core.InvalidModelError, match="needs a Revolute, Prismatic or Planar joint"):
        _provider(ball).set_joint_drive(0, 0, 0.0, 1.0, 1.0)
