"""ENG stubs, unit tier (T3); Catch2 tests replace them with the C++ code."""

import math
from typing import Any

import pytest

from mechsim import _core as core
from mechsim.physics.native import _engine as eng


def _slider(axis: tuple[float, float, float], mass: float) -> Any:
    body = eng.Body()
    body.name = "slider"
    body.joint.type = eng.JointType.Prismatic
    body.joint.axis = list(axis)
    body.mass = mass
    return body


def _single(body: Any) -> Any:
    model = eng.MultibodyModel()
    model.bodies = [body]
    return eng.MultibodyProvider(model)


@pytest.mark.req("ENG-013")
def test_eng_013() -> None:
    """The engine shall enforce joint travel limits as one-sided constraints."""
    # 2 kg on a vertical prismatic joint resting on its lower stop: the row returns the step's -g h to zero with
    # impulse m g h, so the stop carries m g (1e-9) and the coordinate sits on the bound with v exactly zero.
    body = _slider((0.0, 0.0, 1.0), 2.0)
    body.joint.lower = [0.0]
    provider = _single(body)
    state = eng.State([0.0], [0.0])
    integrator = eng.SemiImplicitEuler()
    provider.step(integrator, state)
    assert state.q[0] == 0.0
    assert state.v[0] == 0.0
    assert provider.outputs.joints[0].limit_force[0] == pytest.approx(2.0 * 9.81, rel=1e-9)
    # One-sided: a lift F = 30 N > m g leaves at (F - m g) / m from the first step, the stop carrying nothing.
    provider.set_generalized_forces([30.0])
    provider.step(integrator, state)
    assert state.v[0] == pytest.approx(1e-3 * (30.0 - 2.0 * 9.81) / 2.0, rel=1e-12)
    assert provider.outputs.joints[0].limit_force[0] == 0.0


@pytest.mark.req("ENG-014")
def test_eng_014() -> None:
    """The engine shall apply joint friction and joint damping from per-joint parameters."""
    # Friction F = 2 N held against L = 1.5 N: below v_s the friction is the damper F / v_s, whose fixed point is
    # the creep speed v_s L / F = 7.5e-5 m/s (ruling 38); the step contracts the error by 1 / 21.
    body = _slider((1.0, 0.0, 0.0), 1.0)
    body.joint.friction = [2.0]
    provider = _single(body)
    state = eng.State([0.0], [0.0])
    integrator = eng.SemiImplicitEuler()
    for _ in range(40):
        provider.set_generalized_forces([1.5])
        provider.step(integrator, state)
    creep = eng.MultibodyConfig().friction_slip_threshold * 1.5 / 2.0
    assert state.v[0] == pytest.approx(creep, rel=1e-9)
    assert provider.outputs.joints[0].friction_creep[0] == pytest.approx(creep, rel=1e-9)


@pytest.mark.req("ENG-015")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_015() -> None:
    """The engine shall apply joint backlash from a per-joint parameter."""
    raise NotImplementedError


def _sphere(radius: float, position: tuple[float, float, float], body: int = -1) -> Any:
    g = eng.Geometry()
    g.kind = "sphere"
    g.radius = radius
    g.position = position
    g.body = body
    g.material = "steel"
    return g


def _floor() -> Any:
    g = eng.Geometry()
    g.kind = "plane"
    g.material = "steel"
    return g


def _ball_on_floor(height: float) -> tuple[Any, Any]:
    body = eng.Body()
    body.name = "ball"
    body.joint.type = eng.JointType.Free
    body.mass = 1.0
    body.inertia = [[1e-3, 0.0, 0.0], [0.0, 1e-3, 0.0], [0.0, 0.0, 1e-3]]
    model = eng.MultibodyModel()
    model.bodies = [body]
    provider = eng.MultibodyProvider(model)
    contacts = eng.ContactModel()
    contacts.geometries = [_floor(), _sphere(0.05, (0.0, 0.0, 0.0), body=0)]
    contacts.default_parameters.stiffness = 1e5
    contacts.default_parameters.damping = 1e3
    contacts.default_parameters.friction = 0.5
    provider.set_contact_model(contacts)
    q = provider.neutral_positions()
    q[2] = height
    return provider, q


@pytest.mark.req("ENG-016")
def test_eng_016() -> None:
    """The engine shall detect collisions between primitives and between convex hulls with in-house code."""
    # Spheres r 0.5 and 0.3, centers 0.7 apart along (0.6, 0.2, -0.3) / 0.7: depth 0.8 - 0.7 = 0.1, normal a to b.
    found = eng.collide(_sphere(0.5, (0.0, 0.0, 0.0)), _sphere(0.3, (0.6, 0.2, -0.3)))
    assert len(found) == 1
    assert found[0].depth == pytest.approx(0.1, abs=1e-9)
    assert list(found[0].normal) == pytest.approx([0.6 / 0.7, 0.2 / 0.7, -0.3 / 0.7], abs=1e-12)
    assert eng.collide(_sphere(0.5, (0.0, 0.0, 0.0)), _sphere(0.3, (0.81, 0.0, 0.0))) == []


@pytest.mark.req("ENG-017")
def test_eng_017() -> None:
    """The engine shall end the run with an error when a contact geometry is neither a primitive nor a convex hull."""
    provider, _ = _ball_on_floor(0.04)
    mesh = eng.Geometry()
    mesh.kind = "mesh"
    contacts = eng.ContactModel()
    contacts.geometries = [mesh]
    with pytest.raises(core.InvalidModelError, match="'mesh'"):
        provider.set_contact_model(contacts)


@pytest.mark.req("ENG-020")
def test_eng_020() -> None:
    """The engine shall model contact as soft contact integrated implicitly, with contact friction."""
    # Ball r 0.05 at z 0.04: depth 0.01, K_n d = 1e5 * 0.01 = 1000 N on the ball, so -1000 N on the world (body a).
    provider, q = _ball_on_floor(0.04)
    found = provider.contacts(q, [0.0] * 6)
    assert len(found) == 1
    assert (found[0].body_a, found[0].body_b) == (-1, 0)
    assert found[0].depth == pytest.approx(0.01, abs=1e-9)
    assert found[0].force[2] == pytest.approx(-1000.0, abs=1e-6)
    # Sliding at 0.2 m/s along x at rest depth: Coulomb, mu lambda = 0.5 * 1000 = 500 N against the slip on the
    # ball, so +500 N along x on the world.
    sliding = provider.contacts(q, [0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
    assert sliding[0].force[0] == pytest.approx(500.0, abs=1e-6)


@pytest.mark.req("ENG-029")
@pytest.mark.xfail(strict=True, reason="stub")
def test_eng_029() -> None:
    """The engine shall run faster than real time for batch runs."""
    raise NotImplementedError


@pytest.mark.req("ENG-035")
def test_eng_035() -> None:
    """The engine shall accept a mid-run change of a payload's mass or attachment at a scheduled time."""
    carrier = eng.Body()
    carrier.name = "carrier"
    carrier.joint.type = eng.JointType.Fixed
    carrier.joint.position = [0.0, 0.0, 1.0]
    payload = eng.Body()
    payload.name = "payload"
    payload.joint.type = eng.JointType.Free
    payload.mass = 2.0
    payload.inertia = [[0.0133, 0.0, 0.0], [0.0, 0.0133, 0.0], [0.0, 0.0, 0.0133]]
    model = eng.MultibodyModel()
    model.bodies = [carrier, payload]
    provider = eng.MultibodyProvider(model)
    q = provider.neutral_positions()
    q[2] = 0.8
    state = eng.State(q, [0.0] * 6)
    integrator = eng.SemiImplicitEuler()
    g = 9.81
    provider.set_attachment(1, 0, True, state.q)
    for _ in range(200):
        provider.step(integrator, state)
    # At rest the weld carries m g = 19.62 N up (1 %).
    held = provider.outputs.payloads[0]
    assert held.attached
    assert held.force[2] == pytest.approx(2.0 * g, rel=0.01)
    # Mass doubled mid-run: inertia doubles with it and the weld carries 4 g.
    provider.set_body_mass(1, 4.0)
    for _ in range(200):
        provider.step(integrator, state)
    assert provider.model.bodies[1].inertia[0][0] == pytest.approx(2.0 * 0.0133, rel=1e-12)
    assert provider.outputs.payloads[0].force[2] == pytest.approx(4.0 * g, rel=0.01)
    # Released: falls g t^2 / 2 = 0.1962 m in 0.2 s (semi-implicit Euler: +0.5 %, h / t); spec 1 %.
    provider.set_attachment(1, 0, False, state.q)
    z0 = state.q[2]
    for _ in range(200):
        provider.step(integrator, state)
    assert z0 - state.q[2] == pytest.approx(0.5 * g * 0.2 * 0.2, rel=0.01)
    released = provider.outputs.payloads[0]
    assert not released.attached
    assert released.carrier == -1
    assert math.isclose(released.force[2], 0.0, abs_tol=0.0)
