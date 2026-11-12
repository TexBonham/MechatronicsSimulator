"""ENG stubs, reference tier (T3); Catch2 tests replace them with the C++ code."""

import math
from typing import Any

import pytest

from mechsim.physics.native import _engine as eng


def _geometry(kind: str, position: tuple[float, float, float], body: int = -1) -> Any:
    g = eng.Geometry()
    g.kind = kind
    g.position = position
    g.body = body
    g.material = "steel"
    return g


@pytest.mark.req("ENG-019")
def test_eng_019() -> None:
    """The engine's collision detection shall agree with the closed-form contact set on contact existence, normal and
    penetration depth (ruling 23).
    """
    # Oracle: the closed-form contact set (ruling 23). Tolerance
    # (proposed): 1e-9 m depth, 1e-6 rad normal.
    # Sphere r 0.2 at z 0.15 over the floor: depth 0.05, normal sphere to plane -z.
    sphere = _geometry("sphere", (1.0, -2.0, 0.15))
    sphere.radius = 0.2
    found = eng.collide(sphere, _geometry("plane", (0.0, 0.0, 0.0)))
    print(f"sphere-plane: depth target 0.05, measured {found[0].depth!r}")
    assert len(found) == 1
    assert abs(found[0].depth - 0.05) <= 1e-9
    assert math.acos(min(1.0, -found[0].normal[2])) <= 1e-6
    # Box size (0.2, 0.4, 0.6) at z 0.25: its four bottom corners at z -0.05, depth 0.05 each.
    box = _geometry("box", (0.0, 0.0, 0.25))
    box.size = (0.2, 0.4, 0.6)
    found = eng.collide(box, _geometry("plane", (0.0, 0.0, 0.0)))
    assert len(found) == 4
    for c in found:
        assert abs(c.depth - 0.05) <= 1e-9


@pytest.mark.req("ENG-021")
def test_eng_021() -> None:
    """The engine shall bring a body dropped onto a plane to rest with steady penetration within tolerance, and shall
    stop a body driven into a fixed hard stop.
    """
    # 1 kg sphere r 0.05 dropped 0.1 m onto the floor, K_n 1e5 N/m, C_n 1e3 N s/m. At rest the damper carries
    # nothing, so K_n d = m g: d = 9.81e-5 m. zeta = C / (2 sqrt(K m)) = 1.58, slow decay root 112.7 1/s: the
    # transient is e^-112 of itself after 1 s. Spec (proposed): within 1 % of m g / K_n.
    body = eng.Body()
    body.name = "ball"
    body.joint.type = eng.JointType.Free
    body.mass = 1.0
    body.inertia = [[1e-3, 0.0, 0.0], [0.0, 1e-3, 0.0], [0.0, 0.0, 1e-3]]
    model = eng.MultibodyModel()
    model.bodies = [body]
    provider = eng.MultibodyProvider(model)
    ball = _geometry("sphere", (0.0, 0.0, 0.0), body=0)
    ball.radius = 0.05
    contacts = eng.ContactModel()
    contacts.geometries = [_geometry("plane", (0.0, 0.0, 0.0)), ball]
    contacts.default_parameters.stiffness = 1e5
    contacts.default_parameters.damping = 1e3
    provider.set_contact_model(contacts)
    q = provider.neutral_positions()
    q[2] = 0.15
    state = eng.State(q, [0.0] * 6)
    integrator = eng.SemiImplicitEuler()
    for _ in range(1000):
        provider.step(integrator, state)
    target = 9.81 / 1e5
    penetration = 0.05 - state.q[2]
    print(f"rest penetration: target {target:.6e} m, measured {penetration:.6e} m")
    assert abs(penetration - target) <= 0.01 * target
