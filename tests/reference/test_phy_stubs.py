"""Reference tier for the physics adapter spec (PHY): closed forms on the native adapter through the binding
(PHY-022), the pendulum on both adapters; stubs are strict xfail until converted."""

import math
from pathlib import Path
from typing import Any

import pytest

from mechsim import _core as core
from mechsim.physics.mujoco.prepare import prepare_model as prepare_bridge
from mechsim.physics.native.prepare import prepare_model as prepare_native
from mechsim.physics.neutral import PhysicsModel, from_yaml

ROOT = Path(__file__).resolve().parents[2]
PENDULUM = (ROOT / "schemas/examples/physics_model.pendulum.yaml").read_text()
G = 9.81
ORIGIN = ("0 m", "0 m", "0 m")


def loaded(model: PhysicsModel, adapter: str = "native", hz: int = 1000, starts: dict[str, float] | None = None) -> Any:
    physics = core.make_physics_adapter(adapter)
    if adapter == "native":
        native = prepare_native(model, starts, 1.0 / hz)
        physics.load(core.PreparedModel(native.model_json, "", native.version), core.Rate(hz))
    else:
        bridge = prepare_bridge(model, starts, 1.0 / hz)
        physics.load(core.PreparedModel(bridge.neutral_json, bridge.mjcf, bridge.version), core.Rate(hz))
    return physics


def base(gravity: tuple[str, str, str] = ("0 m/s**2", "0 m/s**2", "-9.81 m/s**2")) -> dict[str, Any]:
    data = from_yaml(PENDULUM).model_dump(mode="json")
    data["gravity"] = gravity
    data["bodies"] = []
    data["signals"] = {}
    return data


def block(name: str, mass: float, joint: dict[str, Any], shapes: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    return {
        "name": name,
        "mass": f"{mass} kg",
        "com": ORIGIN,
        "inertia": {"ixx": "0.01 kg*m**2", "iyy": "0.01 kg*m**2", "izz": "0.01 kg*m**2"},
        "material": "steel",
        "parent": "world",
        "joint": joint,
        "shapes": shapes or [],
    }


def crossings(times: list[float], values: list[float]) -> list[float]:
    """Downward zero crossings, linearly interpolated between samples."""
    out: list[float] = []
    for i in range(1, len(values)):
        if values[i - 1] > 0.0 >= values[i]:
            fraction = values[i - 1] / (values[i - 1] - values[i])
            out.append(times[i - 1] + fraction * (times[i] - times[i - 1]))
    return out


@pytest.mark.req("PHY-021")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_021() -> None:
    """Two adapters run on the same neutral model, inputs and seed shall agree on body poses, velocities and forces
    within the differential tolerance.
    """
    raise NotImplementedError


@pytest.mark.req("PHY-022", "PHY-028")
@pytest.mark.parametrize("adapter", ["mujoco", "native"])
def test_phy_028_pendulum_period(adapter: str) -> None:
    """A rigid pendulum run on an adapter shall match the closed-form small-angle period (0.1 percent, PHY-028).

    The schema example: a compound pendulum, I = m L^2 / 3 about the pivot, omega_0 = sqrt(m g (L / 2) / I) = 5.4249
    rad/s, T_0 = 1.1582031 s; zeta = 0.05 gives T_d = T_0 / sqrt(1 - zeta^2) = 1.1596536 s (ACC-033). Released from
    0.05 rad the finite-amplitude correction is theta^2 / 16 = 1.6e-4 relative and the scheme's (omega h)^2 / 3 is
    1e-5 at 1 ms: both inside 1e-3. Measured over the downward zero crossings of the joint coordinate.
    """
    model = from_yaml(PENDULUM)
    physics = loaded(model, adapter, starts={"pivot": 0.05})
    times: list[float] = []
    values: list[float] = []
    for n in range(1, 8001):
        physics.step()
        times.append(n * 1e-3)
        values.append(
            math.atan2(physics.outputs().bodies[0].orientation[3], physics.outputs().bodies[0].orientation[0])
        )
    found = crossings(times, [value + math.pi / 4.0 for value in values])
    period = (found[-1] - found[0]) / (len(found) - 1)
    print(f"{adapter}: pendulum period target 1.1596536 s measured {period:.9f} s")
    assert period == pytest.approx(1.1596536, rel=1e-3)


@pytest.mark.req("PHY-022", "PHY-029")
def test_phy_029() -> None:
    """A body resting on the floor under gravity shall stay at rest on an adapter, with penetration and drift within
    tolerance.

    A 2 kg box (0.1 m edges) on the floor plane, contact K_n = 1e5 N/m: four corners share m g, so the static
    penetration is m g / (4 K_n) = 4.905e-5 m (each corner a spring K_n); drift: the body's horizontal position and
    velocity stay at zero (symmetric corners). Settled after 2 s (C_n = 1e3 N s/m damps the 0.11 kHz bounce).
    """
    data = base()
    shape = {"shape": "box", "size": ("0.1 m", "0.1 m", "0.1 m"), "material": "steel"}
    free = {"name": "drop", "type": "free", "position": ("0 m", "0 m", "0.05 m")}
    data["bodies"] = [block("box", 2.0, free, [shape])]
    data["world_shapes"] = [{"shape": "plane", "material": "steel"}]
    data["contacts"] = {"default": {"stiffness": "1e5 N/m", "damping": "1e3 N*s/m", "friction": 0.5}}
    physics = loaded(PhysicsModel.model_validate(data))
    for _ in range(2000):
        physics.step()
    out = physics.outputs()
    penetration = 0.05 - out.bodies[0].position[2]
    print(f"floor rest: penetration target {2.0 * G / 4e5:.6e} m measured {penetration:.6e} m")
    assert penetration == pytest.approx(2.0 * G / 4e5, rel=1e-3)
    assert abs(out.bodies[0].position[0]) < 1e-9
    assert abs(out.bodies[0].linear_velocity[2]) < 1e-6
    assert sum(contact.force[2] for contact in out.contacts) == pytest.approx(-2.0 * G, rel=1e-3)


@pytest.mark.req("PHY-022", "PHY-030")
def test_phy_030() -> None:
    """A body on a prismatic joint with Coulomb friction shall decelerate at the closed-form rate on an adapter.

    A 3 kg slider, joint friction F = 1.5 N, pushed to speed and released: while sliding, the constant -F gives a
    deceleration F / m = 0.5 m/s^2 exactly per step; it stops within (v / (F / m)) + one step, and stays.
    """
    data = base(("0 m/s**2", "0 m/s**2", "0 m/s**2"))
    joint = {
        "name": "rail",
        "type": "prismatic",
        "position": ORIGIN,
        "axis": (1, 0, 0),
        "linear": [{"friction": "1.5 N"}],
    }
    data["bodies"] = [block("slider", 3.0, joint)]
    physics = loaded(PhysicsModel.model_validate(data))
    for _ in range(10):
        inputs = core.PhysicsInputs()
        inputs.generalized_forces = [301.5]
        physics.set_inputs(inputs)
        physics.step()
    v0 = physics.outputs().bodies[0].linear_velocity[0]
    physics.step()
    v1 = physics.outputs().bodies[0].linear_velocity[0]
    print(f"friction: deceleration target 0.5 m/s^2 measured {(v0 - v1) / 1e-3:.9f} m/s^2")
    assert (v0 - v1) / 1e-3 == pytest.approx(1.5 / 3.0, rel=1e-9)
    for _ in range(int(v1 / 0.5 / 1e-3) + 2):
        physics.step()
    assert abs(physics.outputs().bodies[0].linear_velocity[0]) < 1e-4


@pytest.mark.req("PHY-022", "PHY-007")
@pytest.mark.parametrize("adapter", ["mujoco", "native"])
def test_cantilever_member_end_loads_on_the_native_adapter(adapter: str) -> None:
    """Statics of a lumped cantilever through the adapter (ENG-030, member frame of ruling 24).

    Steel 1 m member, 8 segments, along world x under g = -9.81 z: the clamped stub [0, l / 2] belongs to the parent,
    so the support carries the weight of the rest, W = rho A (L - l / 2) g, along member z, and its moment about
    x = 0, rho A g (L^2 - l^2 / 4) / 2, about member y (magnitude; the sign is the root frame's). Node damping
    settles the first mode (about 8 Hz) within 4 s.
    """
    data = base()
    data["bodies"] = [block("base", 1.0, {"name": "mount", "type": "fixed", "position": ORIGIN})]
    section = {
        "elastic_modulus": "2e11 Pa",
        "second_moment_of_area": "1e-8 m**4",
        "density": "7800 kg/m**3",
        "area": "1e-4 m**2",
        "length": "1 m",
    }
    data["lumped_members"] = [
        {
            "name": "arm",
            "parent": "base",
            "position": ORIGIN,
            "section": section,
            "root": "clamped",
            "end": "free",
            "node_damping": "5 N*m*s/rad",
        }
    ]
    physics = loaded(PhysicsModel.model_validate(data), adapter)
    for _ in range(4000):
        physics.step()
    loads = physics.outputs().members[0]
    span = 1.0 / 8.0
    weight = 7800.0 * 1e-4 * (1.0 - span / 2.0) * G
    moment = 7800.0 * 1e-4 * G * (1.0 - span * span / 4.0) / 2.0
    print(f"cantilever: start force target {weight:.9f} N measured {loads.start.force[2]:.9f} N")
    assert loads.start.force[2] == pytest.approx(weight, rel=1e-6)
    assert abs(loads.start.moment[1]) == pytest.approx(moment, rel=1e-6)
    assert loads.end.force == [0.0, 0.0, 0.0]


@pytest.mark.req("PHY-022", "PHY-004")
def test_rope_pendulum_period_at_50_microseconds() -> None:
    """A 1 kg load on a 1 m rope from the world, released at 0.05 rad, at the 50 microsecond step (ruling 21).

    Closed form T = 2 pi sqrt(L / g) = 2.006 s. Departures, all relative: rope mass 1e-4 kg against 1 kg (< 1e-4),
    stretch m g / EA = 1e-5, amplitude theta^2 / 16 = 1.6e-4, backward Euler (omega h)^2 / 3 = 8e-9: under 1e-3.
    """
    data = base()
    angle = 0.05
    position = (f"{math.sin(angle)} m", "0 m", f"{-math.cos(angle)} m")
    data["bodies"] = [block("load", 1.0, {"name": "swing", "type": "free", "position": position})]
    data["ropes"] = [
        {
            "name": "line",
            "from_body": "world",
            "from_point": ORIGIN,
            "to_body": "load",
            "to_point": ORIGIN,
            "segments": 8,
            "axial_stiffness": "1e6 N",
            "damping": "0 N*s",
            "mass_per_length": "1e-4 kg/m",
            "rest_length": f"{1.0 - 2.0 * G / 1e6 / 2.0} m",
        }
    ]
    physics = loaded(PhysicsModel.model_validate(data), hz=20000)
    times: list[float] = []
    values: list[float] = []
    for n in range(1, 90001):
        physics.step()
        if n % 10 == 0:
            times.append(n * 5e-5)
            values.append(physics.outputs().bodies[0].position[0])
    found = crossings(times, values)
    period = (found[-1] - found[0]) / (len(found) - 1)
    target = 2.0 * math.pi * math.sqrt(1.0 / G)
    print(f"rope pendulum: period target {target:.9f} s measured {period:.9f} s")
    assert period == pytest.approx(target, rel=1e-3)
    assert physics.outputs().ropes[0].tension > 0.0
