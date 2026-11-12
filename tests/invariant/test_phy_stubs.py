"""Invariant tier stubs for the physics adapter spec (PHY), strict xfail until converted."""

import math
import re
import struct
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest

from mechsim import _core as core
from mechsim.physics.mujoco.prepare import prepare_model as prepare_bridge
from mechsim.physics.native.prepare import prepare_model as prepare_native
from mechsim.physics.neutral import PhysicsModel, from_yaml

PENDULUM = (Path(__file__).resolve().parents[2] / "schemas/examples/physics_model.pendulum.yaml").read_text()
ORIGIN = ("0 m", "0 m", "0 m")
INERTIA = {"ixx": "0.001 kg*m**2", "iyy": "0.001 kg*m**2", "izz": "0.001 kg*m**2"}


def _body(name: str, mass: str, joint: dict[str, Any], shapes: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    return {
        "name": name,
        "mass": mass,
        "com": ORIGIN,
        "inertia": INERTIA,
        "material": "steel",
        "parent": "world",
        "joint": joint,
        "shapes": shapes or [],
    }


def _base() -> dict[str, Any]:
    data = from_yaml(PENDULUM).model_dump(mode="json")
    data["gravity"] = ("0 m/s**2", "0 m/s**2", "-9.81 m/s**2")
    data["signals"] = {}
    return data


def driven() -> PhysicsModel:
    """A rotor driving a carriage through a compliant belt under a varying shaft torque."""
    data = _base()
    data["bodies"] = [
        _body("rotor", "0.1 kg", {"name": "spindle", "type": "revolute", "position": ORIGIN, "axis": (0, 0, 1)}),
        _body("carriage", "1 kg", {"name": "slide", "type": "prismatic", "position": ORIGIN, "axis": (1, 0, 0)}),
    ]
    data["transmissions"] = [
        {
            "name": "belt",
            "type": "belt_pulley",
            "shaft_joint": "spindle",
            "driven_joint": "slide",
            "efficiency": 0.9,
            "pulley_radius": "0.02 m",
            "stiffness": "1e5 N/m",
        }
    ]
    return PhysicsModel.model_validate(data)


def contact() -> PhysicsModel:
    """A box dropped onto the floor from 1 cm, tilted, so corners land one after another and friction acts."""
    data = _base()
    shape = {"shape": "box", "size": ("0.1 m", "0.1 m", "0.1 m"), "material": "steel"}
    half = 0.03
    turn = (math.cos(half), 0.8 * math.sin(half), 0.6 * math.sin(half), 0.0)
    joint = {"name": "drop", "type": "free", "position": ("0 m", "0 m", "0.06 m"), "orientation": turn}
    data["bodies"] = [_body("box", "2 kg", joint, [shape])]
    data["world_shapes"] = [{"shape": "plane", "material": "steel"}]
    data["contacts"] = {"default": {"stiffness": "1e5 N/m", "damping": "1e3 N*s/m", "friction": 0.5}}
    return PhysicsModel.model_validate(data)


def rope() -> PhysicsModel:
    """A load released at 0.2 rad on an unstretched rope: the first stretch overshoots towards 2 m g = 19.6 N and the
    rope snaps at 14 N."""
    data = _base()
    swing = 0.2
    start = (f"{math.sin(swing)} m", "0 m", f"{-math.cos(swing)} m")
    data["bodies"] = [_body("load", "1 kg", {"name": "hang", "type": "free", "position": start})]
    data["ropes"] = [
        {
            "name": "line",
            "from_body": "world",
            "from_point": ORIGIN,
            "to_body": "load",
            "to_point": ORIGIN,
            "segments": 4,
            "axial_stiffness": "1e5 N",
            "damping": "20 N*s",
            "mass_per_length": "0.01 kg/m",
            "rest_length": "1 m",
            "breaking_tension": "14 N",
        }
    ]
    return PhysicsModel.model_validate(data)


def member() -> PhysicsModel:
    """A cantilever on a fixed base, released from its straight start under gravity: the chain swings and its
    end-loads and node poses change every step."""
    data = _base()
    data["bodies"] = [_body("base", "1 kg", {"name": "mount", "type": "fixed", "position": ORIGIN})]
    section = {
        "elastic_modulus": "2e11 Pa",
        "second_moment_of_area": "1e-8 m**4",
        "density": "7800 kg/m**3",
        "area": "1e-4 m**2",
        "length": "1 m",
    }
    data["lumped_members"] = [
        {"name": "arm", "parent": "base", "position": ORIGIN, "section": section, "root": "clamped", "end": "free"}
    ]
    return PhysicsModel.model_validate(data)


def _load(model: PhysicsModel, adapter: str) -> Any:
    physics = core.make_physics_adapter(adapter)
    if adapter == "native":
        native = prepare_native(model, None, 1e-3)
        physics.load(core.PreparedModel(native.model_json, "", native.version), core.Rate(1000))
    else:
        bridge = prepare_bridge(model, None, 1e-3)
        physics.load(core.PreparedModel(bridge.neutral_json, bridge.mjcf, bridge.version), core.Rate(1000))
    return physics


def _drive(physics: Any, n: int) -> None:
    inputs = core.PhysicsInputs()
    inputs.generalized_forces = [0.002 * math.sin(n / 40.0), 0.0]
    physics.set_inputs(inputs)


def _joint_drive(physics: Any, n: int) -> None:
    """A real joint drive on the belt's shaft toward a moving rest (PHY-034), plus a small varying shaft torque."""
    inputs = core.PhysicsInputs()
    inputs.generalized_forces = [0.001 * math.sin(n / 30.0), 0.0]
    inputs.joint_drives = [core.JointDrive(0, 0.5 * math.sin(n / 50.0), 1.0, 0.05, n % 97 != 0)]
    physics.set_inputs(inputs)


def _no_inputs(physics: Any, n: int) -> None:
    del physics, n


def _bits(values: list[float]) -> bytes:
    return struct.pack(f"<{len(values)}d", *values)


def _record(out: Any) -> bytes:
    values: list[float] = [float(out.steps)]
    for body in out.bodies:
        values += [*body.position, *body.orientation, *body.angular_velocity, *body.linear_velocity]
    for joint in out.joints:
        values += [*joint.generalized_force, *joint.reaction.moment, *joint.reaction.force]
    for item in out.contacts:
        values += [float(item.body_a), float(item.body_b), *item.point, *item.force]
    for item in out.members:
        values += [*item.start.moment, *item.start.force, *item.end.moment, *item.end.force]
    for item in out.ropes:
        values += [item.tension, item.max_tension, float(item.slack)]
    for item in out.transmissions:
        values += [item.slip, item.shaft_load_torque, item.force, item.stretch]
    for item in out.model_changes:
        values += [float(int(item.kind)), float(item.index), item.value]
    values += list(out.joint_positions)
    values += [item.torque for item in out.joint_drive_torques]
    return _bits(values)


def _run(model: PhysicsModel, adapter: str, inputs: Callable[[Any, int], None], steps: int) -> list[bytes]:
    physics = _load(model, adapter)
    rows: list[bytes] = []
    for n in range(steps):
        inputs(physics, n)
        physics.step()
        rows.append(_record(physics.outputs()))
    return rows


CASES = [
    ("driven", driven, _drive, "mujoco"),
    ("driven", driven, _drive, "native"),
    ("joint-drive", driven, _joint_drive, "mujoco"),
    ("joint-drive", driven, _joint_drive, "native"),
    ("member", member, _no_inputs, "mujoco"),
    ("member", member, _no_inputs, "native"),
    ("contact", contact, _no_inputs, "mujoco"),
    ("contact", contact, _no_inputs, "native"),
    ("rope", rope, _no_inputs, "native"),
]


@pytest.mark.req("PHY-010")
@pytest.mark.parametrize(("name", "model", "inputs", "adapter"), CASES, ids=[f"{c[0]}-{c[3]}" for c in CASES])
def test_phy_010_run_twice_bitwise(
    name: str, model: Callable[[], PhysicsModel], inputs: Callable[[Any, int], None], adapter: str
) -> None:
    """The same neutral model and inputs run twice on one adapter give identical outputs at every step, bit for bit:
    a driven transmission (a varying shaft torque through a compliant belt), a real joint drive on that shaft
    toward a moving rest (disabled every 97th tick), a lumped cantilever (end-loads and node poses), a contact case (a tilted box landing on the floor,
    friction and per-corner contacts) and a rope case (a swinging load whose rope snaps; the bridge maps no ropes)."""
    first = _run(model(), adapter, inputs, 600)
    second = _run(model(), adapter, inputs, 600)
    assert len(first) == len(second) == 600
    for step, (a, b) in enumerate(zip(first, second, strict=True)):
        assert a == b, f"{name} on {adapter}: step {step + 1} differs"


@pytest.mark.req("PHY-026")
@pytest.mark.xfail(strict=True, reason="stub")
def test_phy_026() -> None:
    """After retirement, no runtime code path shall import MuJoCo."""
    raise NotImplementedError


ROOT = Path(__file__).resolve().parents[2]
_MUJOCO_INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]mujoco/', re.MULTILINE)


@pytest.mark.req("PHY-011")
def test_phy_011_mujoco_headers_only_in_the_cpp_adapter() -> None:
    """C++ side of PHY-011 (ADR 0014); the Python side is the import-boundary checker's test."""
    allowed = ROOT / "core" / "physics" / "mujoco"
    offenders = []
    for path in sorted((ROOT / "core").rglob("*")):
        if path.suffix not in {".c", ".cc", ".cpp", ".h", ".hpp"} or allowed in path.parents:
            continue
        if _MUJOCO_INCLUDE.search(path.read_text(encoding="utf-8", errors="replace")):
            offenders.append(path.relative_to(ROOT).as_posix())
    assert offenders == []
    assert _MUJOCO_INCLUDE.search((allowed / "src" / "mujoco_adapter.cpp").read_text(encoding="utf-8"))
