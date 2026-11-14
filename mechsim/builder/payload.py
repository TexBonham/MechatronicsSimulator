"""The case payload as a neutral body (BLD-036, ruling 24).

`with_payload(model, name, carrier, mass, fixed=False)` appends body `name`: mass `mass`, a placeholder solid box of
edge `PAYLOAD_EDGE` (inertia m a^2 / 6 about each axis), its center hung below the carrier's reference-configuration
origin along gravity by the carrier's shape extent (the largest |shape position| plus the shape's half diagonal) plus
half the box and `PAYLOAD_CLEARANCE`, so the two never overlap. `fixed=True` (a payload attached at t = 0 that is never
released, ruling 24) makes it a child of the carrier on a fixed joint `<name>_fixed`, placed in the carrier's frame
(the world drop rotated by the inverse of the carrier's reference orientation): rigid, no equality, and a load cell
reads that joint's reaction. Otherwise it is a free body on `<name>_free` under world and the run's payload argument
welds it to its carrier (`prepare_model(..., payloads=)`), for a mid-run attach or release.

`rope` (a design rope ending at this payload, `BuildResult.payload_ropes`, BLD-032) hangs the payload on it instead: a
shackle body `<name>_shackle` (mass `fitting_mass`, the rope part's `end_fitting_mass`, LIB-020; no shapes) on the fixed
joint `<name>_fixed` at the rope's `from_point` of its carrier (the rope's `from_body`), so a load cell reading the
payload's joint reaction reads the rope's top; the payload a free body on `<name>_free` under world, its top face centre
plus the rope's `to_point` the rope's rest length below the shackle along gravity with the carrier at the run's start
coordinates (`starts`, joint name to coordinate: the engine builds the rope between its end points at the start
configuration), so the rope starts straight at rest length; the rope from the shackle to that point. The run never welds
a rope payload.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence

from mechsim.builder.result import BuildError
from mechsim.physics.neutral import Body, PhysicsModel, Rope

PAYLOAD_EDGE = 0.1
PAYLOAD_CLEARANCE = 0.01
SHACKLE_EDGE = 0.01


def _q_rot(q: Sequence[float], v: Sequence[float]) -> tuple[float, float, float]:
    w, x, y, z = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


def _q_mul(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _pose(
    model: PhysicsModel, name: str, starts: Mapping[str, float] | None = None
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """World pose of body `name` with every joint at zero, or at its `starts` coordinate (prismatic along, revolute
    about its axis in the joint frame)."""
    given = dict(starts or {})
    poses: dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]] = {}
    for body in model.bodies:
        joint = body.joint
        position: tuple[float, float, float] = (0.0, 0.0, 0.0)
        orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        if body.parent != "world":
            position, orientation = poses[body.parent]
        if joint is not None:
            offset = _q_rot(orientation, joint.position)
            position = (position[0] + offset[0], position[1] + offset[1], position[2] + offset[2])
            orientation = _q_mul(orientation, joint.orientation)
            value = given.get(joint.name, 0.0)
            axis = joint.axis
            if joint.type == "prismatic" and value != 0.0:
                step = _q_rot(orientation, (axis[0] * value, axis[1] * value, axis[2] * value))
                position = (position[0] + step[0], position[1] + step[1], position[2] + step[2])
            if joint.type == "revolute" and value != 0.0:
                half = value / 2.0
                turn = (math.cos(half), axis[0] * math.sin(half), axis[1] * math.sin(half), axis[2] * math.sin(half))
                orientation = _q_mul(orientation, turn)
        poses[body.name] = (position, orientation)
    return poses[name]


def _extent(body: Body) -> float:
    extent = 0.0
    for shape in body.shapes:
        reach = math.sqrt(sum(value * value for value in shape.position))
        half = 0.0
        if shape.shape == "box":
            half = math.sqrt(sum(value * value for value in shape.size)) / 2.0
        elif shape.shape == "sphere":
            half = shape.radius
        elif shape.shape in ("cylinder", "capsule"):
            half = math.sqrt(shape.radius**2 + (shape.length / 2.0) ** 2)
            if shape.shape == "capsule":
                half = half + shape.radius
        extent = max(extent, reach + half)
    return extent


def with_payload(
    model: PhysicsModel,
    name: str,
    carrier: str,
    mass: float,
    fixed: bool = False,
    rope: Rope | None = None,
    starts: Mapping[str, float] | None = None,
    fitting_mass: float | None = None,
) -> PhysicsModel:
    bodies = {body.name: body for body in model.bodies}
    if name in bodies:
        raise BuildError([f"payload '{name}': the model already has a body of that name"])
    if carrier not in bodies:
        raise BuildError([f"payload '{name}': unknown carrier body '{carrier}'"])
    if not mass > 0.0:
        raise BuildError([f"payload '{name}': mass must be positive, got {mass}"])
    if rope is not None:
        if fitting_mass is None or not fitting_mass > 0.0:
            raise BuildError(
                [f"payload '{name}': rope '{rope.name}' needs a positive end fitting mass, got {fitting_mass}"]
            )
        return _on_rope(model, name, carrier, mass, rope, starts, fitting_mass)
    down = _down(model)
    origin, orientation = _pose(model, carrier)
    drop = _extent(bodies[carrier]) + PAYLOAD_EDGE / 2.0 + PAYLOAD_CLEARANCE
    position = [origin[index] + drop * down[index] for index in range(3)]
    parent = "world"
    joint = {"name": f"{name}_free", "type": "free"}
    if fixed:
        inverse = (orientation[0], -orientation[1], -orientation[2], -orientation[3])
        position = list(_q_rot(inverse, (drop * down[0], drop * down[1], drop * down[2])))
        parent = carrier
        joint = {"name": f"{name}_fixed", "type": "fixed"}
    body = _box(name, mass, bodies[carrier], parent, joint, position)
    data = json.loads(model.model_dump_json())
    data["bodies"] = [*data["bodies"], body]
    return PhysicsModel.model_validate(data)


def _down(model: PhysicsModel) -> tuple[float, float, float]:
    gravity = [float(value) for value in model.gravity]
    norm = math.sqrt(sum(value * value for value in gravity))
    if norm > 0.0:
        return (gravity[0] / norm, gravity[1] / norm, gravity[2] / norm)
    return (0.0, 0.0, -1.0)


def _on_rope(
    model: PhysicsModel,
    name: str,
    carrier: str,
    mass: float,
    rope: Rope,
    starts: Mapping[str, float] | None,
    fitting_mass: float,
) -> PhysicsModel:
    if rope.to_body != name:
        raise BuildError([f"payload '{name}': rope '{rope.name}' ends at '{rope.to_body}', not at this payload"])
    if rope.from_body != carrier:
        raise BuildError(
            [f"payload '{name}': carried by '{carrier}', but rope '{rope.name}' hangs from '{rope.from_body}'"]
        )
    bodies = {body.name: body for body in model.bodies}
    shackle = f"{name}_shackle"
    if shackle in bodies:
        raise BuildError([f"payload '{name}': the model already has a body named '{shackle}'"])
    down = _down(model)
    origin, orientation = _pose(model, carrier, starts)
    anchor = _q_rot(orientation, rope.from_point)
    top = tuple(-PAYLOAD_EDGE / 2.0 * down[index] + float(rope.to_point[index]) for index in range(3))
    centre = [origin[index] + anchor[index] + rope.rest_length * down[index] - top[index] for index in range(3)]
    inertia = fitting_mass * SHACKLE_EDGE * SHACKLE_EDGE / 6.0
    fitting = {
        "name": shackle,
        "mass": f"{fitting_mass!r} kg",
        "com": ["0.0 m", "0.0 m", "0.0 m"],
        "inertia": {"ixx": f"{inertia!r} kg*m**2", "iyy": f"{inertia!r} kg*m**2", "izz": f"{inertia!r} kg*m**2"},
        "material": bodies[carrier].material,
        "parent": carrier,
        "joint": {"name": f"{name}_fixed", "type": "fixed", "position": [f"{float(v)!r} m" for v in rope.from_point]},
    }
    body = _box(name, mass, bodies[carrier], "world", {"name": f"{name}_free", "type": "free"}, centre)
    data = json.loads(model.model_dump_json())
    data["bodies"] = [*data["bodies"], fitting, body]
    hung = json.loads(rope.model_dump_json())
    hung["from_body"] = shackle
    hung["from_point"] = ["0.0 m", "0.0 m", "0.0 m"]
    hung["to_point"] = [f"{value!r} m" for value in top]
    data["ropes"] = [*data["ropes"], hung]
    return PhysicsModel.model_validate(data)


def _box(
    name: str, mass: float, carrier: Body, parent: str, joint: dict[str, str], position: Sequence[float]
) -> dict[str, object]:
    material = "payload"
    for shape in carrier.shapes:
        material = shape.material
        break
    inertia = mass * PAYLOAD_EDGE * PAYLOAD_EDGE / 6.0
    return {
        "name": name,
        "mass": f"{mass!r} kg",
        "com": ["0.0 m", "0.0 m", "0.0 m"],
        "inertia": {"ixx": f"{inertia!r} kg*m**2", "iyy": f"{inertia!r} kg*m**2", "izz": f"{inertia!r} kg*m**2"},
        "material": carrier.material,
        "parent": parent,
        "joint": {**joint, "position": [f"{value!r} m" for value in position]},
        "shapes": [
            {"shape": "box", "size": [f"{PAYLOAD_EDGE!r} m"] * 3, "material": material},
        ],
    }
