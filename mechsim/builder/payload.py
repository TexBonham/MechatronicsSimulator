"""The case payload as a neutral body (BLD-036): a free body under world welded to its carrier by the adapter.

`with_payload(model, name, carrier, mass)` appends body `name`: mass `mass`, a placeholder solid box of edge
`PAYLOAD_EDGE` (inertia m a^2 / 6 about each axis), on a free joint `<name>_free` under world, its center hung below the
carrier's
reference-configuration origin along gravity by the carrier's shape extent (the largest |shape position| plus the
shape's half diagonal) plus half the box and `PAYLOAD_CLEARANCE`, so the two never overlap. The run's payload argument
maps the body to its carrier (`prepare_model(..., payloads=)`).
"""

from __future__ import annotations

import json
import math
from collections.abc import Sequence

from mechsim.builder.result import BuildError
from mechsim.physics.neutral import Body, PhysicsModel

PAYLOAD_EDGE = 0.1
PAYLOAD_CLEARANCE = 0.01


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


def _origin(model: PhysicsModel, name: str) -> tuple[float, float, float]:
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
        poses[body.name] = (position, orientation)
    return poses[name][0]


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


def with_payload(model: PhysicsModel, name: str, carrier: str, mass: float) -> PhysicsModel:
    bodies = {body.name: body for body in model.bodies}
    if name in bodies:
        raise BuildError([f"payload '{name}': the model already has a body of that name"])
    if carrier not in bodies:
        raise BuildError([f"payload '{name}': unknown carrier body '{carrier}'"])
    if not mass > 0.0:
        raise BuildError([f"payload '{name}': mass must be positive, got {mass}"])
    gravity = [float(value) for value in model.gravity]
    norm = math.sqrt(sum(value * value for value in gravity))
    down = (0.0, 0.0, -1.0)
    if norm > 0.0:
        down = (gravity[0] / norm, gravity[1] / norm, gravity[2] / norm)
    origin = _origin(model, carrier)
    drop = _extent(bodies[carrier]) + PAYLOAD_EDGE / 2.0 + PAYLOAD_CLEARANCE
    position = [origin[index] + drop * down[index] for index in range(3)]
    material = "payload"
    for shape in bodies[carrier].shapes:
        material = shape.material
        break
    inertia = mass * PAYLOAD_EDGE * PAYLOAD_EDGE / 6.0
    body = {
        "name": name,
        "mass": f"{mass!r} kg",
        "com": ["0.0 m", "0.0 m", "0.0 m"],
        "inertia": {"ixx": f"{inertia!r} kg*m**2", "iyy": f"{inertia!r} kg*m**2", "izz": f"{inertia!r} kg*m**2"},
        "material": bodies[carrier].material,
        "parent": "world",
        "joint": {"name": f"{name}_free", "type": "free", "position": [f"{value!r} m" for value in position]},
        "shapes": [
            {"shape": "box", "size": [f"{PAYLOAD_EDGE!r} m"] * 3, "material": material},
        ],
    }
    data = json.loads(model.model_dump_json())
    data["bodies"] = [*data["bodies"], body]
    return PhysicsModel.model_validate(data)
