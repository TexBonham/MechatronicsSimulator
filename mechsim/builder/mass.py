"""Mass properties of primitive solids from geometry and density (BLD-010), and the frame algebra the builder needs.

Primitives follow the design and neutral conventions: box `size` is the full edge lengths, cylinder and capsule lie
along the shape's z axis, a capsule's `length` is the cylindrical part between its cap centers. The inertia tensor
holds tensor entries (off-diagonals are minus the products of inertia), about the center of mass, in body axes.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

Vector = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Matrix = tuple[Vector, Vector, Vector]

IDENTITY: Quaternion = (1.0, 0.0, 0.0, 0.0)
ORIGIN: Vector = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class MassProperties:
    """Mass (kg), center of mass (m) and the inertia tensor about it (kg m^2), all in one frame."""

    mass: float
    com: Vector
    inertia: Matrix


def rotation(q: Sequence[float]) -> Matrix:
    """Rotation matrix of a w-first quaternion, normalized first."""
    norm = math.sqrt(sum(component * component for component in q))
    w, x, y, z = (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )


def apply(matrix: Matrix, vector: Sequence[float]) -> Vector:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return (
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    )


def compose(
    parent: tuple[Vector, Quaternion], position: Sequence[float], orientation: Sequence[float]
) -> tuple[Vector, Quaternion]:
    """The pose of a frame placed at `position`, `orientation` in the frame `parent`, in parent's reference frame."""
    offset = apply(rotation(parent[1]), position)
    child = (orientation[0], orientation[1], orientation[2], orientation[3])
    return (
        (parent[0][0] + offset[0], parent[0][1] + offset[1], parent[0][2] + offset[2]),
        multiply(parent[1], child),
    )


def _diagonal(ixx: float, iyy: float, izz: float) -> Matrix:
    return ((ixx, 0.0, 0.0), (0.0, iyy, 0.0), (0.0, 0.0, izz))


def box(density: float, size: Sequence[float]) -> MassProperties:
    a, b, c = size[0], size[1], size[2]
    mass = density * a * b * c
    return MassProperties(
        mass,
        ORIGIN,
        _diagonal(mass * (b * b + c * c) / 12.0, mass * (a * a + c * c) / 12.0, mass * (a * a + b * b) / 12.0),
    )


def cylinder(density: float, radius: float, length: float) -> MassProperties:
    mass = density * math.pi * radius * radius * length
    transverse = mass * (3.0 * radius * radius + length * length) / 12.0
    return MassProperties(mass, ORIGIN, _diagonal(transverse, transverse, mass * radius * radius / 2.0))


def sphere(density: float, radius: float) -> MassProperties:
    mass = density * 4.0 / 3.0 * math.pi * radius**3
    moment = 2.0 / 5.0 * mass * radius * radius
    return MassProperties(mass, ORIGIN, _diagonal(moment, moment, moment))


def capsule(density: float, radius: float, length: float) -> MassProperties:
    """Cylinder of `length` plus two hemispherical caps. A hemisphere of mass m_h has its center of mass 3 r / 8 from
    its flat face and a transverse moment 83/320 m_h r^2 about it; its axial moment is 2/5 m_h r^2."""
    body = cylinder(density, radius, length)
    cap_mass = density * 2.0 / 3.0 * math.pi * radius**3
    cap_offset = length / 2.0 + 3.0 * radius / 8.0
    cap_transverse = 83.0 / 320.0 * cap_mass * radius * radius + cap_mass * cap_offset * cap_offset
    transverse = body.inertia[0][0] + 2.0 * cap_transverse
    axial = body.inertia[2][2] + 2.0 * (2.0 / 5.0 * cap_mass * radius * radius)
    return MassProperties(body.mass + 2.0 * cap_mass, ORIGIN, _diagonal(transverse, transverse, axial))


def place(local: MassProperties, position: Sequence[float], orientation: Sequence[float]) -> MassProperties:
    """`local` (about its own center of mass, in shape axes) expressed in the frame the shape is placed in."""
    matrix = rotation(orientation)
    com = apply(matrix, local.com)
    rotated = tuple(
        tuple(
            sum(matrix[i][k] * local.inertia[k][m] * matrix[j][m] for k in range(3) for m in range(3)) for j in range(3)
        )
        for i in range(3)
    )
    return MassProperties(
        local.mass,
        (com[0] + position[0], com[1] + position[1], com[2] + position[2]),
        (
            (rotated[0][0], rotated[0][1], rotated[0][2]),
            (rotated[1][0], rotated[1][1], rotated[1][2]),
            (rotated[2][0], rotated[2][1], rotated[2][2]),
        ),
    )


def combine(parts: Sequence[MassProperties]) -> MassProperties:
    """Sum of rigidly joined parts, each about its own center of mass; parallel-axis shift to the joint center."""
    mass = sum(part.mass for part in parts)
    com = (
        sum(part.mass * part.com[0] for part in parts) / mass,
        sum(part.mass * part.com[1] for part in parts) / mass,
        sum(part.mass * part.com[2] for part in parts) / mass,
    )
    rows: list[list[float]] = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    for part in parts:
        d = (part.com[0] - com[0], part.com[1] - com[1], part.com[2] - com[2])
        squared = d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
        for i in range(3):
            for j in range(3):
                shift = -part.mass * d[i] * d[j]
                if i == j:
                    shift += part.mass * squared
                rows[i][j] += part.inertia[i][j] + shift
    return MassProperties(
        mass,
        com,
        (
            (rows[0][0], rows[0][1], rows[0][2]),
            (rows[1][0], rows[1][1], rows[1][2]),
            (rows[2][0], rows[2][1], rows[2][2]),
        ),
    )


def half_extents(shape: str, size: Sequence[float], radius: float, length: float) -> Vector:
    """Half extents of a primitive's bounding box in its own axes."""
    if shape == "box":
        return (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
    if shape == "sphere":
        return (radius, radius, radius)
    if shape == "cylinder":
        return (radius, radius, length / 2.0)
    return (radius, radius, length / 2.0 + radius)


def world_box(pose: tuple[Vector, Quaternion], extents: Vector) -> tuple[Vector, Vector]:
    """Axis-aligned bounds of a box of half `extents` at `pose`: |R| e about the center."""
    matrix = rotation(pose[1])
    reach = tuple(sum(abs(matrix[i][j]) * extents[j] for j in range(3)) for i in range(3))
    center = pose[0]
    return (
        (center[0] - reach[0], center[1] - reach[1], center[2] - reach[2]),
        (center[0] + reach[0], center[1] + reach[1], center[2] + reach[2]),
    )


def z_to(normal: Sequence[float]) -> Quaternion:
    """Shortest rotation taking +z to the unit `normal`: q = normalize(1 + z . n, z x n); +z to -z turns about x."""
    w = 1.0 + normal[2]
    if w <= 1e-12:
        return (0.0, 1.0, 0.0, 0.0)
    x, y = 0.0 - normal[1], normal[0] + 0.0
    norm = math.sqrt(w * w + x * x + y * y)
    return (w / norm, x / norm, y / norm, 0.0)
