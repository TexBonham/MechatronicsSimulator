"""Beam theory on recorded member end-loads (member frame, round 5 rulings 24 and 25) and the deflection from the
recorded node poses (ruling 28), shared by the checks."""

from __future__ import annotations

import math

import numpy as np

from mechsim.integrity.plugin import (
    BodyPoses,
    FloatArray,
    IntegrityError,
    MemberInputs,
    SectionProperties,
    Wrenches,
)

END_TOLERANCE = 1e-9

# First-mode eigenvalue lambda (f = lambda^2 / (2 pi L^2) sqrt(E I / (rho A))) and effective length factor K
# (P_cr = pi^2 E I / (K L)^2) by (start, end) condition; roller counts as pinned for bending.
_LAMBDA = {
    ("clamped", "free"): 1.8751040687119611,
    ("free", "clamped"): 1.8751040687119611,
    ("pinned", "pinned"): math.pi,
    ("clamped", "clamped"): 4.730040744862704,
    ("clamped", "pinned"): 3.926602312047919,
    ("pinned", "clamped"): 3.926602312047919,
}
_K = {
    ("clamped", "free"): 2.0,
    ("free", "clamped"): 2.0,
    ("pinned", "pinned"): 1.0,
    ("clamped", "clamped"): 0.5,
    ("clamped", "pinned"): math.pi / 4.493409457909064,
    ("pinned", "clamped"): math.pi / 4.493409457909064,
}


def axial(loads: Wrenches) -> FloatArray:
    """Internal axial force (rows, 2) at the start and end, tension positive."""
    return np.stack((-loads.start_force[:, 0], loads.end_force[:, 0]), axis=1)


def normal_stress(loads: Wrenches, section: SectionProperties) -> FloatArray:
    """Peak normal stress (rows, 2) at each end: |N| / A + |M_y| c_z / I_y + |M_z| c_y / I_z (ruling 25)."""
    columns = []
    for force, moment in ((loads.start_force, loads.start_moment), (loads.end_force, loads.end_moment)):
        value = np.abs(force[:, 0]) / section.area
        value = value + np.abs(moment[:, 1]) * section.c_z / section.i_y
        value = value + np.abs(moment[:, 2]) * section.c_y / section.i_z
        columns.append(value)
    return np.stack(columns, axis=1)


def shear_stress(loads: Wrenches, section: SectionProperties) -> FloatArray:
    """Average transverse shear stress (rows, 2) at each end: sqrt(V_y^2 + V_z^2) / A."""
    columns = []
    for force in (loads.start_force, loads.end_force):
        columns.append(np.hypot(force[:, 1], force[:, 2]) / section.area)
    return np.stack(columns, axis=1)


def fibre_histories(loads: Wrenches, section: SectionProperties) -> list[FloatArray]:
    """Signed normal stress at the four section corners of each end, N / A +- M_y c_z / I_y +- M_z c_y / I_z: the
    eight histories a cycle count runs on."""
    out: list[FloatArray] = []
    ends = ((-loads.start_force[:, 0], loads.start_moment), (loads.end_force[:, 0], loads.end_moment))
    for normal, moment in ends:
        for sign_y in (1.0, -1.0):
            for sign_z in (1.0, -1.0):
                value = normal / section.area
                value = value + sign_y * moment[:, 1] * section.c_z / section.i_y
                value = value + sign_z * moment[:, 2] * section.c_y / section.i_z
                out.append(value)
    return out


def end_conditions(member: MemberInputs) -> tuple[str, str]:
    """(start, end) condition: the support at 0 and at L (roller as pinned), `free` without one; an interior support
    is an error for the closed forms that need end conditions."""
    length = member.section.length
    start = "free"
    end = "free"
    for at, condition in member.supports:
        kind = condition
        if kind == "roller":
            kind = "pinned"
        if abs(at) <= END_TOLERANCE * length:
            start = kind
        elif abs(at - length) <= END_TOLERANCE * length:
            end = kind
        else:
            raise IntegrityError(f"member {member.name!r}: support at {at} m is interior; no closed form")
    return start, end


def effective_length_factor(member: MemberInputs) -> float:
    conditions = end_conditions(member)
    if conditions not in _K:
        raise IntegrityError(f"member {member.name!r}: no effective length for ends {conditions}")
    return _K[conditions]


def first_frequency(member: MemberInputs) -> float:
    """First bending frequency (Hz) about the weaker axis from the section and the end conditions."""
    conditions = end_conditions(member)
    if conditions not in _LAMBDA:
        raise IntegrityError(f"member {member.name!r}: no natural frequency for ends {conditions}")
    section = member.section
    lam = _LAMBDA[conditions]
    stiffness = section.elastic_modulus * section.i_min() / (section.density * section.area)
    return lam * lam / (2.0 * math.pi * section.length**2) * math.sqrt(stiffness)


def _rotate(q: FloatArray, v: FloatArray) -> FloatArray:
    """Rotate v (rows, 3) by unit quaternions q (rows, 4), w first."""
    w = q[:, :1]
    u = q[:, 1:]
    t = 2.0 * np.cross(u, v)
    result: FloatArray = v + w * t + np.cross(u, t)
    return result


def _multiply(a: FloatArray, b: FloatArray) -> FloatArray:
    aw, ax, ay, az = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    bw, bx, by, bz = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    return np.stack(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        axis=1,
    )


def node_deflection(member: MemberInputs, bodies: BodyPoses) -> FloatArray:
    """Deflection (rows,) from the recorded node poses (round 5 ruling 28, INT-006): the largest perpendicular distance
    of the chain's node origins and its far end (the last node's origin plus l / 2 along its x) from the member's
    reference line. A clamped root's line is the root tangent: the parent's recorded pose times the member placement,
    through the root point along its x (for a cantilever the chord through both ends is not the undeformed axis; a
    clamped-clamped member's far end lies on that line too). A pinned root's line is the chord from the pin (node 0)
    to the far end. Frame-free: a distance, so the member frame of ruling 24 needs no rotation."""
    chain = member.nodes
    if chain is None:
        raise IntegrityError(f"member {member.name!r}: built rigid, no node poses; deflection not evaluated")
    total = bodies.positions.shape[1]
    if chain.first_body + chain.count > total:
        raise IntegrityError(
            f"member {member.name!r}: nodes {chain.first_body} + {chain.count} outside the {total} recorded bodies"
        )
    rows = bodies.positions.shape[0]
    nodes = bodies.positions[:, chain.first_body : chain.first_body + chain.count, :]
    last = bodies.orientations[:, chain.first_body + chain.count - 1, :]
    reach = np.tile(np.asarray([0.5 * chain.segment_length, 0.0, 0.0], dtype=np.float64), (rows, 1))
    tip = nodes[:, -1, :] + _rotate(last, reach)
    unit_x = np.tile(np.asarray([1.0, 0.0, 0.0], dtype=np.float64), (rows, 1))
    if chain.root == "pinned":
        origin = nodes[:, 0, :]
        chord = tip - origin
        length = np.linalg.norm(chord, axis=1)
        if np.any(length <= 0.0):
            raise IntegrityError(f"member {member.name!r}: its end nodes coincide; no chord")
        direction = chord / length[:, None]
    else:
        parent_position = np.zeros((rows, 3), dtype=np.float64)
        parent_orientation = np.tile(np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64), (rows, 1))
        if chain.parent_body is not None:
            parent_position = bodies.positions[:, chain.parent_body, :]
            parent_orientation = bodies.orientations[:, chain.parent_body, :]
        placement = np.tile(np.asarray(chain.position, dtype=np.float64), (rows, 1))
        turn = np.tile(np.asarray(chain.orientation, dtype=np.float64), (rows, 1))
        origin = parent_position + _rotate(parent_orientation, placement)
        direction = _rotate(_multiply(parent_orientation, turn), unit_x)
    points = np.concatenate((nodes, tip[:, None, :]), axis=1) - origin[:, None, :]
    along = np.einsum("rkj,rj->rk", points, direction)
    off = points - along[:, :, None] * direction[:, None, :]
    result: FloatArray = np.max(np.linalg.norm(off, axis=2), axis=1)
    return result


def _conjugate(q: FloatArray) -> FloatArray:
    result: FloatArray = q * np.asarray([1.0, -1.0, -1.0, -1.0], dtype=np.float64)
    return result


def _bending_angle(previous: FloatArray, current: FloatArray) -> FloatArray:
    """Rotation (rows,) of `current` relative to `previous` about their common z (the chain's bending axis): the
    relative quaternion r = conj(previous) current taken in its w >= 0 half, 2 atan2(r_z, r_w)."""
    relative = _multiply(_conjugate(previous), current)
    sign = np.where(relative[:, 0] < 0.0, -1.0, 1.0)
    result: FloatArray = 2.0 * np.arctan2(sign * relative[:, 3], sign * relative[:, 0])
    return result


def node_moments(member: MemberInputs, bodies: BodyPoses) -> tuple[FloatArray, FloatArray]:
    """Bending moment about member z at every node spring from the recorded node poses (INT-005, INT-023): k times the
    rotation of each node body relative to the one before it (the root frame, the parent's pose times the member
    placement, before a clamped root's node 0; a pinned root's node 0 is the spring-free pin segment), k = E I / l the
    chain's node stiffness. Returns the moments (rows, N) in the spring's sense and the springs' positions along the
    member (N,), x_j = (j + 1/2) l."""
    chain = member.nodes
    if chain is None:
        raise IntegrityError(f"member {member.name!r}: built rigid, no node poses; no node moments")
    total = bodies.orientations.shape[1]
    if chain.first_body + chain.count > total:
        raise IntegrityError(
            f"member {member.name!r}: nodes {chain.first_body} + {chain.count} outside the {total} recorded bodies"
        )
    rows = bodies.orientations.shape[0]
    nodes = bodies.orientations[:, chain.first_body : chain.first_body + chain.count, :]
    columns: list[FloatArray] = []
    if chain.root == "pinned":
        previous = nodes[:, 0, :]
        first = 1
    else:
        parent = np.tile(np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64), (rows, 1))
        if chain.parent_body is not None:
            parent = bodies.orientations[:, chain.parent_body, :]
        turn = np.tile(np.asarray(chain.orientation, dtype=np.float64), (rows, 1))
        previous = _multiply(parent, turn)
        first = 0
    for k in range(first, chain.count):
        columns.append(chain.stiffness * _bending_angle(previous, nodes[:, k, :]))
        previous = nodes[:, k, :]
    springs = len(columns)
    where = (np.arange(springs, dtype=np.float64) + 0.5) * chain.segment_length
    if springs == 0:
        return np.zeros((rows, 0), dtype=np.float64), where
    return np.stack(columns, axis=1), where


def member_normal_stress(
    loads: Wrenches, section: SectionProperties, member: MemberInputs, bodies: BodyPoses | None, factor: float = 1.0
) -> FloatArray:
    """Normal stress (rows, 2 + N) along the member (INT-005): the two ends by `normal_stress`, then each node spring
    with abs(N) the larger end value, abs(M_z) its spring moment (`node_moments`) over I_z / c_y, and abs(M_y) the
    out-of-plane moment interpolated linearly between the end moments (the chain is rigid out of its bending plane,
    ruling 65) over I_y / c_z. A member without node poses has its two ends only. Node moments scale with `factor`
    (linear elasticity) as the given loads already do."""
    ends = normal_stress(loads, section)
    if member.nodes is None or bodies is None:
        return ends
    moments, where = node_moments(member, bodies)
    axial = np.maximum(np.abs(loads.start_force[:, 0]), np.abs(loads.end_force[:, 0])) / section.area
    share = where / section.length
    moment_y = np.outer(loads.start_moment[:, 1], 1.0 - share) - np.outer(loads.end_moment[:, 1], share)
    interior = axial[:, None] + np.abs(moment_y) * section.c_z / section.i_y
    interior = interior + np.abs(moments * factor) * section.c_y / section.i_z
    return np.concatenate((ends, interior), axis=1)


def peak(values: FloatArray, times: FloatArray) -> tuple[float, float]:
    """Largest value of a (rows,) or (rows, k) array and the time of its row (the first on a tie); (0, t0) when
    empty."""
    if values.size == 0:
        if times.size == 0:
            return 0.0, 0.0
        return 0.0, float(times[0])
    flat = values.reshape(values.shape[0], -1).max(axis=1)
    index = int(np.argmax(flat))
    return float(flat[index]), float(times[index])


def mount_stress(force: FloatArray, moment: FloatArray, section: SectionProperties) -> FloatArray:
    """Conservative stress (rows,) in the held member's section at a mount from the joint's world-aligned reaction:
    |F| / A + |M| max(c_y, c_z) / min(I_y, I_z), every component treated as the worst axis."""
    fibre = max(section.c_y, section.c_z)
    result: FloatArray = np.linalg.norm(force, axis=1) / section.area + (
        np.linalg.norm(moment, axis=1) * fibre / section.i_min()
    )
    return result
