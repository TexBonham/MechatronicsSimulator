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


def node_moments(
    member: MemberInputs, bodies: BodyPoses, offsets: FloatArray | None = None
) -> tuple[FloatArray, FloatArray]:
    """Bending moment about member z at every node spring from the recorded node poses (INT-005, INT-023): k times the
    rotation of each node body relative to the one before it (the root frame, the parent's pose times the member
    placement, before a clamped root's node 0; a pinned root's node 0 is the spring-free pin segment), k = E I / l the
    chain's node stiffness. `offsets` (rows, N), rad: each spring's permanent rest offset in effect at the row (the
    tier 2 yield, INT-028, ruling 35); the spring carries k times its elastic rotation, the total minus the offset.
    Returns the moments (rows, N) in the spring's sense and the springs' positions along the member (N,),
    x_j = (j + 1/2) l."""
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
        columns.append(_bending_angle(previous, nodes[:, k, :]))
        previous = nodes[:, k, :]
    springs = len(columns)
    where = (np.arange(springs, dtype=np.float64) + 0.5) * chain.segment_length
    if springs == 0:
        return np.zeros((rows, 0), dtype=np.float64), where
    rotation = np.stack(columns, axis=1)
    if offsets is not None:
        if offsets.shape != rotation.shape:
            raise IntegrityError(
                f"member {member.name!r}: spring offsets {offsets.shape} against {rotation.shape} spring rotations"
            )
        rotation = rotation - offsets
    result: FloatArray = chain.stiffness * rotation
    return result, where


# ===== Moment under a carried load (ruling 71) ==========================
#
# Samples. The moment about member z is known exactly at N + 2 points s_0 .. s_{N+1}: the start s_0 = 0 and the end
# s_{N+1} = L from the recorded end wrenches, M(0) = -M_z0 and M(L) = M_z1, and every node spring s_{j+1} =
# (j + 1/2) l from its relative rotation (spring sense, `node_moments`; `core/integrity` start_node_moment and
# end_node_moment are the same lines evaluated at l / 2 and L - l / 2). These are also the engine's lever nodes
# (`core/engine/.../member_loads.hpp`, lever rule): a carried force F at s in [s_k, s_{k+1}] is applied as
# w_k F at s_k and w_{k+1} F at s_{k+1}, w_k + w_{k+1} = 1, w_k s_k + w_{k+1} s_{k+1} = s.
#
# Exactness. The split is statically equivalent to F at s, so for every cut outside (s_k, s_{k+1}) the loads on
# either side have the same resultant and moment as the point load: the support reactions and every sampled moment
# M_i (statically determinate member) are the point load's. Between s_k and s_{k+1} the chain's moment is the chord;
# the point load's is a tent: linear from (s_k, M_k) with the shear left of the load, linear into (s_{k+1}, M_{k+1})
# with the shear right of it. Where no other load acts, the shear left of the load is the slope of the segment
# before, (M_k - M_{k-1}) / (s_k - s_{k-1}), and right of it the slope of the segment after. So the moment under the
# load is where line A through (s_{k-1}, M_{k-1}), (s_k, M_k), extended right, meets line B through (s_{k+1},
# M_{k+1}), (s_{k+2}, M_{k+2}), extended left: with h = s_{k+1} - s_k, slopes g_A and g_B,
#   u = (M_{k+1} - M_k - g_B h) / (g_A - g_B),   M* = M_k + g_A u,   s* = s_k + u,
# exact for one point load in the segment (its position is s*, its size the slope change g_A - g_B). Nothing is
# needed of the force or its foot point.
#
# Rule (both tiers). For every segment [s_k, s_{k+1}], k = 0 .. N, a candidate M* at s* when g_A != g_B and
# 0 < u < h. Equal slopes: the lines coincide (both pass through the chord's ends when the segment and its
# neighbours carry no load), nothing in the segment, nothing added. u at or beyond an end: the kink is at a sample
# already read. The candidate enters the peak with the axial stress and the out-of-plane moment as a spring does.
# Applied only to a member that carries a joint (`NodeChain.carries`, from the neutral member's `carries`): a member
# carrying nothing has no load between its samples except its own weight (each node body's weight at its centre),
# which the samples already read exactly at the springs, and stays unchanged to the last bit.
#
# End segments. The missing neighbour of [s_0, s_1] is the start's own line: value M(0) = -M_z0, slope F_y0 (the
# moment between the start and the first load is M(0) + s F_y0, the start wrench being the support's reaction).
#   pinned or roller start: M(0) = 0 and F_y0 the pin's reaction, which takes the lever share at x = 0 straight into
#     the pin, so it is the true reaction (static equivalence): exact.
#   clamped start: the start wrench is the root joint's reaction moved across the parent's stub [0, l / 2], so its
#     line is the first spring's line extended: a load in the stub gives its share at x = 0 to the parent (lever
#     rule), the two lines coincide and nothing is added; the clamp moment is read off that line (under-read by at
#     most w_0 F s, s < l / 2, a limitation of the stub). A load in [s_1, s_2] is exact.
# The missing neighbour of [s_N, s_{N+1}] is the end's line: value M(L) = M_z1, slope -F_y1.
#   pinned or roller end: the roller's reaction includes the lever share at x = L: the true reaction, exact.
#   clamped end: the hinge on the last segment's tip at x = L, reaction including the share at L: the chain's statics,
#     exact against the chain (the clamped-clamped support moments themselves are the chain's, within its row).
#   free end (cantilever tip): zero wrench, the line M = 0; a load in the last segment gives M* = 0 at the load,
#     which is correct (no moment between a free tip and the last load), and adds nothing.
#
# Distributed load in a segment. For one sign of load the moment is concave there, so it lies below the tangents at
# the segment's ends; lines A and B are those tangents when the neighbours carry nothing, and their intersection is
# at or above the true maximum: conservative. When the neighbours are loaded too (own weight on a carrying member),
# A and B are secants of the neighbours: on a uniform load w with spring moments on the parabola, the excess above the
# continuum moment at the segment's centre is (w / 2)(l / 2)(3 l / 2) = 3 w l^2 / 8, so the member's peak reads
# w L^2 / 8 (1 + 3 / N^2) at most (N = 16: 1.2 percent, inside the chain's 2 percent row; N = 9: 3.7 percent,
# conservative). Two loads in one segment: for the same sign the moment there is concave with A and B its outer
# tangents, so M* is at or above the larger of the two peaks (conservative, exact when one of them is zero); for
# opposite signs the moment is neither concave nor convex there and the intersection is not a bound: not resolved
# (the gantry carries one load per member). A carried moment (the carried
# body's couple about z, also split by the lever rule) puts steps in the samples, not a kink: not resolved.


def carried_moments(
    member: MemberInputs, loads: Wrenches, moments: FloatArray
) -> tuple[FloatArray, FloatArray, FloatArray]:
    """The moment under a carried load per segment (ruling 71, header above): `moments` (rows, N) the spring moments in
    the spring's sense. Returns (candidates (rows, N + 1), positions (rows, N + 1), found (rows, N + 1) bool); a
    segment without a candidate holds 0 at s = 0 with found False."""
    chain = member.nodes
    if chain is None:
        raise IntegrityError(f"member {member.name!r}: built rigid, no node poses; no carried moments")
    springs = moments.shape[1]
    length = member.section.length
    s = np.concatenate(
        (np.zeros(1), (np.arange(springs, dtype=np.float64) + 0.5) * chain.segment_length, np.full(1, length))
    )
    m = np.concatenate((-loads.start_moment[:, 2:3], moments, loads.end_moment[:, 2:3]), axis=1)
    h = np.diff(s)
    chords = np.diff(m, axis=1) / h
    slopes_a = np.concatenate((loads.start_force[:, 1:2], chords[:, :-1]), axis=1)
    slopes_b = np.concatenate((chords[:, 1:], -loads.end_force[:, 1:2]), axis=1)
    difference = slopes_a - slopes_b
    parallel = difference == 0.0
    safe = np.where(parallel, 1.0, difference)
    u = (m[:, 1:] - m[:, :-1] - slopes_b * h) / safe
    found = np.logical_and(np.logical_not(parallel), np.logical_and(u > 0.0, u < h))
    values = np.where(found, m[:, :-1] + slopes_a * u, 0.0)
    positions = np.where(found, s[:-1] + u, 0.0)
    return values, positions, found


def member_normal_stress(
    loads: Wrenches,
    section: SectionProperties,
    member: MemberInputs,
    bodies: BodyPoses | None,
    factor: float = 1.0,
    offsets: FloatArray | None = None,
) -> FloatArray:
    """Normal stress (rows, 2 + N, + N + 1 on a carrying member) along the member (INT-005): the two ends by
    `normal_stress`, then each node spring with abs(N) the larger end value, abs(M_z) its spring moment
    (`node_moments`, elastic part under `offsets`) over I_z / c_y, and abs(M_y) the out-of-plane moment interpolated
    linearly between the end moments (the chain is rigid out of its bending plane, ruling 65) over I_y / c_z; on a
    member that carries a joint also the moment under the load in each segment (`carried_moments`, ruling 71), 0 where
    a segment has none. A member without node poses has its two ends only. `loads` are already scaled by `factor`;
    node moments scale with it (linear elasticity)."""
    ends = normal_stress(loads, section)
    if member.nodes is None or bodies is None:
        return ends
    moments, where = node_moments(member, bodies, offsets)
    moments = moments * factor
    axial = np.maximum(np.abs(loads.start_force[:, 0]), np.abs(loads.end_force[:, 0])) / section.area

    def interior(values: FloatArray, share: FloatArray) -> FloatArray:
        moment_y = loads.start_moment[:, 1:2] * (1.0 - share) - loads.end_moment[:, 1:2] * share
        stress: FloatArray = axial[:, None] + np.abs(moment_y) * section.c_z / section.i_y
        stress = stress + np.abs(values) * section.c_y / section.i_z
        return stress

    springs = interior(moments, np.broadcast_to(where / section.length, moments.shape))
    if not member.nodes.carries:
        return np.concatenate((ends, springs), axis=1)
    values, positions, found = carried_moments(member, loads, moments)
    under = np.where(found, interior(values, positions / section.length), 0.0)
    return np.concatenate((ends, springs, under), axis=1)


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
