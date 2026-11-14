"""Beam theory on recorded member end-loads (member frame, round 5 rulings 24 and 25) and the deflection from the
recorded node poses (ruling 28), shared by the checks."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from mechsim.integrity.plugin import (
    BodyPoses,
    CarriedLoad,
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


# ===== Moment under a carried load (rulings 71, 75) =====================
#
# Samples. The moment about member z is known at N + 2 points x_0 .. x_{N+1}: the start x_0 = 0 and the end
# x_{N+1} = L from the recorded end wrenches, M(0) = -M_z0 and M(L) = M_z1, and every node spring x_{j+1} =
# (j + 1/2) l from its relative rotation (spring sense, `node_moments`). These are the engine's lever nodes
# (`core/engine/.../member_loads.hpp`, lever rule; `load_nodes`): node 0 is a clamped root's parent or a pinned root's
# pin segment at x = 0, node i >= 1 at a spring is the node body that starts there (the body to the spring's right),
# node N + 1 the last body's tip. A carried joint at s in [x_k, x_{k+1}] (k the largest with x_k <= s, at most N)
# puts w_k F at x_k and w_{k+1} F at x_{k+1}, w_k = (x_{k+1} - s) / l_k, w_k + w_{k+1} = 1, and the torques w_k C on
# node k's body and w_{k+1} C on node k + 1's (F = F . y, C = M_p . z about the foot point, ENG-052).
#
# Which samples are clean (equal the moment of a force F and a couple C at s). A sample at x_i is the moment at the
# cut x_i, from the loads on one side. The two force shares are statically equivalent to F at s, so every cut outside
# the open interval (x_k, x_{k+1}) sees the point force's moment exactly, and a share applied at the cut has no arm.
# The torque shares sit on the bodies right of x_k and of x_{k+1}: the cut at x_i <= x_k has both on its right, like
# C at s > x_i; the cut at x_i >= x_{k+2} has both on its left, like C at s < x_i; the cut at x_{k+1} has w_k C on its
# left where the point couple puts all of C: off by w_{k+1} C. So samples 0 .. k and k + 2 .. N + 1 are clean, sample
# k + 1 is off by w_{k+1} C (clean under a force alone). The end samples are the support reactions: clean at a pinned
# or roller end (the share at x = 0 or L goes straight into the support), a free tip (zero) and a clamped far end
# (the chain's own statics, its clamp moment within the chain's row); a clamped root's x = 0 is clean unless a load
# lies in its half-segment stub [0, l / 2], whose share w_0 goes to the parent and bypasses the member.
#
# The rule. Where no other load acts, the moment is linear each side of s with a jump C at s. Line A through the clean
# samples k - 1 and k, evaluated at s, is the moment just left of the load, A(s) = M(s-); line B through the clean
# samples k + 2 and k + 3 gives B(s) = M(s+); M(s+) - M(s-) = -C in the spring sense (a weight P offset by +e along
# the member gives C = -P e and a step +P e, brief M3-I5c's closure test). The peak under the load is the larger
# magnitude of the two: exact for a force and a couple, no bound, nothing needed but s. Fewer than two samples on a
# side:
#   left, k = 0: a pinned root's start line (value M(0) = 0, slope the pin's reaction F_y0), clean; a clamped root's
#     stub has no clean left line (bounded below);
#   right, k + 3 > N + 1: the end's line (value M(L) = M_z1, slope -F_y1), clean at every end condition (above).
# s is the carried body's origin projected on the member axis of the root frame (the parent's pose times the member
# placement, `_root_frame`) at the step's start, the state the engine placed the pair at: the previous row's poses
# and the reaction recorded by the previous row (the engine applies the reaction of the step before; the first row,
# and the first row of an evaluation window, use their own). Tier 1 does the same with the carried bodies and joints
# its limits table lists (`core/integrity`, `carried_reads`).
#
# Clusters. Loads whose segments k differ by at most 2 share samples (a load's line B would cross the next load, or its
# off sample k + 1 would be the next load's line A) and are read together: line A of the first at its s_1 and line B
# of the last at its s_m. Two forces without couples are exact: between them the moment is linear (no load between),
# the chord from A(s_1) to B(s_2), so the peak over the cluster is the larger of the two magnitudes, whatever their
# signs or whether they share a segment. Not resolved, a bound U added to the row (conservative, with a line): a
# couple in a cluster of two or more, three or more loads, and a clamped root's stub. With a = x_{k_1} and b =
# x_{k_m + 2} (L when beyond the end), both clean samples bracketing the cluster, the true moment on [a, b] is the
# chord between M(a) and M(b), at most the read of the two samples, plus each load's moment on a simply supported span
# [a, b]: abs(F_i) u (b - a - u) / (b - a) <= abs(F_i) (b - a) / 4 for a force and at most abs(C_i) for a couple; so
# U = sum abs(F_i) (b - a) / 4 + sum abs(C_i). In the stub the sample at a = 0 misses the share w_0 F s + w_0 C, at most
# abs(F) l / 8 + abs(C) (w_0 s <= l / 8 for s in [0, l / 2]), added. A couple below 1e-9 abs(F) L counts as none for
# the two-force rule (rounding of a joint on the member axis). F and C are rebuilt from the recorded reaction like s.
#
# Own weight within the loaded span: the member's weight w per length makes the moment between the samples a parabola,
# not a line, so the extrapolated lines miss its curvature: A reaches s from x_k (at most l), B from x_{k+2} (at most
# 2 l), and a secant of a parabola extended by d beyond its chord of length l departs from it by (w / 2) d (d + l):
# up to w l^2 for A and 3 w l^2 for B. The line lies on the outside of the curvature: above a sagging moment (an
# over-read, conservative) and, in a hogging region (a clamped end's), below it by the same at most 3 w l^2. The reads
# stay the rule's values (exact without own weight); the peak a check takes is the largest magnitude over the ends,
# every spring and the reads (`member_normal_stress`, tier 1 the same), so it is never below a plain sample (brief
# M3-I5c). A hogging under-read can then matter only where the moment at s exceeds every sample in magnitude, which
# needs a hogging extremum strictly between samples: under own weight (M'' = -w, concave) and downward forces
# (concave kinks) the hogging peak lies at an end, a clean sample; only a carried couple's jump can put it at s, and
# there the read is low by at most 3 w l^2. A member carrying nothing has no candidate and stays unchanged to the
# last bit.

COUPLE_FLOOR = 1e-9


@dataclass(frozen=True)
class CarriedReads:
    """The rule's reads per row: `values` (rows, 2 n) the moments just left of each cluster's first load and just right
    of its last (spring sense), `positions` (rows, 2 n) where (m), `found` (rows, 2 n) which entries are reads, `bound`
    (rows,) N m the largest U of a cluster the rule does not resolve (0 elsewhere), `causes` the kinds of those."""

    values: FloatArray
    positions: FloatArray
    found: FloatArray
    bound: FloatArray
    causes: tuple[str, ...]


STUB = "a carried load in a clamped root's half-segment stub (no clean sample left of it)"
CROWDED = "three or more carried loads within two segments of each other"
COUPLED = "a carried couple among several loads within two segments of each other"


def _root_frame(member: MemberInputs, bodies: BodyPoses) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
    chain = member.nodes
    assert chain is not None
    rows = bodies.positions.shape[0]
    parent_position = np.zeros((rows, 3), dtype=np.float64)
    parent_orientation = np.tile(np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64), (rows, 1))
    if chain.parent_body is not None:
        parent_position = bodies.positions[:, chain.parent_body, :]
        parent_orientation = bodies.orientations[:, chain.parent_body, :]
    placement = np.tile(np.asarray(chain.position, dtype=np.float64), (rows, 1))
    turn = _multiply(parent_orientation, np.tile(np.asarray(chain.orientation, dtype=np.float64), (rows, 1)))
    axes = []
    for axis in np.eye(3):
        axes.append(_rotate(turn, np.tile(axis, (rows, 1))))
    return parent_position + _rotate(parent_orientation, placement), axes[0], axes[1], axes[2]


def carried_couples(member: MemberInputs, bodies: BodyPoses, item: CarriedLoad) -> tuple[FloatArray, ...]:
    """(s, F . y, C) per row of one carried joint from that row's poses and recorded reaction (header above)."""
    origin, x, y, z = _root_frame(member, bodies)
    child = bodies.positions[:, item.body, :]
    s = np.einsum("rj,rj->r", child - origin, x)
    foot = origin + s[:, None] * x
    moment = item.moment + np.cross(child - foot, item.force)
    return s, np.einsum("rj,rj->r", item.force, y), np.einsum("rj,rj->r", moment, z)


def carried_at(member: MemberInputs, bodies: BodyPoses) -> tuple[FloatArray, FloatArray, FloatArray]:
    """(s, F . y, C) (rows, n) of each carried joint as the engine applied it in each row's step: from the previous
    row's poses and reaction, the first row from its own (header above)."""
    rows = bodies.positions.shape[0]
    count = len(member.carried)
    s = np.zeros((rows, count), dtype=np.float64)
    force = np.zeros((rows, count), dtype=np.float64)
    couple = np.zeros((rows, count), dtype=np.float64)
    source = np.maximum(np.arange(rows) - 1, 0)
    for column, item in enumerate(member.carried):
        at, f, c = carried_couples(member, bodies, item)
        s[:, column] = at[source]
        force[:, column] = f[source]
        couple[:, column] = c[source]
    return s, force, couple


def _line(x: FloatArray, m: FloatArray, first: int) -> tuple[float, float, float]:
    return float(x[first + 1]), float(m[first + 1]), float((m[first + 1] - m[first]) / (x[first + 1] - x[first]))


def _at(line: tuple[float, float, float], s: float) -> float:
    return line[1] + line[2] * (s - line[0])


def row_reads(
    x: FloatArray,
    m: FloatArray,
    start_slope: float,
    end_slope: float,
    pinned_root: bool,
    segment: float,
    loads: list[tuple[float, float, float]],
) -> tuple[list[tuple[float, float]], float, set[str]]:
    """One row of the rule (header above): samples `x`, `m` (N + 2), the start line's slope F_y0 and the end line's
    -F_y1, and the carried loads (s, F, C). Returns the reads (moment, position), the bound U (N m) and its causes."""
    last = x.size - 1
    length = float(x[last])
    placed: list[tuple[float, int, float, float]] = []
    for s, f, c in loads:
        at = min(max(s, 0.0), length)
        k = 0
        while k + 1 < last and x[k + 1] <= at:
            k += 1
        placed.append((at, k, f, c))
    placed.sort(key=lambda item: item[0])
    clusters: list[list[tuple[float, int, float, float]]] = []
    for item in placed:
        if len(clusters) > 0 and item[1] - clusters[-1][-1][1] <= 2:
            clusters[-1].append(item)
        else:
            clusters.append([item])
    reads: list[tuple[float, float]] = []
    bound = 0.0
    causes: set[str] = set()
    for cluster in clusters:
        first, final = cluster[0], cluster[-1]
        left: tuple[float, float, float] | None = None
        if first[1] >= 1:
            left = _line(x, m, first[1] - 1)
        elif pinned_root:
            left = (0.0, float(m[0]), start_slope)
        right = (length, float(m[last]), end_slope)
        if final[1] + 3 <= last:
            right = _line(x, m, final[1] + 2)
        if left is not None:
            reads.append((_at(left, first[0]), first[0]))
        reads.append((_at(right, final[0]), final[0]))
        coupled = any(abs(c) > COUPLE_FLOOR * abs(f) * length for _, _, f, c in cluster)
        if left is not None and (len(cluster) == 1 or (len(cluster) == 2 and not coupled)):
            continue
        a = float(x[first[1]])
        b = float(x[min(final[1] + 2, last)])
        value = sum(abs(f) for _, _, f, _ in cluster) * (b - a) / 4.0 + sum(abs(c) for _, _, _, c in cluster)
        if left is None:
            value = value + sum(abs(f) * segment / 8.0 + abs(c) for _, k, f, c in cluster if k == 0)
            causes.add(STUB)
        elif len(cluster) > 2:
            causes.add(CROWDED)
        else:
            causes.add(COUPLED)
        bound = max(bound, value)
    return reads, bound, causes


def carried_reads(
    member: MemberInputs, loads: Wrenches, moments: FloatArray, bodies: BodyPoses, factor: float = 1.0
) -> CarriedReads:
    """The rule over every row (header above): `loads` and `moments` (rows, N) already at the design load (`factor`),
    the carried loads from `carried_at` scaled by it for the bound."""
    chain = member.nodes
    if chain is None:
        raise IntegrityError(f"member {member.name!r}: built rigid, no node poses; no carried moments")
    rows, springs = moments.shape
    count = len(member.carried)
    values = np.zeros((rows, 2 * count), dtype=np.float64)
    positions = np.zeros((rows, 2 * count), dtype=np.float64)
    found = np.zeros((rows, 2 * count), dtype=bool)
    bound = np.zeros(rows, dtype=np.float64)
    causes: set[str] = set()
    if count == 0 or rows == 0:
        return CarriedReads(values, positions, found, bound, ())
    length = member.section.length
    x = np.concatenate(
        (np.zeros(1), (np.arange(springs, dtype=np.float64) + 0.5) * chain.segment_length, np.full(1, length))
    )
    m = np.concatenate((-loads.start_moment[:, 2:3], moments, loads.end_moment[:, 2:3]), axis=1)
    s, force, couple = carried_at(member, bodies)
    pinned = chain.root == "pinned"
    for row in range(rows):
        items = [
            (float(s[row, j]), factor * float(force[row, j]), factor * float(couple[row, j])) for j in range(count)
        ]
        start, end = float(loads.start_force[row, 1]), float(-loads.end_force[row, 1])
        reads, value, kinds = row_reads(x, m[row], start, end, pinned, chain.segment_length, items)
        bound[row] = value
        causes.update(kinds)
        for column, (value, position) in enumerate(reads):
            values[row, column] = value
            positions[row, column] = position
            found[row, column] = True
    return CarriedReads(values, positions, found, bound, tuple(sorted(causes)))


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
    member that carries a joint also the moments just left and right of its carried loads (`carried_reads`, rulings
    71, 75), 0 where a row has fewer reads. A member without node poses has its two ends only. `loads` are already
    scaled by `factor`; node moments scale with it (linear elasticity)."""
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
    reads = carried_reads(member, loads, moments, bodies, factor)
    under = np.where(reads.found, interior(reads.values, reads.positions / section.length), 0.0)
    return np.concatenate((ends, springs, under), axis=1)


@dataclass(frozen=True)
class SectionHistory:
    """One section along a member (INT-008): `index` in the order of `member_normal_stress` (0 start, 1 end, 2 + j
    spring j, then the reads under a carried load), `position` (m; NaN for a read under a carried load, which moves
    with it), `corners` the signed normal stress at its four corners (eight end corners for a member without node
    poses), `present` (rows,) the rows that have a value (a read under a carried load exists only in the rows whose
    clusters give one)."""

    index: int
    position: float
    corners: list[FloatArray]
    present: FloatArray


def _corners(
    normal: FloatArray, moment_y: FloatArray, moment_z: FloatArray, section: SectionProperties
) -> list[FloatArray]:
    out: list[FloatArray] = []
    for sign_y in (1.0, -1.0):
        for sign_z in (1.0, -1.0):
            value = normal / section.area
            value = value + sign_y * moment_y * section.c_z / section.i_y
            value = value + sign_z * moment_z * section.c_y / section.i_z
            out.append(value)
    return out


def member_sections(
    loads: Wrenches,
    section: SectionProperties,
    member: MemberInputs,
    bodies: BodyPoses | None,
    factor: float = 1.0,
    offsets: FloatArray | None = None,
) -> list[SectionHistory]:
    """Every section of `member_normal_stress` with the signed normal stress at its four corners (INT-008, INT-011,
    rulings 72, 75): N / A +- M_y c_z / I_y +- M_z c_y / I_z with the section's signed moments (a spring's or a read's
    axial force the end value of the larger magnitude, the start's on a tie; M_y interpolated between the ends). A
    member without node poses (or no rows) has one section, index -1, the eight end-corner histories of
    `fibre_histories`.
    `loads` are already scaled by `factor`; node moments scale with it."""
    rows = loads.start_force.shape[0]
    if member.nodes is None or bodies is None or rows == 0:
        return [SectionHistory(-1, math.nan, fibre_histories(loads, section), np.ones(rows, dtype=bool))]
    everywhere = np.ones(rows, dtype=bool)
    start_normal = -loads.start_force[:, 0]
    end_normal = loads.end_force[:, 0]
    out = [
        SectionHistory(
            0, 0.0, _corners(start_normal, loads.start_moment[:, 1], loads.start_moment[:, 2], section), everywhere
        ),
        SectionHistory(
            1, section.length, _corners(end_normal, loads.end_moment[:, 1], loads.end_moment[:, 2], section), everywhere
        ),
    ]
    normal = np.where(np.abs(start_normal) >= np.abs(end_normal), start_normal, end_normal)
    moments, where = node_moments(member, bodies, offsets)
    moments = moments * factor

    def interior(moment_z: FloatArray, share: FloatArray) -> list[FloatArray]:
        moment_y = loads.start_moment[:, 1] * (1.0 - share) - loads.end_moment[:, 1] * share
        return _corners(normal, moment_y, moment_z, section)

    springs = moments.shape[1]
    for j in range(springs):
        share = np.full(rows, where[j] / section.length)
        out.append(SectionHistory(2 + j, float(where[j]), interior(moments[:, j], share), everywhere))
    if not member.nodes.carries:
        return out
    reads = carried_reads(member, loads, moments, bodies, factor)
    for column in range(reads.values.shape[1]):
        present = reads.found[:, column].astype(bool)
        share = reads.positions[:, column] / section.length
        out.append(SectionHistory(2 + springs + column, math.nan, interior(reads.values[:, column], share), present))
    return out


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
