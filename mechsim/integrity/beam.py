"""Beam theory on recorded member end-loads (member frame, round 5 rulings 24 and 25), shared by the checks."""

from __future__ import annotations

import math

import numpy as np

from mechsim.integrity.plugin import FloatArray, IntegrityError, MemberInputs, SectionProperties, Wrenches

GRID_POINTS = 201
ROW_CHUNK = 4096
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


def _constraints(member: MemberInputs) -> FloatArray:
    rows: list[list[float]] = []
    for at, condition in member.supports:
        rows.append([1.0, at, 0.0])
        if condition == "clamped":
            rows.append([0.0, 1.0, 1.0])
    matrix = np.asarray(rows, dtype=np.float64)
    if np.linalg.matrix_rank(matrix[:, :2]) < 2:
        raise IntegrityError(f"member {member.name!r}: its supports leave it free to move; no deflection")
    return matrix


def _plane(
    moment0: FloatArray, shear0: FloatArray, shear1: FloatArray, balance: FloatArray, length: float, grid: FloatArray
) -> tuple[FloatArray, FloatArray]:
    """E I v and E I v' (rows, grid) of the particular solution: the start's internal moment A and shear B and the
    interior load Q = -(F_start + F_end) at its statically equivalent point a = balance / Q."""
    load = -(shear0 + shear1)
    scale = np.maximum(np.abs(shear0), np.abs(shear1))
    present = np.abs(load) > 1e-9 * scale
    safe = np.where(present, load, 1.0)
    where = np.clip(np.where(present, balance / safe, length), 0.0, length)
    load = np.where(present, load, 0.0)
    x = grid[None, :]
    beyond = np.maximum(x - where[:, None], 0.0)
    value = moment0[:, None] * x**2 / 2.0 + shear0[:, None] * x**3 / 6.0 + load[:, None] * beyond**3 / 6.0
    slope = moment0[:, None] * x + shear0[:, None] * x**2 / 2.0 + load[:, None] * beyond**2 / 2.0
    return value, slope


def _fit(
    member: MemberInputs, matrix: FloatArray, grid: FloatArray, value: FloatArray, slope: FloatArray
) -> FloatArray:
    """Add C0 + C1 x so the supports hold (v = 0 at each, v' = 0 at a clamp), least squares over the conditions."""
    rhs = []
    for at, condition in member.supports:
        rhs.append(-_at(grid, value, at))
        if condition == "clamped":
            rhs.append(-_at(grid, slope, at))
    stacked = np.stack(rhs, axis=0)
    coefficients = np.linalg.pinv(matrix[:, :2]) @ stacked
    result: FloatArray = value + coefficients[0][:, None] + coefficients[1][:, None] * grid[None, :]
    return result


def _at(grid: FloatArray, values: FloatArray, at: float) -> FloatArray:
    index = int(np.argmin(np.abs(grid - at)))
    return values[:, index]


def deflection(member: MemberInputs, loads: Wrenches) -> FloatArray:
    """Peak transverse deflection magnitude (rows,) along the member per sample, quasi-static from the end-loads:
    E I v'' equals the internal moment from the start wrench and the interior load resolved to one point load at its
    statically equivalent position, with the support conditions fixing the rigid-body terms. Exact for end loads and
    one interior point load; for a uniform load it reads high (simply supported: 1/48 against 5/384 of W L^3 / E I)."""
    section = member.section
    length = section.length
    points = [length * index / (GRID_POINTS - 1) for index in range(GRID_POINTS)]
    points += [at for at, _ in member.supports]
    grid = np.unique(np.asarray(points, dtype=np.float64))
    matrix = _constraints(member)
    rows = loads.start_force.shape[0]
    out = np.zeros(rows, dtype=np.float64)
    for first in range(0, rows, ROW_CHUNK):
        part = slice(first, min(rows, first + ROW_CHUNK))
        fs = loads.start_force[part]
        fe = loads.end_force[part]
        ms = loads.start_moment[part]
        me = loads.end_moment[part]
        value_y, slope_y = _plane(
            -ms[:, 2], fs[:, 1], fe[:, 1], -(ms[:, 2] + me[:, 2] + length * fe[:, 1]), length, grid
        )
        value_z, slope_z = _plane(ms[:, 1], fs[:, 2], fe[:, 2], ms[:, 1] + me[:, 1] - length * fe[:, 2], length, grid)
        v = _fit(member, matrix, grid, value_y, slope_y) / (section.elastic_modulus * section.i_z)
        w = _fit(member, matrix, grid, value_z, slope_z) / (section.elastic_modulus * section.i_y)
        out[part] = np.max(np.hypot(v, w), axis=1)
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
