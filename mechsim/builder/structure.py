"""Structural pieces of the build: the lumped chain's linear statics and first mode, calibration (BLD-019), mounts and
fasteners as addressable fastenings (BLD-031), ropes (BLD-032) and catalog sections (LIB-030).

The chain is the engine's lumped beam (ENG-022, `core/engine` lumped_beam): N segments of l = L / N, a rotational
spring E I / l at each segment midpoint x_j = (j + 1/2) l, rigid spans node to node, a clamped root leaving [0, l/2]
on the parent, a pinned root adding a free pin at x = 0 and a body [0, l/2], a roller end holding the transverse
displacement at x = L. Linearized in the hinge angles theta: y(x) = sum over x_j < x of theta_j (x - x_j); each
span is a thin rod of mass rho A s and inertia rho A s^3 / 12 about its centre, as the engine builds it.
Structural damping (BLD-040, ruling 78): a modal damping ratio of the first bending mode becomes stiffness-proportional
node damping, `node_damping`.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy import linalg  # type: ignore[import-untyped]

from mechsim.design.schema import Body, Design, OpenValue, Rope


class ChainError(ValueError):
    """The member has no static solution (a pinned root with a free end is a mechanism)."""


@dataclass(frozen=True)
class Chain:
    length: float
    hinges: tuple[float, ...]
    springs: tuple[bool, ...]
    starts: tuple[float, ...]
    spans: tuple[float, ...]
    roller: bool


def chain_layout(root: str, end: str, segments: int, length: float) -> Chain:
    segment = length / segments
    hinges = [(j + 0.5) * segment for j in range(segments)]
    springs = [True] * segments
    spans = [segment] * (segments - 1) + [segment / 2.0]
    start = segment / 2.0
    if root == "pinned":
        hinges = [0.0, *hinges]
        springs = [False, *springs]
        spans = [segment / 2.0, *spans]
        start = 0.0
    starts: list[float] = []
    for span in spans:
        starts.append(start)
        start += span
    return Chain(length, tuple(hinges), tuple(springs), tuple(starts), tuple(spans), end == "roller")


def load_point(end: str, length: float) -> float:
    """Where a calibration load acts and its deflection is read: the tip of a free end, else midspan (DES-034)."""
    if end == "free":
        return length
    return 0.5 * length


def _influence(chain: Chain, x: float) -> NDArray[np.float64]:
    return np.array([max(x - hinge, 0.0) for hinge in chain.hinges])


def _matrices(
    chain: Chain, flexural_rigidity: float, mass_per_length: float
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Stiffness K, mass M in the hinge angles, and a basis of the angles the roller allows."""
    count = len(chain.hinges)
    segment = chain.length / sum(1 for spring in chain.springs if spring)
    stiffness = np.diag([flexural_rigidity / segment if spring else 0.0 for spring in chain.springs])
    mass = np.zeros((count, count))
    for index, (start, span) in enumerate(zip(chain.starts, chain.spans, strict=True)):
        centre = start + 0.5 * span
        lever = np.array([centre - chain.hinges[j] if j <= index else 0.0 for j in range(count)])
        turn = np.array([1.0 if j <= index else 0.0 for j in range(count)])
        body_mass = mass_per_length * span
        mass += body_mass * np.outer(lever, lever) + body_mass * span * span / 12.0 * np.outer(turn, turn)
    basis = np.eye(count)
    if chain.roller:
        basis = linalg.null_space(_influence(chain, chain.length).reshape(1, count))
    return stiffness, mass, basis


def chain_deflection(chain: Chain, flexural_rigidity: float, load: float, at: float) -> float:
    """Static deflection at `at` under a point load `load` there (same sense), small angles."""
    stiffness, _mass, basis = _matrices(chain, flexural_rigidity, 1.0)
    reduced = basis.T @ stiffness @ basis
    influence = _influence(chain, at)
    if np.linalg.matrix_rank(reduced) < reduced.shape[0]:
        raise ChainError("the member's supports leave it a mechanism; it has no static deflection")
    angles = basis @ np.linalg.solve(reduced, load * (basis.T @ influence))
    return float(influence @ angles)


def chain_first_frequency(chain: Chain, flexural_rigidity: float, mass_per_length: float) -> float:
    """Lowest eigenfrequency of K phi = omega^2 M phi on the allowed angles, rad/s."""
    stiffness, mass, basis = _matrices(chain, flexural_rigidity, mass_per_length)
    values = linalg.eigh(basis.T @ stiffness @ basis, basis.T @ mass @ basis, eigvals_only=True)
    return math.sqrt(max(float(values[0]), 0.0))


# ===== Structural damping ===============================================


def _member_chain(root: str, end: str, segments: int, length: float) -> Chain:
    """The chain of any lumped support pair; a clamped far end uses the roller layout, its rotation held separately."""
    if end == "clamped":
        return chain_layout(root, "roller", segments, length)
    return chain_layout(root, end, segments, length)


def _member_system(
    root: str, end: str, segments: int, length: float, flexural_rigidity: float, mass_per_length: float
) -> tuple[Chain, NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """The one chain of any support pair (BLD-019, BLD-040): stiffness, mass and the basis of allowed angles; a
    clamped far end holds both the end deflection and the end slope (ENG-022, ruling 16)."""
    chain = _member_chain(root, end, segments, length)
    stiffness, mass, basis = _matrices(chain, flexural_rigidity, mass_per_length)
    if end == "clamped":
        basis = linalg.null_space(np.vstack([_influence(chain, length), np.ones(len(chain.hinges))]))
    return chain, stiffness, mass, basis


def member_deflection(
    root: str, end: str, segments: int, length: float, flexural_rigidity: float, load: float, at: float
) -> float:
    """Static deflection at `at` under a point load `load` there, on the member's chain for its supports."""
    chain, stiffness, _mass, basis = _member_system(root, end, segments, length, flexural_rigidity, 1.0)
    reduced = basis.T @ stiffness @ basis
    if np.linalg.matrix_rank(reduced) < reduced.shape[0]:
        raise ChainError("the member's supports leave it a mechanism; it has no static deflection")
    influence = _influence(chain, at)
    angles = basis @ np.linalg.solve(reduced, load * (basis.T @ influence))
    return float(influence @ angles)


def loaded_first_frequency(
    root: str,
    end: str,
    segments: int,
    length: float,
    flexural_rigidity: float,
    mass_per_length: float,
    carried_mass: float = 0.0,
    at: float = 0.0,
) -> float:
    """Lowest eigenfrequency (rad/s) of the member's chain for its supports, with a point mass `carried_mass` moving
    with the deflection at `at` (the engine's carried mass, ENG-052: inertia at the foot point for the deflection
    motion, no weight, no rotary inertia): M + m a a^T, a the deflection influence of the hinge angles at `at`. The
    same chain calibrates a member (BLD-019), so a calibrated member's damping uses its recorded frequency."""
    chain, stiffness, mass, basis = _member_system(root, end, segments, length, flexural_rigidity, mass_per_length)
    influence = _influence(chain, at)
    mass = mass + carried_mass * np.outer(influence, influence)
    values = linalg.eigh(basis.T @ stiffness @ basis, basis.T @ mass @ basis, eigvals_only=True)
    return math.sqrt(max(float(values[0]), 0.0))


def node_damping(ratio: float, omega: float, node_stiffness: float) -> float:
    """The node damper c (N m s/rad) that gives the chain's first bending mode the modal damping ratio `ratio`.

    Derivation (stiffness-proportional damping, ruling 78). The chain in its hinge angles theta is
    M theta'' + C theta' + K theta = 0 with K = k diag(spring nodes), k = E I / l, and the engine puts the damper c on
    the same nodes (a pinned root's pin has neither), so C = beta K with beta = c / k; a clamped far end or a roller
    restricts theta to a basis B, and B^T C B = beta B^T K B still. Then the undamped modes phi_n of (K, M) diagonalise
    C: phi_n^T C phi_n = beta omega_n^2 m_n = 2 zeta_n omega_n m_n, so zeta_n = beta omega_n / 2 exactly. Choosing
    beta = 2 zeta / omega_1, omega_1 the chain's own first mode for its supports and section (unloaded), gives
    zeta_1 = zeta and zeta_n = zeta omega_n / omega_1 above it; c = beta k = 2 zeta k / omega_1. A carried mass adds to
    M only, so C = beta K holds and the loaded first mode omega_c < omega_1 gets zeta omega_c / omega_1."""
    return 2.0 * ratio * node_stiffness / omega


def lowest_carried_frequency(
    root: str,
    end: str,
    segments: int,
    length: float,
    flexural_rigidity: float,
    mass_per_length: float,
    carried_mass: float,
) -> tuple[float, float]:
    """The lowest loaded first frequency (rad/s) over the span and the foot position (m) giving it, with the carried
    mass at 4 N + 1 evenly spaced points of [0, L]."""
    count = 4 * segments
    lowest = math.inf
    where = 0.0
    for index in range(count + 1):
        at = length * index / count
        omega = loaded_first_frequency(
            root, end, segments, length, flexural_rigidity, mass_per_length, carried_mass, at
        )
        if omega < lowest:
            lowest = omega
            where = at
    return lowest, where


CALIBRATION_MIN_SEGMENTS = 8
DEFAULT_SEGMENTS = 8
CLAMPED_CLAMPED_SEGMENTS = 16


def default_segments(root: str, end: str) -> int:
    """The segment count a member gets when its design states none (BLD-018, rulings 17 and 31): eight, and sixteen
    for a clamped-clamped member, whose chain deflection error is -4 / N^2 (-6.25 % at 8, -1.56 % at 16)."""
    if root == "clamped" and end == "clamped":
        return CLAMPED_CLAMPED_SEGMENTS
    return DEFAULT_SEGMENTS


def calibration_min_segments(root: str, end: str) -> int:
    """The fewest segments a calibrated member may have (ruling 32): calibrating a clamped-clamped member below
    sixteen would scale E by 1.067 and density by 1.120 at eight, so it is a build error; eight otherwise."""
    if root == "clamped" and end == "clamped":
        return CLAMPED_CLAMPED_SEGMENTS
    return CALIBRATION_MIN_SEGMENTS


CALIBRATION_SCALE_RANGE = (0.5, 2.0)


@dataclass(frozen=True)
class CalibrationScales:
    stiffness_scale: float
    mass_scale: float


def approximation_notes(member: str, scales: CalibrationScales) -> list[str]:
    """A note per scale factor outside [0.5, 2]: the recorded values are far from the section's, so the lumped beam is
    an approximation of that member (ruling 20)."""
    low, high = CALIBRATION_SCALE_RANGE
    notes: list[str] = []
    for name, value in (("stiffness_scale", scales.stiffness_scale), ("mass_scale", scales.mass_scale)):
        if value < low or value > high:
            notes.append(
                f"flexible body '{member}': calibration {name} {value!r} lies outside [{low}, {high}]; the lumped beam "
                "is an approximation (BLD-019)"
            )
    return notes


def calibrate(
    chain: Chain,
    flexural_rigidity: float,
    mass_per_length: float,
    load: float,
    deflection: float,
    frequency_hz: float,
) -> CalibrationScales:
    """Scale E I so the chain's deflection under `load` equals `deflection` (deflection is linear in 1 / E I), then
    scale the distributed mass so its first mode equals `frequency_hz` (omega is proportional to sqrt(E I / m)),
    BLD-019."""
    at = chain.length
    if chain.roller:
        at = 0.5 * chain.length
    stiffness_scale = chain_deflection(chain, flexural_rigidity, load, at) / deflection
    omega = chain_first_frequency(chain, flexural_rigidity * stiffness_scale, mass_per_length)
    target = 2.0 * math.pi * frequency_hz
    return CalibrationScales(stiffness_scale, (omega / target) ** 2)


def lumped_section(values: Mapping[str, float | bool]) -> dict[str, float]:
    """The flexible_beam plugin entry (LIB-030): a part's section and material constants, SI."""
    out: dict[str, float] = {}
    for name in ("area", "second_moment", "length", "elastic_modulus", "density"):
        value = values[name]
        if isinstance(value, bool):
            raise TypeError(f"section parameter '{name}' is a flag")
        out[name] = value
    return out


def mount_joints(design: Design) -> list[str]:
    return [joint.name for joint in design.joints if joint.type == "fixed"]


def fastenings(design: Design) -> list[dict[str, Any]]:
    """Every mount (a fixed joint) and every fastener set of a body (held across the body's joint), BLD-031."""
    parents = {joint.child: joint for joint in design.joints}
    out: list[dict[str, Any]] = []
    for joint in design.joints:
        if joint.type == "fixed":
            out.append(
                {
                    "name": joint.name,
                    "kind": "mount",
                    "body_a": joint.parent,
                    "body_b": joint.child,
                    "joint": joint.name,
                }
            )
    for body in design.bodies:
        held = parents.get(body.name)
        if held is None:
            continue
        kinds = [fastener.kind for fastener in body.fasteners]
        for index, fastener in enumerate(body.fasteners):
            name = f"{body.name}_{fastener.kind}"
            if kinds.count(fastener.kind) > 1:
                name = f"{name}_{index}"
            out.append(
                {"name": name, "kind": "fastener", "body_a": held.parent, "body_b": body.name, "joint": held.name}
            )
    return out


def member_mounts(body: Body, design: Design) -> list[str]:
    return [joint.name for joint in design.joints if joint.type == "fixed" and body.name in (joint.parent, joint.child)]


def rope_element(rope: Rope, values: Mapping[str, float | bool]) -> dict[str, Any] | None:
    """The neutral rope (ADR 0016) of a design rope; None while its to end is a case payload (placed at run time)."""
    if rope.to_end.body is None or rope.from_end.body is None:
        return None
    return _rope(rope, rope.to_end.body, values)


def payload_rope_element(rope: Rope, values: Mapping[str, float | bool]) -> dict[str, Any] | None:
    """The neutral rope of a design rope whose to end is a case payload, `to_body` naming the payload and `to_point`
    the design's point, offset from the payload's top face centre (BLD-032; `payload.with_payload` places both)."""
    if rope.to_end.payload is None or rope.from_end.body is None:
        return None
    return _rope(rope, rope.to_end.payload, values)


def _rope(rope: Rope, to_body: str, values: Mapping[str, float | bool]) -> dict[str, Any]:
    rest: float | bool | OpenValue | None = rope.rest_length
    if rest is None:
        rest = values["rest_length"]
    if isinstance(rest, OpenValue):
        raise TypeError(f"rope '{rope.name}': open rest_length reached the build")
    numbers = {name: values[name] for name in ("axial_stiffness", "damping", "mass_per_length")}
    element: dict[str, Any] = {
        "name": rope.name,
        "from_body": rope.from_end.body,
        "from_point": _points(rope.from_end.point),
        "to_body": to_body,
        "to_point": _points(rope.to_end.point),
        "segments": rope.segments,
        "axial_stiffness": f"{float(numbers['axial_stiffness'])!r} N",
        "damping": f"{float(numbers['damping'])!r} N*s",
        "mass_per_length": f"{float(numbers['mass_per_length'])!r} kg/m",
        "rest_length": f"{float(rest)!r} m",
    }
    breaking = values.get("breaking_tension")
    if isinstance(breaking, float):
        element["breaking_tension"] = f"{breaking!r} N"
    return element


def _points(values: Sequence[Any]) -> list[str]:
    return [f"{float(value)!r} m" for value in values]
