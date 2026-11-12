"""Neutral model to the own engine's model document (PHY-018, PHY-020): build time only, stepping is
`core/physics/native/` (`NativeAdapter`), which parses this JSON into `engine::MultibodyModel`.

Engine body order: the neutral bodies in file order (so neutral body, joint and payload indices are engine indices),
then a fixed world anchor when a constraint names the world (engine loop constraints join two bodies), then the C++
side appends member segments and rope nodes. Constraints keep neutral order; members, ropes and transmissions too.
Per-DOF arrays are in engine velocity order: planar [x, y, theta], spherical [rx, ry, rz], free [rx, ry, rz, x, y, z]
(angular first, Featherstone); the adapter maps free-joint generalized forces back to the neutral linear-first order.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from mechsim.physics.neutral import (
    WORLD,
    BeltPulley,
    Body,
    Box,
    Capsule,
    ConvexHull,
    Cylinder,
    GearPair,
    HingeConstraint,
    LeadScrew,
    LumpedMember,
    PhysicsModel,
    Plane,
    Rope,
    Sphere,
)

FORMAT = "mechsim-native-1"
SUPPORTED_ELEMENTS: tuple[str, ...] = (
    "body",
    "joint.revolute",
    "joint.prismatic",
    "joint.fixed",
    "joint.planar",
    "joint.spherical",
    "joint.free",
    "shape.sphere",
    "shape.capsule",
    "shape.box",
    "shape.cylinder",
    "shape.convex_hull",
    "shape.plane",
    "constraint.point",
    "constraint.hinge",
    "contacts",
    "lumped_member",
    "transmission.belt_pulley",
    "transmission.lead_screw",
    "transmission.gear_pair",
    "rope",
)
WORLD_ANCHOR = "world.anchor"
SLIP_VELOCITY = 1e-4
WELD_FREQUENCY = 1000.0
SNATCH_LIMIT = 0.2
CLAMPED_MIN_SEGMENTS = 8


class UnmappableElementError(ValueError):
    """An element the own engine cannot carry exactly (PHY-019), naming the element."""

    def __init__(self, element: str, reason: str) -> None:
        super().__init__(f"{element}: {reason}")
        self.element = element
        self.reason = reason


class StartStateError(ValueError):
    """A start state the native adapter cannot place."""


@dataclass(frozen=True)
class Translation:
    document: dict[str, Any]
    report: tuple[str, ...]

    @property
    def text(self) -> str:
        return json.dumps(self.document, sort_keys=True, separators=(",", ":"), allow_nan=False)


def supported_elements() -> tuple[str, ...]:
    """Element kinds the translation maps; mirrors `supported_kinds` in core/physics/native/src/native_adapter.cpp."""
    return SUPPORTED_ELEMENTS


def element_kinds(model: PhysicsModel) -> tuple[str, ...]:
    kinds: list[str] = ["body"]
    for body in model.bodies:
        if body.joint is not None:
            kinds.append(f"joint.{body.joint.type}")
        kinds += [f"shape.{shape.shape}" for shape in body.shapes]
    kinds += [f"shape.{shape.shape}" for shape in model.world_shapes]
    kinds += [f"constraint.{constraint.type}" for constraint in model.constraints]
    if model.contacts is not None:
        kinds.append("contacts")
    if len(model.lumped_members) > 0:
        kinds.append("lumped_member")
    kinds += [f"transmission.{transmission.type}" for transmission in model.transmissions]
    if len(model.ropes) > 0:
        kinds.append("rope")
    return tuple(dict.fromkeys(kinds))


def _floats(values: Sequence[float]) -> list[float]:
    out = [float(value) for value in values]
    for value in out:
        if not math.isfinite(value):
            raise UnmappableElementError(f"value {value!r}", "not finite")
    return out


# ===== Joints ===========================================================


def _dofs(body: Body) -> list[Any]:
    joint = body.joint
    if joint is None:
        raise UnmappableElementError(f"body '{body.name}'", "no joint")
    if joint.type == "free":
        return [*joint.angular, *joint.linear]
    return [*joint.linear, *joint.angular]


def _joint(body: Body) -> dict[str, Any]:
    joint = body.joint
    if joint is None:
        raise UnmappableElementError(f"body '{body.name}'", "no joint")
    dofs = _dofs(body)
    coordinate = joint.type in ("revolute", "prismatic", "planar")
    stiffness: list[float] = []
    rest: list[float] = []
    damping: list[float] = []
    friction: list[float] = []
    lower: list[float | None] = []
    upper: list[float | None] = []
    if len(dofs) > 0:
        damping = _floats([dof.damping for dof in dofs])
        if coordinate:
            stiffness = _floats([dof.stiffness for dof in dofs])
            rest = _floats([dof.rest for dof in dofs])
            friction = _floats([dof.friction for dof in dofs])
            lower = [dof.lower for dof in dofs]
            upper = [dof.upper for dof in dofs]
        elif any(dof.friction != 0.0 for dof in dofs):
            raise UnmappableElementError(
                f"joint '{joint.name}'", f"friction on a {joint.type} joint: the own engine has no ball-joint friction"
            )
        elif any(dof.stiffness != 0.0 for dof in dofs):
            raise UnmappableElementError(
                f"joint '{joint.name}'",
                f"stiffness on a {joint.type} joint: the own engine's joint springs act on revolute, prismatic and "
                "planar coordinates only (PHY-019)",
            )
        elif any(dof.lower is not None or dof.upper is not None for dof in dofs):
            raise UnmappableElementError(
                f"joint '{joint.name}'",
                f"limits on a {joint.type} joint: the own engine's joint limits act on revolute, prismatic and planar "
                "coordinates only (PHY-019)",
            )
        if not any(value != 0.0 for value in damping):
            damping = []
    return {
        "type": joint.type,
        "position": _floats(joint.position),
        "orientation": _floats(joint.orientation),
        "axis": _floats(joint.axis),
        "stiffness": stiffness,
        "damping": damping,
        "rest": rest,
        "friction": friction,
        "lower": lower,
        "upper": upper,
    }


def _body(body: Body, parent: int) -> dict[str, Any]:
    i = body.inertia
    return {
        "name": body.name,
        "parent": parent,
        "mass": float(body.mass),
        "com": _floats(body.com),
        "inertia": _floats([i.ixx, i.ixy, i.ixz, i.ixy, i.iyy, i.iyz, i.ixz, i.iyz, i.izz]),
        "joint": _joint(body),
    }


def _anchor() -> dict[str, Any]:
    return {
        "name": WORLD_ANCHOR,
        "parent": -1,
        "mass": 1.0,
        "com": [0.0, 0.0, 0.0],
        "inertia": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "joint": {
            "type": "fixed",
            "position": [0.0, 0.0, 0.0],
            "orientation": [1.0, 0.0, 0.0, 0.0],
            "axis": [0.0, 0.0, 1.0],
            "stiffness": [],
            "damping": [],
            "rest": [],
            "friction": [],
            "lower": [],
            "upper": [],
        },
    }


# ===== Shapes and contacts ==============================================


def _geometry(shape: Any, body: int) -> dict[str, Any]:
    geometry: dict[str, Any] = {
        "kind": shape.shape,
        "body": body,
        "position": _floats(shape.position),
        "orientation": _floats(shape.orientation),
        "radius": 0.0,
        "length": 0.0,
        "size": [0.0, 0.0, 0.0],
        "vertices": [],
        "material": shape.material,
    }
    if isinstance(shape, Sphere):
        geometry["radius"] = float(shape.radius)
    elif isinstance(shape, Capsule | Cylinder):
        geometry["radius"] = float(shape.radius)
        geometry["length"] = float(shape.length)
    elif isinstance(shape, Box):
        geometry["size"] = _floats(shape.size)
    elif isinstance(shape, ConvexHull):
        geometry["vertices"] = [_floats(vertex) for vertex in shape.vertices]
    elif not isinstance(shape, Plane):
        raise UnmappableElementError(f"shape {shape!r}", "unknown shape type")
    return geometry


def _contact(model: PhysicsModel, index: Mapping[str, int], report: list[str]) -> dict[str, Any] | None:
    contacts = model.contacts
    if contacts is None:
        return None
    geometries: list[dict[str, Any]] = [_geometry(shape, -1) for shape in model.world_shapes]
    for body in model.bodies:
        geometries += [_geometry(shape, index[body.name]) for shape in body.shapes]
    exclusions: list[list[int]] = []
    for a, b in contacts.exclusions:
        exclusions.append([index.get(a, -1), index.get(b, -1)])
    joined: list[str] = []
    for body in model.bodies:
        if body.joint is None or body.joint.type == "free":
            continue
        pair = [index.get(body.parent, -1), index[body.name]]
        if pair not in exclusions and [pair[1], pair[0]] not in exclusions:
            exclusions.append(pair)
            joined.append(f"{body.parent}-{body.name}")
    if len(joined) > 0:
        report.append(
            "contact: pairs on bodies joined by a non-free joint are excluded, as on the bridge: " + ", ".join(joined)
        )
    kinds = {geometry["kind"] for geometry in geometries}
    if len(kinds & {"box", "convex_hull"}) > 0:
        report.append(
            "contact: box-box and hull-hull pairs carry one witness point (ruling 23; a box on a box tips; shapes on "
            "a plane use one contact per corner or vertex)"
        )
    report.append(
        f"contact: penalty spring-damper per pair (K_n, C_n, mu), friction regularised below {SLIP_VELOCITY} m/s"
    )
    return {
        "default": {
            "stiffness": float(contacts.default.stiffness),
            "damping": float(contacts.default.damping),
            "friction": float(contacts.default.friction),
        },
        "pairs": [
            {
                "material_a": pair.materials[0],
                "material_b": pair.materials[1],
                "stiffness": float(pair.stiffness),
                "damping": float(pair.damping),
                "friction": float(pair.friction),
            }
            for pair in contacts.pairs
        ],
        "exclusions": exclusions,
        "slip_velocity": SLIP_VELOCITY,
        "geometries": geometries,
    }


# ===== Members, ropes and transmissions =================================


def _ends(member: LumpedMember) -> str:
    if member.root == "clamped" and member.end == "free":
        return "cantilever"
    if member.root == "pinned" and member.end == "roller":
        return "simply_supported"
    if member.root == "clamped" and member.end == "clamped":
        return "clamped"
    raise UnmappableElementError(
        f"lumped member '{member.name}'",
        f"supports ({member.root}, {member.end}): the own engine's lumped beam is cantilever, pinned-roller or "
        "clamped-clamped",
    )


def member_node_names(model: PhysicsModel) -> tuple[str, ...]:
    """Names of the member node poses the native adapter publishes after the neutral bodies in
    `PhysicsOutputs.bodies`: each lumped member in neutral order, its chain bodies root to tip, `<member>.node<k>`
    (k from 0; a pinned root's spring-free pin segment is node 0, so a pinned member has segments + 1 nodes). Each
    pose is the segment body's frame: origin at its joint on the member axis, x along the member."""
    names: list[str] = []
    for member in model.lumped_members:
        count = int(member.segments)
        if member.root == "pinned":
            count += 1
        names.extend(f"{member.name}.node{k}" for k in range(count))
    return tuple(names)


def _member(member: LumpedMember, parent: int, h: float, report: list[str]) -> dict[str, Any]:
    section = member.section
    ends = _ends(member)
    flexural = float(section.elastic_modulus) * float(section.second_moment_of_area)
    mass_per_length = float(section.density) * float(section.area)
    beta = 1.875104
    if ends == "simply_supported":
        beta = math.pi
    if ends == "clamped":
        beta = 4.730041
    omega = (beta / float(section.length)) ** 2 * math.sqrt(flexural / mass_per_length)
    x = omega * h
    report.append(
        f"lumped member '{member.name}': {member.segments} segments, {ends}; first mode {omega / (2 * math.pi):.6g} Hz "
        f"integrates as backward Euler: amplitude loss pi omega h = {math.pi * x:.3g} per period, frequency "
        f"{x * x / 3:.3g} low (ruling 21)"
    )
    return {
        "parent": parent,
        "position": _floats(member.position),
        "orientation": _floats(member.orientation),
        "youngs_modulus": float(section.elastic_modulus),
        "area_moment": float(section.second_moment_of_area),
        "density": float(section.density),
        "area": float(section.area),
        "length": float(section.length),
        "segments": int(member.segments),
        "node_damping": float(member.node_damping),
        "ends": ends,
        "roller_mass_fraction": float(member.roller_mass_fraction),
    }


def _rope(rope: Rope, index: Mapping[str, int], model: PhysicsModel, h: float, report: list[str]) -> dict[str, Any]:
    gravity = _gravity(model)
    length = float(rope.rest_length)
    swing = math.sqrt(gravity / length) * h
    report.append(
        f"rope '{rope.name}': lumped chain of {rope.segments} segments (ADR 0016); swing mode pi omega h = "
        f"{math.pi * swing:.3g} amplitude loss per period, frequency {swing * swing / 3:.3g} low, omega = sqrt(g / L) "
        "(ruling 21); the neutral rope carries no attachment damping or friction: 0"
    )
    if rope.to_body != WORLD:
        load = next(body.mass for body in model.bodies if body.name == rope.to_body)
        snatch = math.sqrt(float(rope.axial_stiffness) / (length * float(load))) * h
        if snatch > SNATCH_LIMIT:
            report.append(
                f"rope '{rope.name}': snatch omega_s h = {snatch:.3g} above {SNATCH_LIMIT}: the first snatch peak "
                "reads about 15 percent low (ruling 22)"
            )
    breaking: float | None = None
    if rope.breaking_tension is not None:
        breaking = float(rope.breaking_tension)
    return {
        "from_body": index.get(rope.from_body, -1),
        "from_point": _floats(rope.from_point),
        "to_body": index.get(rope.to_body, -1),
        "to_point": _floats(rope.to_point),
        "segments": int(rope.segments),
        "axial_rigidity": float(rope.axial_stiffness),
        "damping": float(rope.damping),
        "mass_per_length": float(rope.mass_per_length),
        "rest_length": length,
        "breaking_tension": breaking,
        "attachment_damping": 0.0,
        "attachment_friction": 0.0,
    }


def _gravity(model: PhysicsModel) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in model.gravity))


def _subtree_mass(model: PhysicsModel, root: str) -> float:
    names = {root}
    total = 0.0
    for body in model.bodies:
        if body.name in names or body.parent in names:
            names.add(body.name)
            total += float(body.mass)
    return total


def _transmission(
    transmission: Any, model: PhysicsModel, joints: Mapping[str, int], h: float, report: list[str]
) -> dict[str, Any]:
    ratio = 0.0
    reverses = False
    self_locking = False
    if isinstance(transmission, BeltPulley):
        ratio = float(transmission.pulley_radius)
    elif isinstance(transmission, LeadScrew):
        ratio = float(transmission.lead)
        self_locking = bool(transmission.self_locking)
    elif isinstance(transmission, GearPair):
        ratio = abs(float(transmission.ratio))
        reverses = float(transmission.ratio) < 0.0
    else:
        raise UnmappableElementError(f"transmission {transmission!r}", "unknown transmission type")
    driven = model.bodies[joints[transmission.driven_joint]]
    stiffness = float(transmission.stiffness)
    if isinstance(transmission, GearPair):
        mode = "the output inertia"
    else:
        omega = math.sqrt(stiffness / _subtree_mass(model, driven.name))
        x = omega * h
        mode = (
            f"omega = sqrt(k / m) = {omega:.6g} rad/s on the driven mass: amplitude loss pi omega h = "
            f"{math.pi * x:.3g} per period, frequency {x * x / 3:.3g} low"
        )
    report.append(
        f"transmission '{transmission.name}': compliant, stiffness {stiffness:.6g} at the output, no damping (ruling "
        f"10); efficiency {float(transmission.efficiency):.6g} both directions; mode on {mode} (backward Euler, "
        "ruling 21)"
    )
    return {
        "type": transmission.type,
        "input_body": joints[transmission.shaft_joint],
        "output_body": joints[transmission.driven_joint],
        "ratio": ratio,
        "reverses": reverses,
        "efficiency": float(transmission.efficiency),
        "back_efficiency": float(transmission.efficiency),
        "self_locking": self_locking,
        "stiffness": stiffness,
        "damping": 0.0,
        "backlash": float(transmission.backlash),
        "slip": float(transmission.slip),
    }


def output_per_input(transmission: Any) -> float:
    """Driven travel per shaft radian, the engine's n (`output_per_input`): r, lead / 2 pi, 1 / ratio (signed)."""
    if isinstance(transmission, BeltPulley):
        return float(transmission.pulley_radius)
    if isinstance(transmission, LeadScrew):
        return float(transmission.lead) / (2.0 * math.pi)
    return 1.0 / float(transmission.ratio)


def _shaft_starts(model: PhysicsModel, starts: dict[str, float]) -> dict[str, float]:
    """A shaft starts where its driven joint does, theta = x / n, so the coupling starts unstretched (a shaft left at
    zero under a driven start of 0.1 m would stretch a belt by 0.1 m); an explicit shaft start is kept."""
    for transmission in model.transmissions:
        if transmission.driven_joint in starts and transmission.shaft_joint not in starts:
            starts[transmission.shaft_joint] = starts[transmission.driven_joint] / output_per_input(transmission)
    return starts


# ===== Model ============================================================


def translate(
    model: PhysicsModel,
    step: float | None = None,
    payloads: Mapping[str, str] | None = None,
    starts: Mapping[str, float] | None = None,
) -> Translation:
    """The engine model document and the approximation report (SCO-013); `step` (s) is the run's physics step when
    it is not the model's, `payloads` maps payload bodies to carriers, `starts` single-DOF joints to coordinates."""
    h = float(model.run.step)
    if step is not None:
        h = float(step)
    report: list[str] = [
        f"native adapter: own engine, backward Euler on springs, dampers and contact (SemiImplicitEuler, ADR 0013) "
        f"at {h:.6g} s"
    ]
    index = {body.name: position for position, body in enumerate(model.bodies)}
    joints = {body.joint.name: position for position, body in enumerate(model.bodies) if body.joint is not None}
    bodies = [_body(body, index.get(body.parent, -1)) for body in model.bodies]
    anchor = -1
    if any(WORLD in (constraint.body_a, constraint.body_b) for constraint in model.constraints):
        anchor = len(bodies)
        bodies.append(_anchor())
    index_world = dict(index)
    index_world[WORLD] = anchor
    constraints: list[dict[str, Any]] = []
    for constraint in model.constraints:
        entry: dict[str, Any] = {
            "type": constraint.type,
            "body_a": index_world[constraint.body_a],
            "body_b": index_world[constraint.body_b],
            "point_a": _floats(constraint.anchor_a),
            "point_b": _floats(constraint.anchor_b),
        }
        if isinstance(constraint, HingeConstraint):
            entry["axis_a"] = _floats(constraint.axis_a)
            entry["axis_b"] = _floats(constraint.axis_b)
        constraints.append(entry)
    for body in model.bodies:
        joint = body.joint
        if joint is None:
            continue
        dofs = _dofs(body)
        if any(dof.friction > 0.0 for dof in dofs):
            report.append(f"joint '{joint.name}': Coulomb friction regularised below {SLIP_VELOCITY} m/s or rad/s")
        if any(dof.lower is not None or dof.upper is not None for dof in dofs):
            report.append(
                f"joint '{joint.name}': limits as an implicit one-sided spring-damper critically damped at "
                f"w = 1 / (2 h); static penetration under gravity 4 g h^2 = {4 * _gravity(model) * h * h:.3g} m"
            )
    drivable = [transmission.shaft_joint for transmission in model.transmissions] + list(model.signals.drive_joints)
    for name in dict.fromkeys(drivable):
        report.append(
            f"joint drive on '{name}': implicit spring-damper (backward Euler, no stability limit, PHY-034); with "
            "x = omega h, omega = sqrt(K / m), the drive mode runs x^2 / 3 slow and loses about pi x of its "
            f"amplitude per period (keeps (1 + x^2)^(-pi / atan x), about e^(-pi x); h = {h:.6g} s; K is the "
            "controller's, so x is known only at run time; ruling 21)"
        )
    members = [_member(member, index.get(member.parent, -1), h, report) for member in model.lumped_members]
    ropes = [_rope(rope, index, model, h, report) for rope in model.ropes]
    transmissions = [_transmission(item, model, joints, h, report) for item in model.transmissions]
    contact = _contact(model, index, report)
    payload_entries: list[dict[str, int]] = []
    for payload, carrier in sorted((payloads or {}).items()):
        if payload not in index or carrier not in index or payload == carrier:
            raise UnmappableElementError(f"payload '{payload}'", f"carrier '{carrier}' is not another body")
        body = model.bodies[index[payload]]
        if body.joint is None or body.joint.type != "free" or body.parent != WORLD:
            raise UnmappableElementError(f"payload '{payload}'", "not a free body under world")
        payload_entries.append({"body": index[payload], "carrier": index[carrier]})
        report.append(
            f"payload '{payload}': weld as an implicit 6-DOF spring-damper critically damped at {WELD_FREQUENCY:.0f} "
            f"Hz (ruling 23), static sag g / w^2 = {_gravity(model) / (2 * math.pi * WELD_FREQUENCY) ** 2:.3g} m"
        )
    start_entries: list[dict[str, Any]] = []
    chosen = _shaft_starts(model, dict(starts or {}))
    for name in sorted(chosen):
        if name not in joints:
            raise StartStateError(f"joint '{name}': no such joint in the physics model")
        body = model.bodies[joints[name]]
        if body.joint is None or body.joint.type not in ("revolute", "prismatic"):
            kind = "?"
            if body.joint is not None:
                kind = body.joint.type
            raise StartStateError(f"joint '{name}': a {kind} joint takes no single start coordinate")
        start_entries.append({"body": joints[name], "value": float(chosen[name])})
    run = model.run
    document: dict[str, Any] = {
        "format": FORMAT,
        "elements": list(element_kinds(model)),
        "gravity": _floats(model.gravity),
        "config": {
            "iteration_cap": int(run.iteration_cap),
            "velocity_tolerance": min(float(run.velocity_tolerance_linear), float(run.velocity_tolerance_angular)),
            "projection_iterations": int(run.projection_iterations),
            "drift_threshold_m": float(run.drift_threshold_linear),
            "drift_threshold_rad": float(run.drift_threshold_angular),
            "unit_tolerance": float(run.unit_quaternion_tolerance),
            "symmetry_tolerance": float(run.symmetry_tolerance),
        },
        "bodies": bodies,
        "neutral_bodies": len(model.bodies),
        "constraints": constraints,
        "members": members,
        "ropes": ropes,
        "transmissions": transmissions,
        "contact": contact,
        "payloads": payload_entries,
        "starts": start_entries,
    }
    for kind in document["elements"]:
        if kind not in SUPPORTED_ELEMENTS:
            raise UnmappableElementError(kind, "not mapped by the native adapter (PHY-019)")
    return Translation(document, tuple(report))
