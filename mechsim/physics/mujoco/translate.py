"""Neutral physics model to MJCF text, element by element (ADR 0014, PHY-013, PHY-017, PHY-018).

Build-time only; the C++ stepping adapter in `core/physics/mujoco/` compiles the text this module emits. The mapping
table is in `mechsim/physics/mujoco/CLAUDE.md`. Every element this module cannot map exactly raises
`UnmappableElementError` naming the element (PHY-019, invariant 7); nothing is approximated silently.

MJCF names: neutral names verbatim for bodies, joints, constraints and transmissions; generated names join the
neutral name and a suffix with "." (never in a neutral name), so they cannot collide. A `<custom>` block carries the
neutral element order and kinds the C++ adapter needs to publish outputs in neutral-model file order; it adds no
physics.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass

from mechsim.physics.neutral import (
    WORLD,
    AngularDof,
    BeltPulley,
    Body,
    Box,
    Capsule,
    ContactModel,
    ContactParameters,
    ConvexHull,
    Cylinder,
    GearPair,
    HingeConstraint,
    Joint,
    LeadScrew,
    LinearDof,
    LumpedMember,
    PhysicsModel,
    Plane,
    PointConstraint,
    Sphere,
)

# Placeholders of a drive's soft equality; the adapter sets both every step it is enabled (PHY-034).
DRIVE_SOLREF = "-1 -1"
DRIVE_SOLIMP = "0.0001 0.0001 0.001 0.5 2"

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
)

# A hinge constraint is two MuJoCo connect constraints: one at the anchors and one this far along the axes, which
# together remove the five relative DOFs (the axial row of the second is redundant with the first).
HINGE_AXIS_OFFSET = 0.1  # m

# MuJoCo's soft-constraint impedance for contacts. With impedance d the constrained acceleration is
# d a_ref + (1 - d) a_0, so a_ref = -(K / m) r - (C / m) v reproduces a spring K and damper C on the pair's reduced
# mass m to within 1 - d.
CONTACT_IMPEDANCE = 0.99

# Equality and limit constraints use MuJoCo's reference time constant at its stability floor, two steps, critically
# damped: the stiffest setting MuJoCo's integrator safety (refsafe) accepts.
REFERENCE_STEPS = 2.0


class UnmappableElementError(ValueError):
    """A neutral element the MuJoCo translation cannot map; `element` names it (PHY-017, PHY-019)."""

    def __init__(self, element: str, reason: str) -> None:
        super().__init__(f"MuJoCo translation cannot map {element}: {reason}")
        self.element = element
        self.reason = reason


def supported_elements() -> tuple[str, ...]:
    """Element kinds the translation maps; mirrors the C++ adapter's list (PHY-019)."""
    return SUPPORTED_ELEMENTS


def _num(value: float) -> str:
    if not math.isfinite(value):
        raise UnmappableElementError(f"value {value!r}", "not finite")
    if value == 0.0:
        return "0"
    return repr(float(value))


def _vec(values: Iterable[float]) -> str:
    return " ".join(_num(value) for value in values)


def _plane_basis(normal: Sequence[float]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Orthonormal u, w with u x w = n, chosen as the own engine chooses them (ENG-009)."""
    norm = math.sqrt(sum(component * component for component in normal))
    n = (normal[0] / norm, normal[1] / norm, normal[2] / norm)
    magnitudes = [abs(component) for component in n]
    smallest = magnitudes.index(min(magnitudes))
    unit = [0.0, 0.0, 0.0]
    unit[smallest] = 1.0
    u = _cross(n, (unit[0], unit[1], unit[2]))
    u_norm = math.sqrt(sum(component * component for component in u))
    u = (u[0] / u_norm, u[1] / u_norm, u[2] / u_norm)
    return u, _cross(n, u)


def _cross(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _rotate(quaternion: Sequence[float], vector: Sequence[float]) -> tuple[float, float, float]:
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    t = _cross((x, y, z), vector)
    t = (2.0 * t[0], 2.0 * t[1], 2.0 * t[2])
    u = _cross((x, y, z), t)
    return (vector[0] + w * t[0] + u[0], vector[1] + w * t[1] + u[1], vector[2] + w * t[2] + u[2])


# ===== Joints ===========================================================


def _dof_attributes(
    element: ET.Element, joint: Joint, dof: LinearDof | AngularDof | None, springs: bool, limit_ref: str
) -> None:
    if dof is None:
        return
    if (dof.lower is None) != (dof.upper is None):
        raise UnmappableElementError(f"joint '{joint.name}'", "a one-sided limit; MuJoCo joint ranges take both bounds")
    if dof.lower is not None and dof.upper is not None:
        element.set("limited", "true")
        element.set("range", _vec((dof.lower, dof.upper)))
        element.set("solreflimit", limit_ref)
    if dof.friction != 0.0:
        element.set("frictionloss", _num(dof.friction))
    if dof.damping != 0.0:
        element.set("damping", _num(dof.damping))
    if springs and dof.stiffness != 0.0:
        element.set("stiffness", _num(dof.stiffness))
    if springs and dof.rest != 0.0:
        element.set("springref", _num(dof.rest))


def _uniform_dofs(joint: Joint) -> LinearDof | AngularDof | None:
    dofs: list[LinearDof | AngularDof] = [*joint.linear, *joint.angular]
    if not dofs:
        return None
    first = dofs[0]
    for dof in dofs[1:]:
        if dof.damping != first.damping or dof.friction != first.friction:
            raise UnmappableElementError(
                f"joint '{joint.name}'",
                f"per-DOF damping or friction on a {joint.type} joint; MuJoCo takes one value per joint",
            )
    return first


def _optional_dof[Dof: (LinearDof, AngularDof)](dofs: tuple[Dof, ...], index: int) -> Dof | None:
    if dofs:
        return dofs[index]
    return None


def _joint(body: ET.Element, joint: Joint, limit_ref: str) -> None:
    if joint.type == "fixed":
        return
    if joint.type in ("revolute", "prismatic"):
        mj_type = "hinge"
        dof: LinearDof | AngularDof | None = _optional_dof(joint.angular, 0)
        if joint.type == "prismatic":
            mj_type = "slide"
            dof = _optional_dof(joint.linear, 0)
        element = ET.SubElement(body, "joint", {"name": joint.name, "type": mj_type, "axis": _vec(joint.axis)})
        _dof_attributes(element, joint, dof, True, limit_ref)
        return
    if joint.type == "planar":
        u, w = _plane_basis(joint.axis)
        axes = (
            (f"{joint.name}.x", "slide", u),
            (f"{joint.name}.y", "slide", w),
            (f"{joint.name}.theta", "hinge", None),
        )
        dofs: list[LinearDof | AngularDof | None] = [
            _optional_dof(joint.linear, 0),
            _optional_dof(joint.linear, 1),
            _optional_dof(joint.angular, 0),
        ]
        for (name, mj_type, axis), planar_dof in zip(axes, dofs, strict=True):
            direction: Sequence[float] = joint.axis
            if axis is not None:
                direction = axis
            element = ET.SubElement(body, "joint", {"name": name, "type": mj_type, "axis": _vec(direction)})
            _dof_attributes(element, joint, planar_dof, True, limit_ref)
        return
    if joint.type in ("spherical", "free"):
        mj_type = "ball"
        if joint.type == "free":
            mj_type = "free"
        element = ET.SubElement(body, "joint", {"name": joint.name, "type": mj_type})
        _dof_attributes(element, joint, _uniform_dofs(joint), False, limit_ref)
        return
    raise UnmappableElementError(f"joint '{joint.name}'", f"joint type '{joint.type}'")


# ===== Shapes ===========================================================

ShapeType = Sphere | Capsule | Box | Cylinder | ConvexHull | Plane


def _shape_kind(shape: object) -> str:
    if isinstance(shape, Sphere | Capsule | Box | Cylinder | ConvexHull | Plane):
        return f"shape.{shape.shape}"
    raise UnmappableElementError(f"shape {shape!r}", "unknown shape type")


def _geom(parent: ET.Element, assets: ET.Element, owner: str, index: int, shape: ShapeType) -> ET.Element:
    name = f"{owner}.shape{index}"
    attributes = {"name": name, "pos": _vec(shape.position), "quat": _vec(shape.orientation)}
    if isinstance(shape, Sphere):
        attributes |= {"type": "sphere", "size": _num(shape.radius)}
    elif isinstance(shape, Capsule):
        attributes |= {"type": "capsule", "size": _vec((shape.radius, 0.5 * shape.length))}
    elif isinstance(shape, Box):
        attributes |= {"type": "box", "size": _vec(0.5 * edge for edge in shape.size)}
    elif isinstance(shape, Cylinder):
        attributes |= {"type": "cylinder", "size": _vec((shape.radius, 0.5 * shape.length))}
    elif isinstance(shape, ConvexHull):
        vertices = " ".join(_vec(vertex) for vertex in shape.vertices)
        ET.SubElement(assets, "mesh", {"name": name, "vertex": vertices})
        attributes |= {"type": "mesh", "mesh": name}
    else:
        attributes |= {"type": "plane", "size": "0 0 1"}
    attributes |= {"contype": "0", "conaffinity": "0"}
    return ET.SubElement(parent, "geom", attributes)


# ===== Lumped members ===================================================


def _member_segment(
    parent: ET.Element, name: str, member: LumpedMember, span: float, position: Sequence[float], quat: Sequence[float]
) -> ET.Element:
    section = member.section
    mass = section.density * section.area * span
    transverse = mass * span * span / 12.0
    axial = 2.0 * section.density * section.second_moment_of_area * span
    body = ET.SubElement(parent, "body", {"name": name, "pos": _vec(position), "quat": _vec(quat)})
    ET.SubElement(
        body,
        "inertial",
        {"pos": _vec((0.5 * span, 0.0, 0.0)), "mass": _num(mass), "diaginertia": _vec((axial, transverse, transverse))},
    )
    return body


def _member(parents: dict[str, ET.Element], equality: ET.Element, member: LumpedMember, eq_ref: str) -> None:
    """Midpoint-spring chain of the own engine (ENG-022): spring E I / l at x_j = (j + 1/2) l about root z."""
    section = member.section
    length = section.length / member.segments
    stiffness = section.elastic_modulus * section.second_moment_of_area / length
    root = parents[member.parent]
    previous = root
    previous_span = 0.0
    if member.root == "pinned":
        pin = _member_segment(root, f"{member.name}.pin", member, 0.5 * length, member.position, member.orientation)
        joint = {"name": f"{member.name}.pin", "type": "hinge", "axis": "0 0 1"}
        if member.node_damping != 0.0:
            joint["damping"] = _num(member.node_damping)
        ET.SubElement(pin, "joint", joint)
        previous = pin
        previous_span = 0.5 * length
    for j in range(member.segments):
        span = length
        if j == member.segments - 1:
            span = 0.5 * length
        position: Sequence[float] = (previous_span, 0.0, 0.0)
        quat: Sequence[float] = (1.0, 0.0, 0.0, 0.0)
        if previous is root:
            offset = _rotate(member.orientation, ((j + 0.5) * length, 0.0, 0.0))
            position = tuple(p + o for p, o in zip(member.position, offset, strict=True))
            quat = member.orientation
        body = _member_segment(previous, f"{member.name}.segment{j}", member, span, position, quat)
        joint = {"name": f"{member.name}.node{j}", "type": "hinge", "axis": "0 0 1", "stiffness": _num(stiffness)}
        if member.node_damping != 0.0:
            joint["damping"] = _num(member.node_damping)
        ET.SubElement(body, "joint", joint)
        previous = body
        previous_span = span
    if member.end == "roller":
        roller_mass = member.roller_mass_fraction * section.density * section.area * section.length
        roller_inertia = roller_mass * length * length
        end = _rotate(member.orientation, (section.length, 0.0, 0.0))
        roller_position = tuple(p + o for p, o in zip(member.position, end, strict=True))
        roller = ET.SubElement(
            root,
            "body",
            {"name": f"{member.name}.roller", "pos": _vec(roller_position), "quat": _vec(member.orientation)},
        )
        ET.SubElement(
            roller,
            "inertial",
            {"pos": "0 0 0", "mass": _num(roller_mass), "diaginertia": _vec((roller_inertia,) * 3)},
        )
        ET.SubElement(roller, "joint", {"name": f"{member.name}.roller", "type": "slide", "axis": "1 0 0"})
        ET.SubElement(
            equality,
            "connect",
            {
                "name": f"{member.name}.roller",
                "body1": f"{member.name}.segment{member.segments - 1}",
                "body2": f"{member.name}.roller",
                "anchor": _vec((previous_span, 0.0, 0.0)),
                "solref": eq_ref,
            },
        )


# ===== Constraints and transmissions ====================================


def _connect(
    equality: ET.Element,
    parents: dict[str, ET.Element],
    name: str,
    bodies: tuple[str, str],
    anchors: tuple[Sequence[float], Sequence[float]],
    eq_ref: str,
) -> None:
    """MuJoCo connect between two sites, so both neutral anchors hold exactly (an anchor form would infer body_b's)."""
    sites = (f"{name}.a", f"{name}.b")
    for site, body, anchor in zip(sites, bodies, anchors, strict=True):
        ET.SubElement(parents[body], "site", {"name": site, "pos": _vec(anchor)})
    ET.SubElement(equality, "connect", {"name": name, "site1": sites[0], "site2": sites[1], "solref": eq_ref})


def _constraint(equality: ET.Element, parents: dict[str, ET.Element], constraint: object, eq_ref: str) -> str:
    if isinstance(constraint, PointConstraint):
        bodies = (constraint.body_a, constraint.body_b)
        _connect(equality, parents, constraint.name, bodies, (constraint.anchor_a, constraint.anchor_b), eq_ref)
        return "constraint.point"
    if isinstance(constraint, HingeConstraint):
        bodies = (constraint.body_a, constraint.body_b)
        _connect(
            equality, parents, f"{constraint.name}.point", bodies, (constraint.anchor_a, constraint.anchor_b), eq_ref
        )
        axis_anchors = (_along(constraint.anchor_a, constraint.axis_a), _along(constraint.anchor_b, constraint.axis_b))
        _connect(equality, parents, f"{constraint.name}.axis", bodies, axis_anchors, eq_ref)
        return "constraint.hinge"
    raise UnmappableElementError(f"constraint {_element_name(constraint)}", "unknown constraint type")


def _along(anchor: Sequence[float], axis: Sequence[float]) -> tuple[float, float, float]:
    norm = math.sqrt(sum(component * component for component in axis))
    scale = HINGE_AXIS_OFFSET / norm
    return (anchor[0] + scale * axis[0], anchor[1] + scale * axis[1], anchor[2] + scale * axis[2])


def _element_name(element: object) -> str:
    name = getattr(element, "name", None)
    if isinstance(name, str):
        return f"'{name}'"
    return repr(element)


@dataclass(frozen=True)
class _Reflection:
    """A transmission whose shaft is folded into its driven joint (exact for a rigid fixed ratio)."""

    transmission: str
    kind: str
    coefficient: float
    efficiency: float
    inertia: float
    shaft: Body
    driven_joint: str
    backlash: float  # dead band in shaft units (rad), the neutral output-side backlash over |c|


def _coefficient(transmission: object) -> float:
    """c in driven = c * shaft: pulley radius, lead / 2 pi, 1 / ratio."""
    if isinstance(transmission, BeltPulley):
        return transmission.pulley_radius
    if isinstance(transmission, LeadScrew):
        return transmission.lead / (2.0 * math.pi)
    if isinstance(transmission, GearPair):
        return 1.0 / transmission.ratio
    raise UnmappableElementError(f"transmission {_element_name(transmission)}", "unknown transmission type")


def _axis_inertia(body: Body, axis: Sequence[float]) -> float:
    """Inertia about the joint axis through the body origin: a . I_com a + m (|com|^2 - (com . a)^2)."""
    norm = math.sqrt(sum(component * component for component in axis))
    a = [component / norm for component in axis]
    i = body.inertia
    tensor = ((i.ixx, i.ixy, i.ixz), (i.ixy, i.iyy, i.iyz), (i.ixz, i.iyz, i.izz))
    about_com = sum(a[r] * tensor[r][k] * a[k] for r in range(3) for k in range(3))
    along = sum(c * u for c, u in zip(body.com, a, strict=True))
    offset = sum(c * c for c in body.com) - along * along
    return about_com + body.mass * offset


def _reflections(model: PhysicsModel, report: list[str]) -> dict[str, _Reflection]:
    """Shaft bodies by name. Refuses what a rigid reflection cannot carry exactly; records every approximation."""
    owners = {body.joint.name: body for body in model.bodies if body.joint is not None}
    driven_joints = {transmission.driven_joint for transmission in model.transmissions}
    reflected: dict[str, _Reflection] = {}
    for transmission in model.transmissions:
        coefficient = _coefficient(transmission)
        element = f"transmission '{transmission.name}'"
        if transmission.slip != 0.0:
            raise UnmappableElementError(
                element, "a static slip parameter; slip is a fault the adapter injects (set_transmission_slip, PHY-006)"
            )
        if isinstance(transmission, LeadScrew) and transmission.self_locking:
            raise UnmappableElementError(element, "self-locking; the own engine carries it (milestone 1)")
        shaft = owners[transmission.shaft_joint]
        assert shaft.joint is not None
        if shaft.name in reflected:
            raise UnmappableElementError(element, f"shaft body '{shaft.name}' already drives another transmission")
        if transmission.shaft_joint in driven_joints:
            raise UnmappableElementError(element, f"shaft joint '{transmission.shaft_joint}' is itself driven")
        dependants = [body.name for body in model.bodies if body.parent == shaft.name]
        dependants += [member.name for member in model.lumped_members if member.parent == shaft.name]
        dependants += [
            constraint.name for constraint in model.constraints if shaft.name in (constraint.body_a, constraint.body_b)
        ]
        if dependants:
            raise UnmappableElementError(
                element, f"shaft body '{shaft.name}' carries {', '.join(dependants)}; a reflected shaft carries nothing"
            )
        for dof in shaft.joint.angular:
            if dof.stiffness != 0.0 or dof.rest != 0.0 or dof.lower is not None or dof.upper is not None:
                raise UnmappableElementError(
                    f"joint '{shaft.joint.name}'", "a spring or limit on a reflected shaft joint"
                )
        inertia = _axis_inertia(shaft, shaft.joint.axis)
        report.append(
            f"{element}: shaft body '{shaft.name}' reflected onto joint '{transmission.driven_joint}' as armature "
            f"J / c^2 = {_num(inertia / coefficient**2)} (J = {_num(inertia)} about the shaft axis, "
            f"c = {_num(coefficient)}); its mass {_num(shaft.mass)} kg and its inertia off the axis are not carried"
        )
        if shaft.shapes:
            report.append(f"{element}: the {len(shaft.shapes)} shape(s) of shaft body '{shaft.name}' are not emitted")
        report.append(f"{element}: stiffness {_num(transmission.stiffness)} treated as rigid (milestone 1)")
        report.append(
            f"{element}: the rotor's reaction torque on its mount body '{shaft.parent}' and the spin-mount "
            "(gyroscopic) coupling are dropped: the shaft has no hinge in the MJCF"
        )
        report.append(
            f"{element}: efficiency read lossless on the rotor, (m + J / c^2) a = eta tau / c driving and "
            "tau / (eta c) back-driven; where the loss sits is an owner ruling (PHY-033)"
        )
        backlash = transmission.backlash / abs(coefficient)
        if backlash != 0.0:
            report.append(
                f"{element}: backlash {_num(transmission.backlash)} at the output is a play element of "
                f"{_num(backlash)} rad on the shaft's drive rest (PHY-006): no impact dynamics, and a torque-driven "
                "shaft passes its torque without play"
            )
        reflected[shaft.name] = _Reflection(
            transmission.name,
            f"transmission.{transmission.type}",
            coefficient,
            transmission.efficiency,
            inertia,
            shaft,
            transmission.driven_joint,
            backlash,
        )
    return reflected


def _drives(model: PhysicsModel, reflected: dict[str, _Reflection], report: list[str]) -> list[tuple[str, str]]:
    """(neutral joint, MuJoCo joint it moves) for every shaft joint and every listed drive joint, in body order."""
    shafts = {r.shaft.joint.name: r for r in reflected.values() if r.shaft.joint is not None}
    driven = {transmission.driven_joint for transmission in model.transmissions}
    listed = set(model.signals.drive_joints)
    drives: list[tuple[str, str]] = []
    for body in model.bodies:
        joint = body.joint
        if joint is None:
            continue
        if joint.name in shafts:
            reflection = shafts[joint.name]
            drives.append((joint.name, reflection.driven_joint))
            report.append(
                f"joint '{joint.name}': drive as soft equality '{joint.name}.drive' on joint "
                f"'{reflection.driven_joint}' (rest c q, stiffness K / c^2, damping D / c^2; c = "
                f"{_num(reflection.coefficient)}), inactive at load (PHY-034)"
            )
            if joint.name in listed:
                raise UnmappableElementError(f"joint '{joint.name}'", "listed as a drive joint; shafts always are")
            continue
        if joint.name not in listed:
            continue
        if joint.type not in ("revolute", "prismatic"):
            raise UnmappableElementError(f"joint '{joint.name}'", f"a drive on a {joint.type} joint")
        if joint.name in driven:
            raise UnmappableElementError(
                f"joint '{joint.name}'", "a drive on a transmission's driven joint; drive its shaft joint"
            )
        drives.append((joint.name, joint.name))
        report.append(f"joint '{joint.name}': drive as soft equality '{joint.name}.drive', inactive at load (PHY-034)")
    return drives


def _reflect(worldbody: ET.Element, reflection: _Reflection) -> None:
    """Armature J / c^2 on the driven joint; the shaft's own damping D and friction f reflect as D / c^2, f / |c|."""
    driven = worldbody.find(f".//joint[@name='{reflection.driven_joint}']")
    assert driven is not None
    c = reflection.coefficient
    added = {"armature": reflection.inertia / (c * c), "damping": 0.0, "frictionloss": 0.0}
    assert reflection.shaft.joint is not None
    for dof in reflection.shaft.joint.angular:
        added["damping"] += dof.damping / (c * c)
        added["frictionloss"] += dof.friction / abs(c)
    for attribute, value in added.items():
        if value != 0.0:
            driven.set(attribute, _num(float(driven.get(attribute, "0")) + value))


# ===== Payloads =========================================================


def _payloads(
    model: PhysicsModel, reflected: dict[str, _Reflection], payloads: Mapping[str, str], report: list[str]
) -> list[tuple[str, str]]:
    """(payload, carrier) in body order. A payload is a free body under world held on its carrier by a weld the
    adapter toggles (PHY-006, ENG-035); MuJoCo cannot re-parent a body, so the carrier is fixed at translation."""
    bodies = {body.name: body for body in model.bodies}
    for name, carrier in sorted(payloads.items()):
        element = f"payload '{name}'"
        body = bodies.get(name)
        if body is None:
            raise UnmappableElementError(element, "not a body in the physics model")
        if body.joint is None or body.joint.type != "free" or body.parent != WORLD:
            raise UnmappableElementError(element, "a payload must be a body on a free joint under world")
        if name in reflected:
            raise UnmappableElementError(element, "a reflected transmission shaft cannot be a payload")
        if carrier == name or carrier not in bodies:
            raise UnmappableElementError(element, f"carrier '{carrier}' is not another body in the physics model")
        if carrier in reflected:
            raise UnmappableElementError(element, f"carrier '{carrier}' is a reflected transmission shaft")
    attached: list[tuple[str, str]] = []
    for body in model.bodies:
        if body.name in payloads:
            carrier = payloads[body.name]
            attached.append((body.name, carrier))
            report.append(
                f"payload '{body.name}': free body held on '{carrier}' by weld '{body.name}.attach' at the translated "
                "relative pose (equality softness, 2 steps, critically damped); release and re-attach toggle the weld, "
                "and re-attaching pulls the payload back to that pose"
            )
    return attached


# ===== Contacts =========================================================


def _parameters(contacts: ContactModel, material_a: str, material_b: str) -> ContactParameters:
    for pair in contacts.pairs:
        if sorted(pair.materials) == sorted((material_a, material_b)):
            return pair
    return contacts.default


def _contact_pairs(
    contact: ET.Element,
    contacts: ContactModel,
    geoms: Sequence[tuple[str, str, str]],
    masses: dict[str, float],
    joined: set[tuple[str, str]],
    report: list[str],
) -> None:
    """One explicit pair per geom pair on different bodies, neither excluded nor joined by a non-free joint.

    Explicit pairs bypass MuJoCo's parent-child filter, so jointed neighbours are skipped here and reported.
    """
    excluded = {_ordered(pair) for pair in contacts.exclusions}
    skipped: list[tuple[str, str]] = []
    for i, (geom_a, body_a, material_a) in enumerate(geoms):
        for geom_b, body_b, material_b in geoms[i + 1 :]:
            bodies = _ordered((body_a, body_b))
            if body_a == body_b or bodies in excluded:
                continue
            if bodies in joined:
                if bodies not in skipped:
                    skipped.append(bodies)
                continue
            parameters = _parameters(contacts, material_a, material_b)
            reduced = _reduced_mass(masses.get(body_a), masses.get(body_b))
            ET.SubElement(
                contact,
                "pair",
                {
                    "geom1": geom_a,
                    "geom2": geom_b,
                    "condim": "3",
                    "friction": _vec((parameters.friction, parameters.friction, 0.0, 0.0, 0.0)),
                    "solref": _vec((-parameters.stiffness / reduced, -parameters.damping / reduced)),
                    "solimp": _vec((CONTACT_IMPEDANCE, CONTACT_IMPEDANCE, 0.001, 0.5, 2.0)),
                },
            )

    for body_a, body_b in skipped:
        report.append(f"contacts: bodies '{body_a}' and '{body_b}' are joined by a joint; their shapes do not collide")


def _ordered(pair: Sequence[str]) -> tuple[str, str]:
    return (min(pair[0], pair[1]), max(pair[0], pair[1]))


def _reduced_mass(mass_a: float | None, mass_b: float | None) -> float:
    if mass_a is None and mass_b is None:
        raise UnmappableElementError("contacts", "a contact pair between two world shapes")
    if mass_a is None:
        assert mass_b is not None
        return mass_b
    if mass_b is None:
        return mass_a
    return mass_a * mass_b / (mass_a + mass_b)


# ===== Model ============================================================


def _body(
    parents: dict[str, ET.Element],
    assets: ET.Element,
    body: Body,
    limit_ref: str,
    geoms: list[tuple[str, str, str]],
    kinds: set[str],
) -> None:
    if body.joint is None:
        raise UnmappableElementError(f"body '{body.name}'", "no joint to its parent")
    if body.joint.type == "free" and body.parent != WORLD:
        raise UnmappableElementError(
            f"joint '{body.joint.name}'", "a free joint below another body; MuJoCo allows free joints only on world"
        )
    element = ET.SubElement(
        parents[body.parent],
        "body",
        {"name": body.name, "pos": _vec(body.joint.position), "quat": _vec(body.joint.orientation)},
    )
    inertia = body.inertia
    ET.SubElement(
        element,
        "inertial",
        {
            "pos": _vec(body.com),
            "mass": _num(body.mass),
            "fullinertia": _vec((inertia.ixx, inertia.iyy, inertia.izz, inertia.ixy, inertia.ixz, inertia.iyz)),
        },
    )
    _joint(element, body.joint, limit_ref)
    kinds.add(f"joint.{body.joint.type}")
    for index, shape in enumerate(body.shapes):
        kinds.add(_shape_kind(shape))
        geom = _geom(element, assets, body.name, index, shape)
        geoms.append((geom.get("name", ""), body.name, shape.material))
    parents[body.name] = element


@dataclass(frozen=True)
class Translation:
    """MJCF text plus the mapping report: one line per approximation or omission (ADR 0003: none is silent)."""

    mjcf: str
    report: tuple[str, ...]


def translate(model: PhysicsModel, step: float | None = None, payloads: Mapping[str, str] | None = None) -> str:
    """MJCF text for the neutral model; `step` (s) overrides the run's step; `payloads` maps each payload body to the
    carrier body it starts attached to. Same inputs, same text, byte for byte."""
    return translate_report(model, step, payloads).mjcf


def translate_report(
    model: PhysicsModel, step: float | None = None, payloads: Mapping[str, str] | None = None
) -> Translation:
    """`translate` with its mapping report."""
    report: list[str] = []
    reflected = _reflections(model, report)
    attached = _payloads(model, reflected, dict(payloads or {}), report)
    timestep = model.run.step
    if step is not None:
        timestep = step
    reference = _vec((REFERENCE_STEPS * timestep, 1.0))
    root = ET.Element("mujoco", {"model": model.design.name})
    ET.SubElement(
        root,
        "compiler",
        {
            "angle": "radian",
            "autolimits": "false",
            "inertiafromgeom": "false",
            "balanceinertia": "false",
            "boundmass": "0",
            "boundinertia": "0",
        },
    )
    ET.SubElement(
        root,
        "option",
        {
            "timestep": _num(timestep),
            "gravity": _vec(model.gravity),
            "integrator": "implicitfast",
            "iterations": str(model.run.iteration_cap),
        },
    )
    assets = ET.SubElement(root, "asset")
    worldbody = ET.SubElement(root, "worldbody")
    contact = ET.Element("contact")
    equality = ET.Element("equality")
    custom = ET.Element("custom")
    kinds: set[str] = set()
    geoms: list[tuple[str, str, str]] = []

    for index, shape in enumerate(model.world_shapes):
        kinds.add(_shape_kind(shape))
        geom = _geom(worldbody, assets, WORLD, index, shape)
        geoms.append((geom.get("name", ""), WORLD, shape.material))

    parents: dict[str, ET.Element] = {WORLD: worldbody}
    for body in model.bodies:
        kinds.add("body")
        if body.name in reflected and body.joint is not None:
            kinds.add(f"joint.{body.joint.type}")
            continue
        _body(parents, assets, body, reference, geoms, kinds)

    for member in model.lumped_members:
        kinds.add("lumped_member")
        _member(parents, equality, member, reference)

    for constraint in model.constraints:
        kinds.add(_constraint(equality, parents, constraint, reference))

    for reflection in reflected.values():
        kinds.add(reflection.kind)
        _reflect(worldbody, reflection)

    for payload, carrier in attached:
        ET.SubElement(
            equality,
            "weld",
            {"name": f"{payload}.attach", "body1": payload, "body2": carrier, "solref": reference},
        )

    drives = _drives(model, reflected, report)
    for neutral_joint, mj_joint in drives:
        ET.SubElement(
            equality,
            "joint",
            {
                "name": f"{neutral_joint}.drive",
                "joint1": mj_joint,
                "polycoef": "0 0 0 0 0",
                "active": "false",
                "solref": DRIVE_SOLREF,
                "solimp": DRIVE_SOLIMP,
            },
        )

    if model.contacts is not None:
        kinds.add("contacts")
        masses = {body.name: body.mass for body in model.bodies}
        joined = {
            _ordered((body.parent, body.name))
            for body in model.bodies
            if body.joint is not None and body.joint.type != "free"
        }
        _contact_pairs(contact, model.contacts, geoms, masses, joined, report)

    for kind in sorted(kinds):
        if kind not in SUPPORTED_ELEMENTS:
            raise UnmappableElementError(kind, "not in the MuJoCo adapter's supported elements")

    transmission_order = {transmission.name: index for index, transmission in enumerate(model.transmissions)}
    joints = [f"{body.joint.name}:{body.joint.type}" for body in model.bodies if body.joint is not None]
    constraints = [f"{constraint.name}:{constraint.type}" for constraint in model.constraints]
    members = [
        f"{member.name}:{member.parent}:{member.root}:{member.end}:{member.segments}" for member in model.lumped_members
    ]
    texts = (
        ("mechsim:elements", sorted(kinds)),
        ("mechsim:bodies", [body.name for body in model.bodies]),
        ("mechsim:joints", joints),
        ("mechsim:constraints", constraints),
        ("mechsim:members", members),
        ("mechsim:drives", [neutral_joint for neutral_joint, _ in drives]),
        ("mechsim:payloads", [f"{payload}:{carrier}" for payload, carrier in attached]),
        (
            "mechsim:transmissions",
            [
                f"{r.transmission}:{r.shaft.name}:{r.shaft.parent}:{r.driven_joint}"
                for r in sorted(reflected.values(), key=lambda r: transmission_order[r.transmission])
            ],
        ),
    )
    for name, tokens in texts:
        if tokens:
            ET.SubElement(custom, "text", {"name": name, "data": " ".join(tokens)})
    for member in model.lumped_members:
        frame = (member.section.length / member.segments, *member.position, *member.orientation)
        ET.SubElement(custom, "numeric", {"name": f"mechsim:member:{member.name}", "data": _vec(frame)})

    for reflection in sorted(reflected.values(), key=lambda r: transmission_order[r.transmission]):
        joint = reflection.shaft.joint
        assert joint is not None
        axis_norm = math.sqrt(sum(component * component for component in joint.axis))
        values = (
            reflection.coefficient,
            reflection.efficiency,
            reflection.inertia,
            *joint.position,
            *joint.orientation,
            *(component / axis_norm for component in joint.axis),
            reflection.backlash,
        )
        ET.SubElement(
            custom, "numeric", {"name": f"mechsim:transmission:{reflection.transmission}", "data": _vec(values)}
        )

    for section in (contact, equality, custom):
        if len(section):
            root.append(section)
    if not len(assets):
        root.remove(assets)
    ET.indent(root)
    return Translation(ET.tostring(root, encoding="unicode") + "\n", tuple(report))
