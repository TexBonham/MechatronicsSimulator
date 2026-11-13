"""Design to engine-neutral physics model (BLD-001 to BLD-005, BLD-010 to BLD-014, BLD-020 to BLD-022, BLD-024).

The neutral model validates unit strings, so the builder writes every quantity as `repr(value) unit` in SI units,
which parses back to the same float, and validates the whole model once.
"""

from __future__ import annotations

import math
from collections.abc import Collection, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pydantic import ValidationError
from scipy import linalg  # type: ignore[import-untyped]

from mechsim.builder import mass, structure
from mechsim.builder.result import LumpedRecord
from mechsim.builder.structure import CalibrationScales
from mechsim.design.cad import MassPropertiesSI, StepImporter, cross_check_mass
from mechsim.design.schema import (
    WORLD,
    AngularDof,
    Body,
    Calibration,
    ComponentInstance,
    Design,
    FlexibleMember,
    Joint,
    LinearDof,
    MassProperties,
    OpenValue,
    PrimitiveGeometry,
    StepGeometry,
    iter_open_values,
    open_name,
    section_part_values,
)
from mechsim.library.registry import Registry, RegistryError
from mechsim.materials.schema import Material
from mechsim.physics.neutral import LumpedMember, PhysicsModel, Rope, StructuralMember
from mechsim.schema_base import format_errors

FLOOR_MATERIAL = "floor"
LUMPED_SUFFIX = "_beam"
TRANSMISSION_TYPES = ("belt_pulley", "lead_screw")
LUMPED_ROOTS = ("clamped", "pinned")


def clamped_chain(
    segments: int, length: float, flexural_rigidity: float, mass_per_length: float
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """The engine's clamped-clamped chain (ENG-022, ruling 16) in its hinge angles: the roller layout (springs E I / l
    at the segment midpoints, a half segment at the far end) with the far end held in place and in rotation (the
    engine's hinge about the member axis), so the angles allowed are those with zero end deflection sum (x_j <
    L) and zero end slope sum. Returns stiffness, mass, the allowed basis and the midspan influence row."""
    chain = structure.chain_layout("clamped", "roller", segments, length)
    segment = length / segments
    count = len(chain.hinges)
    stiffness = np.eye(count) * (flexural_rigidity / segment)
    inertia = np.zeros((count, count))
    for index, (start, span) in enumerate(zip(chain.starts, chain.spans, strict=True)):
        centre = start + 0.5 * span
        lever = np.array([centre - chain.hinges[j] if j <= index else 0.0 for j in range(count)])
        turn = np.array([1.0 if j <= index else 0.0 for j in range(count)])
        body = mass_per_length * span
        inertia += body * np.outer(lever, lever) + body * span * span / 12.0 * np.outer(turn, turn)
    end = np.array([length - hinge for hinge in chain.hinges])
    basis = linalg.null_space(np.vstack([end, np.ones(count)]))
    midspan = np.array([max(0.5 * length - hinge, 0.0) for hinge in chain.hinges])
    return stiffness, inertia, basis, midspan


def clamped_calibration(
    segments: int,
    length: float,
    flexural_rigidity: float,
    mass_per_length: float,
    load: float,
    deflection: float,
    frequency_hz: float,
) -> CalibrationScales:
    """BLD-019 on a clamped-clamped member (ruling 32, N >= 16): E I scaled so the chain's midspan deflection under
    `load` equals `deflection`, then the mass so its first mode equals `frequency_hz` (as `structure.calibrate`)."""
    stiffness, inertia, basis, midspan = clamped_chain(segments, length, flexural_rigidity, mass_per_length)
    reduced = basis.T @ stiffness @ basis
    angles = basis @ np.linalg.solve(reduced, load * (basis.T @ midspan))
    stiffness_scale = float(midspan @ angles) / deflection
    values = linalg.eigh(stiffness_scale * reduced, basis.T @ inertia @ basis, eigvals_only=True)
    omega = math.sqrt(max(float(values[0]), 0.0))
    return CalibrationScales(stiffness_scale, (omega / (2.0 * math.pi * frequency_hz)) ** 2)


LUMPED_ENDS = ("free", "roller", "clamped")


@dataclass
class Collector:
    failures: list[str] = field(default_factory=list)
    pending: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)


def _q(value: float, unit: str) -> str:
    return f"{value!r} {unit}"


def _v(values: Sequence[float], unit: str) -> list[str]:
    return [_q(value, unit) for value in values]


def _f(value: float | OpenValue) -> float:
    if isinstance(value, float):
        return value
    raise TypeError(f"open value {value} reached the physics build")


def _floats(values: Sequence[float | OpenValue]) -> tuple[float, float, float]:
    return (_f(values[0]), _f(values[1]), _f(values[2]))


@dataclass(frozen=True)
class PhysicsOutput:
    model: PhysicsModel | None
    members: tuple[StructuralMember, ...]
    lumped: tuple[LumpedRecord, ...]


# ===== Checks ===========================================================


PHYSICS_CATEGORIES = ("actuator", "transmission")


def _scan_pending(design: Design, registry: Registry, rigid_members: bool, collector: Collector) -> None:
    """Open values the physics model needs. Only actuator and transmission mounts reach the model; a rigid build
    does not need a flexible member's calibration."""
    sections: list[tuple[str, Any]] = [
        ("environment.gravity", design.environment.gravity),
        ("environment.contact", design.environment.contact),
    ]
    for index, body in enumerate(design.bodies):
        sections.append((f"bodies[{index}]", body))
    for index, joint in enumerate(design.joints):
        sections.append((f"joints[{index}]", joint))
    for index, component in enumerate(design.components):
        if not registry.has_type(component.type):
            continue
        if registry.component_type(component.type).category in PHYSICS_CATEGORIES:
            sections.append((f"components[{index}].mechanical", component.mechanical))
        if component.type in TRANSMISSION_TYPES:
            for key, value in component.overrides.items():
                if open_name(value) is not None:
                    collector.pending.append(f"components[{index}].overrides.{key}")
    skipped: list[str] = []
    if rigid_members:
        skipped = [f"bodies[{index}].flexible.calibration" for index in range(len(design.bodies))]
    for path, value in sections:
        for found, _open in iter_open_values(value, path):
            if not any(found.startswith(prefix) for prefix in skipped):
                collector.pending.append(found)


def _order(design: Design, collector: Collector) -> list[tuple[Body, Joint]]:
    """Bodies parent before child, depth first from the world in joint order; unreached bodies are unconstrained."""
    bodies = {body.name: body for body in design.bodies}
    children: dict[str, list[Joint]] = {}
    for joint in design.joints:
        children.setdefault(joint.parent, []).append(joint)
        if joint.child not in bodies:
            collector.failures.append(f"joint '{joint.name}': unknown child body '{joint.child}'")
        if joint.parent != WORLD and joint.parent not in bodies:
            collector.failures.append(f"joint '{joint.name}': unknown parent body '{joint.parent}'")
    ordered: list[tuple[Body, Joint]] = []
    reached: set[str] = set()
    stack = list(reversed(children.get(WORLD, [])))
    while len(stack) > 0:
        joint = stack.pop()
        if joint.child in reached or joint.child not in bodies:
            continue
        reached.add(joint.child)
        ordered.append((bodies[joint.child], joint))
        stack.extend(reversed(children.get(joint.child, [])))
    for body in design.bodies:
        if body.name not in reached:
            collector.failures.append(
                f"unconstrained body '{body.name}': no chain of joints connects it to the world (BLD-021)"
            )
    return ordered


def _check_bodies(design: Design, materials: dict[str, Material], collector: Collector) -> None:
    for body in design.bodies:
        if isinstance(body.material, str) and body.material not in materials:
            collector.failures.append(
                f"body '{body.name}' has no material: '{body.material}' is not in the materials database (BLD-020)"
            )
    for joint in design.joints:
        dofs: list[LinearDof | AngularDof] = [*joint.linear, *joint.angular]
        for index, dof in enumerate(dofs):
            if isinstance(dof.backlash, float) and dof.backlash != 0.0:
                collector.failures.append(
                    f"joint '{joint.name}' DOF {index}: backlash {dof.backlash!r} has no neutral joint element"
                )


def _check_supports(design: Design, collector: Collector) -> None:
    """Skipped in a rigid build (BLD-033), where no lumped beam is made."""
    for body in design.bodies:
        if body.flexible is not None:
            root, end = body.flexible.supports
            clamped_end_on_pin = end == "clamped" and root != "clamped"
            if root not in LUMPED_ROOTS or end not in LUMPED_ENDS or clamped_end_on_pin:
                collector.failures.append(
                    f"flexible body '{body.name}': supports ({root}, {end}) have no lumped beam; the neutral model "
                    f"takes a root in {LUMPED_ROOTS} and an end in {LUMPED_ENDS}, a clamped end with a clamped root"
                )


def _check_carried_travel(design: Design, registry: Registry, collector: Collector) -> None:
    """A carried joint's origin projects onto the member's span [0, L] over its whole travel (ENG-052, BLD-037).

    The member frame is the flexible body's frame (the lumped chain's root, x along the member). A fixed, revolute
    or spherical joint's origin is its placement; a prismatic joint's moves along its axis between its limits (an
    open side is unbounded travel); planar and free joints have no bounded travel. Skipped while a value is open."""
    joints = {joint.name: joint for joint in design.joints}
    for body in design.bodies:
        flexible = body.flexible
        if flexible is None or len(flexible.carries) == 0:
            continue
        section = flexible.section
        if section is None:
            values = section_part_values(body, registry)
            if values is None:
                continue
            length = values["length"]
        elif isinstance(section.length, OpenValue):
            continue
        else:
            length = _f(section.length)
        for name in flexible.carries:
            joint = joints.get(name)
            if joint is None or isinstance(joint.placement, OpenValue) or isinstance(joint.axis, OpenValue):
                continue
            label = f"flexible body '{body.name}': carried joint '{name}'"
            origin = _floats(joint.placement.position)
            ends = [origin[0]]
            if joint.type in ("planar", "free"):
                collector.failures.append(f"{label}: a {joint.type} joint has no bounded travel along the member")
                continue
            if joint.type == "prismatic" and joint.axis is not None:
                direction = mass.apply(mass.rotation(joint.placement.orientation), [float(v) for v in joint.axis])
                along = direction[0]
                limits = joint.linear[0].limits
                if limits is None or isinstance(limits.lower, OpenValue) or isinstance(limits.upper, OpenValue):
                    if abs(along) > 0.0:
                        collector.failures.append(
                            f"{label}: travel along the member is unbounded (no limits); the span is [0, {length}] m"
                        )
                    continue
                ends = [origin[0] + along * _f(limits.lower), origin[0] + along * _f(limits.upper)]
            low = min(ends)
            high = max(ends)
            if low < 0.0 or high > length:
                collector.failures.append(
                    f"{label}: origin travels over x in [{low:.6g}, {high:.6g}] m, outside the member's span "
                    f"[0, {length:.6g}] m by {max(-low, high - length):.6g} m (ENG-052)"
                )


def _check_bindings(design: Design, registry: Registry, collector: Collector) -> None:
    """Every cross-domain binding resolves to an existing joint or body mount face (BLD-028)."""
    bodies = {body.name for body in design.bodies}
    joints = {joint.name for joint in design.joints}
    for component in design.components:
        if not registry.has_type(component.type):
            continue
        mounts = {mount.name: mount.kind for mount in registry.component_type(component.type).mechanical.mounts}
        for binding in component.mechanical:
            label = f"component '{component.name}' mount '{binding.mount}'"
            if binding.mount not in mounts:
                collector.failures.append(f"{label}: type '{component.type}' has no such mount (BLD-028)")
                continue
            if isinstance(binding.body, str) and binding.body not in bodies:
                collector.failures.append(f"{label}: unresolved binding to unknown body '{binding.body}' (BLD-028)")
            if isinstance(binding.joint, str) and binding.joint not in joints:
                collector.failures.append(f"{label}: unresolved binding to unknown joint '{binding.joint}' (BLD-028)")
            if mounts[binding.mount] == "shaft" and binding.joint is None:
                collector.failures.append(f"{label}: a shaft mount must bind a joint (BLD-028)")


# ===== Bodies and joints ================================================


def _primitive_shapes(geometry: PrimitiveGeometry, material: str) -> list[dict[str, Any]]:
    shapes: list[dict[str, Any]] = []
    for shape in geometry.shapes:
        entry: dict[str, Any] = {
            "shape": shape.shape,
            "position": _v(_floats(shape.position), "m"),
            "orientation": list(shape.orientation),
            "material": material,
        }
        if shape.shape == "box":
            entry["size"] = _v(_floats(shape.size), "m")
        else:
            entry["radius"] = _q(_f(shape.radius), "m")
        if shape.shape in ("cylinder", "capsule"):
            entry["length"] = _q(_f(shape.length), "m")
        shapes.append(entry)
    return shapes


def primitive_mass(geometry: PrimitiveGeometry, density: float) -> mass.MassProperties:
    """Mass, center of mass and inertia of the union of a body's primitives at `density` (BLD-010)."""
    parts: list[mass.MassProperties] = []
    for shape in geometry.shapes:
        if shape.shape == "box":
            local = mass.box(density, _floats(shape.size))
        elif shape.shape == "sphere":
            local = mass.sphere(density, _f(shape.radius))
        elif shape.shape == "cylinder":
            local = mass.cylinder(density, _f(shape.radius), _f(shape.length))
        else:
            local = mass.capsule(density, _f(shape.radius), _f(shape.length))
        parts.append(mass.place(local, _floats(shape.position), shape.orientation))
    return mass.combine(parts)


def _declared(properties: MassProperties) -> mass.MassProperties:
    inertia = properties.inertia
    ixy, ixz, iyz = _f(inertia.ixy), _f(inertia.ixz), _f(inertia.iyz)
    return mass.MassProperties(
        _f(properties.mass),
        _floats(properties.com),
        ((_f(inertia.ixx), ixy, ixz), (ixy, _f(inertia.iyy), iyz), (ixz, iyz, _f(inertia.izz))),
    )


def _inertia_entry(properties: mass.MassProperties) -> dict[str, str]:
    tensor = properties.inertia
    unit = "kg*m**2"
    return {
        "ixx": _q(tensor[0][0], unit),
        "iyy": _q(tensor[1][1], unit),
        "izz": _q(tensor[2][2], unit),
        "ixy": _q(tensor[0][1], unit),
        "ixz": _q(tensor[0][2], unit),
        "iyz": _q(tensor[1][2], unit),
    }


def _step_body(
    body: Body,
    geometry: StepGeometry,
    declared: mass.MassProperties,
    density: float,
    importer: StepImporter | None,
    base: Path | None,
    collector: Collector,
) -> list[dict[str, Any]]:
    if importer is None or base is None:
        collector.notes.append(
            f"body '{body.name}': STEP geometry, no importer; declared mass properties used without a cross-check"
        )
        return []
    result = importer.import_body(base / geometry.file, body.name, density)
    if result.needs_mesh_contact:
        collector.failures.append(f"body '{body.name}': needs mesh contact, outside the v1 contact scope (BLD-012)")
    tensor = declared.inertia
    report = cross_check_mass(
        body.name,
        MassPropertiesSI(
            mass=declared.mass,
            center_of_mass=declared.com,
            inertia=(tensor[0][0], tensor[1][1], tensor[2][2], tensor[0][1], tensor[0][2], tensor[1][2]),
        ),
        result.mass_properties,
        _f(geometry.mass_check_tolerance),
        result.volume ** (1.0 / 3.0),
    )
    for mismatch in report.mismatches:
        collector.failures.append(
            f"body '{body.name}': declared {mismatch.quantity} differs from the STEP geometry by {mismatch.error!r}, "
            f"over the tolerance {mismatch.tolerance!r}"
        )
    return [
        {"shape": "convex_hull", "vertices": [_v(vertex, "m") for vertex in hull.vertices], "material": body.material}
        for hull in result.hulls
    ]


def _dof(dof: Any, unit_length: str, unit_force: str, unit_damping: str) -> dict[str, Any]:
    entry: dict[str, Any] = {"friction": _q(_f(dof.friction), unit_force), "damping": _q(_f(dof.damping), unit_damping)}
    if dof.limits is not None:
        entry["lower"] = _q(_f(dof.limits.lower), unit_length)
        entry["upper"] = _q(_f(dof.limits.upper), unit_length)
    return entry


def _joint(joint: Joint) -> dict[str, Any]:
    placement = joint.placement
    if isinstance(placement, OpenValue):
        raise TypeError("open placement reached the physics build")
    entry: dict[str, Any] = {
        "name": joint.name,
        "type": joint.type,
        "position": _v(_floats(placement.position), "m"),
        "orientation": list(placement.orientation),
        "linear": [_dof(dof, "m", "N", "N*s/m") for dof in joint.linear],
        "angular": [_dof(dof, "rad", "N*m", "N*m*s/rad") for dof in joint.angular],
    }
    if isinstance(joint.axis, list):
        entry["axis"] = list(joint.axis)
    return entry


# ===== Overlap ==========================================================


def _shape_boxes(entry: dict[str, Any], pose: tuple[mass.Vector, mass.Quaternion]) -> tuple[mass.Vector, mass.Vector]:
    def number(text: str) -> float:
        return float(text.split()[0])

    position = [number(text) for text in entry.get("position", ["0 m", "0 m", "0 m"])]
    orientation = entry.get("orientation", [1.0, 0.0, 0.0, 0.0])
    shape_pose = mass.compose(pose, position, orientation)
    if entry["shape"] == "convex_hull":
        matrix = mass.rotation(shape_pose[1])
        points = [mass.apply(matrix, [number(text) for text in vertex]) for vertex in entry["vertices"]]
        low = tuple(shape_pose[0][axis] + min(point[axis] for point in points) for axis in range(3))
        high = tuple(shape_pose[0][axis] + max(point[axis] for point in points) for axis in range(3))
        return ((low[0], low[1], low[2]), (high[0], high[1], high[2]))
    size = [number(text) for text in entry.get("size", ["0 m", "0 m", "0 m"])]
    radius = number(entry.get("radius", "0 m"))
    length = number(entry.get("length", "0 m"))
    return mass.world_box(shape_pose, mass.half_extents(entry["shape"], size, radius, length))


def _check_overlap(
    ordered: list[tuple[Body, Joint]],
    shapes: dict[str, list[dict[str, Any]]],
    exclusions: set[tuple[str, str]],
    collector: Collector,
) -> None:
    """Bodies not joined to each other and not excluded may not overlap at the reference configuration (BLD-022).

    Test geometry: world axis-aligned bounds of each primitive or hull, all joint coordinates zero; touching passes.
    """
    poses: dict[str, tuple[mass.Vector, mass.Quaternion]] = {WORLD: (mass.ORIGIN, mass.IDENTITY)}
    joined: set[tuple[str, str]] = set()
    boxes: list[tuple[str, tuple[mass.Vector, mass.Vector]]] = []
    for body, joint in ordered:
        placement = joint.placement
        if isinstance(placement, OpenValue):
            raise TypeError("open placement reached the overlap test")
        poses[body.name] = mass.compose(poses[joint.parent], _floats(placement.position), placement.orientation)
        joined.add((min(joint.parent, body.name), max(joint.parent, body.name)))
        for entry in shapes[body.name]:
            boxes.append((body.name, _shape_boxes(entry, poses[body.name])))
    reported: set[tuple[str, str]] = set()
    for index, (first, (low_a, high_a)) in enumerate(boxes):
        for second, (low_b, high_b) in boxes[index + 1 :]:
            pair = (min(first, second), max(first, second))
            if first == second or pair in joined or pair in exclusions or pair in reported:
                continue
            if all(low_a[axis] < high_b[axis] and low_b[axis] < high_a[axis] for axis in range(3)):
                reported.add(pair)
                collector.failures.append(
                    f"overlapping bodies '{first}' and '{second}' at the reference configuration (BLD-022)"
                )


# ===== Lumped members ===================================================


def lumped_record(
    body: str, member: LumpedMember, flexible: FlexibleMember, scales: CalibrationScales | None
) -> LumpedRecord:
    """Segment layout per `core/engine` lumped beam: N segments of l = L / N, springs E I / l at segment midpoints;
    rigid spans node to node, a clamped root leaving [0, l/2] on the parent, a pinned root adding [0, l/2]. The
    member's section is the calibrated one; `scales` are the calibration factors (BLD-019), None when uncalibrated."""
    section = member.section
    count = member.segments
    segment = section.length / count
    spans = [segment] * (count - 1) + [segment / 2.0]
    if member.root == "pinned":
        spans = [segment / 2.0, *spans]
    calibration = flexible.calibration
    if isinstance(calibration, OpenValue):
        raise TypeError("open calibration reached the lumped build")
    record: dict[str, Any] = {
        "member": member.name,
        "body": body,
        "segments": count,
        "segment_length": segment,
        "node_stiffness": section.elastic_modulus * section.second_moment_of_area / segment,
        "spans": tuple(spans),
        "masses": tuple(section.density * section.area * span for span in spans),
        "calibrated": scales is not None,
    }
    if calibration is not None and scales is not None:
        record["calibration_source"] = calibration.source
        record["calibration_load"] = _f(calibration.load)
        record["calibration_deflection"] = _f(calibration.deflection)
        record["calibration_frequency"] = _f(calibration.first_natural_frequency)
        record["stiffness_scale"] = scales.stiffness_scale
        record["mass_scale"] = scales.mass_scale
    return LumpedRecord(**record)


def member_constants(body: Body, material: Material, registry: Registry) -> dict[str, float]:
    """E, I, rho, A, L of a flexible body: the design section with the body's material, else its catalog part's
    section and material constants (LIB-030)."""
    flexible = body.flexible
    if flexible is None:
        raise TypeError("rigid body passed as flexible")
    if flexible.section is None:
        values = section_part_values(body, registry)
        if values is None:
            raise TypeError(f"flexible body '{body.name}' has neither a section nor a section part")
        return {
            "elastic_modulus": values["elastic_modulus"],
            "second_moment_of_area": values["second_moment"],
            "density": values["density"],
            "area": values["area"],
            "length": values["length"],
        }
    area, moment = flexible.section.area_and_moment()
    constants = {
        "elastic_modulus": material.youngs_modulus.typ,
        "second_moment_of_area": moment,
        "density": material.density.typ,
        "area": area,
        "length": _f(flexible.section.length),
    }
    properties = flexible.section.properties()
    if properties is not None:
        constants.update(
            {
                "second_moment_y": properties.second_moment_y,
                "fibre_y": properties.fibre_y,
                "fibre_z": properties.fibre_z,
                "torsion_constant": properties.torsion_constant,
                "shear_area_y": properties.shear_area_y,
                "shear_area_z": properties.shear_area_z,
            }
        )
        return constants
    axes = flexible.section.bending_axes()
    if axes is not None:
        i_y, _, c_y, c_z = axes
        constants.update({"second_moment_y": i_y, "fibre_y": c_y, "fibre_z": c_z})
    return constants


SECTION_UNITS = {
    "elastic_modulus": "Pa",
    "second_moment_of_area": "m**4",
    "density": "kg/m**3",
    "area": "m**2",
    "length": "m",
    "second_moment_y": "m**4",
    "fibre_y": "m",
    "fibre_z": "m",
    "torsion_constant": "m**4",
    "shear_area_y": "m**2",
    "shear_area_z": "m**2",
}


def _section(constants: Mapping[str, float]) -> dict[str, str]:
    return {name: _q(constants[name], unit) for name, unit in SECTION_UNITS.items() if name in constants}


def _flexible(
    body: Body, material: Material, design: Design, registry: Registry, collector: Collector, rigid: bool
) -> tuple[dict[str, Any], dict[str, Any], CalibrationScales | None]:
    """The lumped member (calibrated, BLD-019), the structural member list entry (true section, BLD-008, BLD-009)
    and the calibration scales of a flexible body."""
    flexible = body.flexible
    if flexible is None:
        raise TypeError("rigid body passed as flexible")
    root, end = flexible.supports
    constants = member_constants(body, material, registry)
    lumped_constants = dict(constants)
    scales: CalibrationScales | None = None
    calibration = flexible.calibration
    segments = structure.default_segments(root, end)
    if "segments" in flexible.model_fields_set:
        segments = flexible.segments
    minimum = structure.calibration_min_segments(root, end)
    lumped_form = root in LUMPED_ROOTS and end in ("free", "roller")
    if rigid:
        scales = None
    elif isinstance(calibration, Calibration) and segments < minimum:
        collector.failures.append(
            f"flexible body '{body.name}': calibrated member has {segments} segments; calibration needs at least "
            f"{minimum} for supports ({root}, {end}) (the 2 percent tolerance holds for N >= 8, and N >= 16 "
            "clamped-clamped, rulings 20 and 32; BLD-018, BLD-019)"
        )
    elif isinstance(calibration, Calibration) and root == "clamped" and end == "clamped":
        scales = clamped_calibration(
            segments,
            constants["length"],
            constants["elastic_modulus"] * constants["second_moment_of_area"],
            constants["density"] * constants["area"],
            _f(calibration.load),
            _f(calibration.deflection),
            _f(calibration.first_natural_frequency),
        )
        lumped_constants["elastic_modulus"] *= scales.stiffness_scale
        lumped_constants["density"] *= scales.mass_scale
        collector.notes.extend(structure.approximation_notes(body.name, scales))
    elif isinstance(calibration, Calibration) and not lumped_form:
        collector.notes.append(
            f"flexible body '{body.name}': supports ({root}, {end}) have no calibration chain layout yet (owed, "
            "ruling 16); its calibration data is not applied (BLD-019)"
        )
    elif isinstance(calibration, Calibration):
        chain = structure.chain_layout(root, end, segments, constants["length"])
        try:
            scales = structure.calibrate(
                chain,
                constants["elastic_modulus"] * constants["second_moment_of_area"],
                constants["density"] * constants["area"],
                _f(calibration.load),
                _f(calibration.deflection),
                _f(calibration.first_natural_frequency),
            )
        except structure.ChainError as error:
            collector.failures.append(f"flexible body '{body.name}': calibration failed: {error} (BLD-019)")
        else:
            lumped_constants["elastic_modulus"] *= scales.stiffness_scale
            lumped_constants["density"] *= scales.mass_scale
            collector.notes.extend(structure.approximation_notes(body.name, scales))
    elif calibration is None:
        collector.notes.append(
            f"flexible body '{body.name}': no calibration data; the lumped beam uses its section and material as is "
            "(BLD-019)"
        )
    member: dict[str, Any] = {
        "name": body.name + LUMPED_SUFFIX,
        "parent": body.name,
        "position": _v(mass.ORIGIN, "m"),
        "section": _section(lumped_constants),
        "root": root,
        "end": end,
        "structural_member": body.name,
    }
    if len(flexible.carries) > 0:
        child_of = {joint.name: joint.child for joint in design.joints}
        member["carries"] = [child_of[name] for name in flexible.carries]
    member["segments"] = segments
    length = constants["length"]
    supports = [{"at": _q(0.0, "m"), "condition": root}]
    if end != "free":
        supports.append({"at": _q(length, "m"), "condition": end})
    structural = {
        "name": body.name,
        "section": _section(constants),
        "material": material.name,
        "supports": supports,
        "fasteners": [fastener.model_dump(mode="json") for fastener in body.fasteners],
        "mounts": structure.member_mounts(body, design),
    }
    return member, structural, scales


# ===== Transmissions and signals ========================================


def _transmission(
    component: ComponentInstance, registry: Registry, child_joints: dict[str, Joint], collector: Collector
) -> dict[str, Any] | None:
    kind = registry.component_type(component.type)
    shaft_mounts = {mount.name for mount in kind.mechanical.mounts if mount.kind == "shaft"}
    shaft: str | None = None
    driven: str | None = None
    for binding in component.mechanical:
        if binding.mount in shaft_mounts and isinstance(binding.joint, str):
            shaft = binding.joint
        elif isinstance(binding.body, str):
            joint = child_joints.get(binding.body)
            if joint is not None:
                driven = joint.name
    label = f"transmission '{component.name}'"
    if shaft is None or driven is None:
        collector.failures.append(f"{label}: needs a shaft joint and a driven body on a joint (BLD-005)")
        return None
    concrete = {key: value for key, value in component.overrides.items() if open_name(value) is None}
    try:
        values = registry.resolve_part(component.part, concrete).values
    except RegistryError as error:
        collector.failures.append(f"{label}: {error} (BLD-027)")
        return None
    if "stiffness" not in values:
        collector.failures.append(f"{label}: part '{component.part}' sets no stiffness")
        return None
    ratio = float(values["ratio"])
    entry: dict[str, Any] = {
        "type": component.type,
        "name": component.name,
        "shaft_joint": shaft,
        "driven_joint": driven,
        "efficiency": float(values["efficiency"]),
        "slip": float(values.get("slip", 0.0)),
        "stiffness": _q(float(values["stiffness"]), "N/m"),
        "backlash": _q(float(values.get("backlash", 0.0)), "m"),
    }
    if component.type == "belt_pulley":
        entry["pulley_radius"] = _q(ratio, "m")
    else:
        entry["lead"] = _q(ratio * 2.0 * math.pi, "m")
        entry["self_locking"] = bool(values["self_locking"])
    return entry


def _rotor_inertias(design: Design, registry: Registry) -> dict[str, tuple[float, list[str]]]:
    """Shaft joint name -> the rotor inertia of the actuator instances whose shaft mount binds it (kg m^2), and their
    names.

    Rotor inertia has one source, the motor instance's `rotor_inertia` (coordinator ruling 2026-11-04, BLD-034): the
    shaft body's axial inertia is set to it and the signal layer reads the same value (SIG-037)."""
    rotors: dict[str, tuple[float, list[str]]] = {}
    for component in design.components:
        if not registry.has_type(component.type):
            continue
        kind = registry.component_type(component.type)
        if kind.category != "actuator":
            continue
        concrete = {key: value for key, value in component.overrides.items() if open_name(value) is None}
        try:
            values = registry.resolve_part(component.part, concrete).values
        except RegistryError:
            continue
        inertia = values.get("rotor_inertia")
        if not isinstance(inertia, float) or isinstance(inertia, bool):
            continue
        shafts = {mount.name for mount in kind.mechanical.mounts if mount.kind == "shaft"}
        for binding in component.mechanical:
            if binding.mount in shafts and isinstance(binding.joint, str):
                total, motors = rotors.get(binding.joint, (0.0, []))
                rotors[binding.joint] = (total + inertia, [*motors, component.name])
    return rotors


def _check_one_actuator_per_shaft(rotors: Mapping[str, tuple[float, list[str]]], collector: Collector) -> None:
    """Two actuators on one shaft joint would sum their rotor inertias into one body, which the signal layer then
    refuses (SIG-037); the build refuses first, naming both (BLD-034)."""
    for joint in sorted(rotors):
        motors = rotors[joint][1]
        if len(motors) > 1:
            names = ", ".join(f"'{name}'" for name in motors)
            collector.failures.append(
                f"shaft joint '{joint}': driven by actuators {names}; one actuator per shaft joint (BLD-034)"
            )


def _check_rotor_bodies(design: Design, rotors: Mapping[str, tuple[float, list[str]]], collector: Collector) -> None:
    """A shaft body's axial inertia is its motor's rotor inertia (BLD-034); a declared one would be a second source."""
    bodies = {body.name: body for body in design.bodies}
    for joint in design.joints:
        if joint.name not in rotors:
            continue
        body = bodies.get(joint.child)
        if body is not None and isinstance(body.mass_properties, MassProperties):
            motors = ", ".join(f"'{name}'" for name in rotors[joint.name][1])
            collector.failures.append(
                f"body '{body.name}': declares its own mass properties, but its axial inertia is the rotor inertia "
                f"of motor {motors} on shaft joint '{joint.name}' (BLD-034)"
            )


def with_rotor(properties: mass.MassProperties, axis: Sequence[float], rotor: float) -> mass.MassProperties:
    """`properties` with its axial inertia about the joint axis through the center of mass set to `rotor` (kg m^2):
    I + (J - a^T I a) a a^T, so a^T I' a = J. The rotor turns about the shaft axis, which the design puts through the
    shaft body's origin; a shaft body's center of mass on that axis makes the two the same."""
    norm = math.sqrt(sum(value * value for value in axis))
    a = [value / norm for value in axis]
    i = properties.inertia
    axial = sum(a[r] * i[r][c] * a[c] for r in range(3) for c in range(3))
    added = rotor - axial

    def row(r: int) -> mass.Vector:
        return (i[r][0] + added * a[r] * a[0], i[r][1] + added * a[r] * a[1], i[r][2] + added * a[r] * a[2])

    return mass.MassProperties(properties.mass, properties.com, (row(0), row(1), row(2)))


def _load_torque_joints(design: Design, registry: Registry) -> list[str]:
    joints: list[str] = []
    for component in design.components:
        if not registry.has_type(component.type):
            continue
        kind = registry.component_type(component.type)
        if kind.category != "actuator":
            continue
        shafts = {mount.name for mount in kind.mechanical.mounts if mount.kind == "shaft"}
        for binding in component.mechanical:
            if binding.mount in shafts and isinstance(binding.joint, str) and binding.joint not in joints:
                joints.append(binding.joint)
    return joints


# ===== Physics model ====================================================


def element_kinds(model: PhysicsModel) -> list[tuple[str, str]]:
    """Every element of the model as (kind, name); kinds are the vocabulary `supported_elements` uses."""
    elements: list[tuple[str, str]] = []
    for shape in model.world_shapes:
        elements.append((f"shape.{shape.shape}", WORLD))
    if model.contacts is not None:
        elements.append(("contacts", "contacts"))
    for body in model.bodies:
        elements.append(("body", body.name))
        if body.joint is not None:
            elements.append((f"joint.{body.joint.type}", body.joint.name))
        for shape in body.shapes:
            elements.append((f"shape.{shape.shape}", body.name))
    for constraint in model.constraints:
        elements.append((f"constraint.{constraint.type}", constraint.name))
    for member in model.lumped_members:
        elements.append(("lumped_member", member.name))
    for transmission in model.transmissions:
        elements.append((f"transmission.{transmission.type}", transmission.name))
    for rope in model.ropes:
        elements.append(("rope", rope.name))
    return elements


def build_physics(
    design: Design,
    registry: Registry,
    materials: dict[str, Material],
    collector: Collector,
    *,
    blob: str,
    supported_elements: Collection[str] | None,
    adapter: str,
    step_importer: StepImporter | None,
    base: Path | None,
    rigid_members: bool,
) -> PhysicsOutput:
    """`rigid_members` builds flexible bodies as rigid bodies: no lumped members, the member list kept (BLD-033)."""
    empty = PhysicsOutput(None, (), ())
    _scan_pending(design, registry, rigid_members, collector)
    ordered = _order(design, collector)
    _check_bodies(design, materials, collector)
    _check_bindings(design, registry, collector)
    if not rigid_members:
        _check_supports(design, collector)
        _check_carried_travel(design, registry, collector)
    rotors = _rotor_inertias(design, registry)
    _check_one_actuator_per_shaft(rotors, collector)
    _check_rotor_bodies(design, rotors, collector)
    if len(collector.pending) > 0 or len(collector.failures) > 0:
        return empty
    child_joints = {joint.child: joint for joint in design.joints}
    bodies: list[dict[str, Any]] = []
    shapes: dict[str, list[dict[str, Any]]] = {}
    lumped: list[dict[str, Any]] = []
    structural: list[dict[str, Any]] = []
    flexible_bodies: dict[str, tuple[Body, CalibrationScales | None]] = {}
    for body, joint in ordered:
        material = materials[_name(body.material)]
        geometry = body.geometry
        body_shapes: list[dict[str, Any]] = []
        if isinstance(geometry, PrimitiveGeometry):
            body_shapes = _primitive_shapes(geometry, material.name)
        if isinstance(body.mass_properties, MassProperties):
            properties = _declared(body.mass_properties)
        elif isinstance(geometry, PrimitiveGeometry):
            properties = primitive_mass(geometry, material.density.typ)
        else:
            raise TypeError(f"body '{body.name}' has neither declared mass properties nor primitives")
        if isinstance(geometry, StepGeometry):
            body_shapes = _step_body(body, geometry, properties, material.density.typ, step_importer, base, collector)
        if joint.name in rotors and joint.axis is not None and not isinstance(joint.axis, OpenValue):
            properties = with_rotor(properties, [float(value) for value in joint.axis], rotors[joint.name][0])
        shapes[body.name] = body_shapes
        bodies.append(
            {
                "name": body.name,
                "mass": _q(properties.mass, "kg"),
                "com": _v(properties.com, "m"),
                "inertia": _inertia_entry(properties),
                "material": material.name,
                "parent": joint.parent,
                "joint": _joint(joint),
                "shapes": body_shapes,
            }
        )
        if body.catalog_part is not None:
            bodies[-1]["catalog_part"] = body.catalog_part.part
        if body.flexible is not None:
            member, entry, scales = _flexible(body, material, design, registry, collector, rigid_members)
            structural.append(entry)
            if not rigid_members:
                lumped.append(member)
                flexible_bodies[member["name"]] = (body, scales)
    contact = design.environment.contact
    exclusions: set[tuple[str, str]] = set()
    data: dict[str, Any] = {
        "schema_version": 1,
        "design": {"name": design.name, "version": design.version, "blob": blob},
        "gravity": _v(_floats(design.environment.gravity), "m/s**2"),
        "bodies": bodies,
        "lumped_members": lumped,
        "transmissions": [],
        "signals": {
            "load_torque_joints": _load_torque_joints(design, registry),
            "pose_bodies": [body.name for body, _joint_entry in ordered],
        },
        "structural_members": structural,
        "ropes": _ropes(design, registry, collector),
        "fastenings": structure.fastenings(design),
    }
    if contact is not None:
        exclusions = {(min(pair), max(pair)) for pair in contact.exclusions}

        def parameters(source: Any) -> dict[str, Any]:
            return {
                "stiffness": _q(_f(source.stiffness), "N/m"),
                "damping": _q(_f(source.damping), "N*s/m"),
                "friction": _f(source.friction),
            }

        data["contacts"] = {
            "default": parameters(contact.default),
            "pairs": [{**parameters(pair), "materials": list(pair.materials)} for pair in contact.pairs],
            "exclusions": [list(pair) for pair in contact.exclusions],
        }
        if contact.floor is not None:
            normal = contact.floor.normal
            offset = _f(contact.floor.offset)
            data["world_shapes"] = [
                {
                    "shape": "plane",
                    "position": _v((normal[0] * offset, normal[1] * offset, normal[2] * offset), "m"),
                    "orientation": list(mass.z_to(normal)),
                    "material": FLOOR_MATERIAL,
                }
            ]
    for component in design.components:
        if registry.has_type(component.type) and registry.component_type(component.type).category == "transmission":
            if component.type not in TRANSMISSION_TYPES:
                collector.failures.append(
                    f"transmission '{component.name}': type '{component.type}' has no neutral transmission model"
                )
                continue
            transmission = _transmission(component, registry, child_joints, collector)
            if transmission is not None:
                data["transmissions"].append(transmission)
    _check_overlap(ordered, shapes, exclusions, collector)
    if len(collector.failures) > 0:
        return empty
    try:
        model = PhysicsModel.model_validate(data)
    except ValidationError as error:
        collector.failures.extend(f"physics model: {message}" for message in format_errors(error))
        return empty
    if supported_elements is not None:
        supported = set(supported_elements)
        for kind, name in element_kinds(model):
            if kind not in supported:
                collector.failures.append(f"element '{name}' ({kind}): adapter '{adapter}' cannot map it (BLD-024)")
        if len(collector.failures) > 0:
            return empty
    records: list[LumpedRecord] = []
    for lumped_member in model.lumped_members:
        flexible, scales = flexible_bodies[lumped_member.name]
        if flexible.flexible is not None:
            records.append(lumped_record(flexible.name, lumped_member, flexible.flexible, scales))
    return PhysicsOutput(model, model.structural_members, tuple(records))


def _ropes(design: Design, registry: Registry, collector: Collector) -> list[dict[str, Any]]:
    """Ropes with both ends on bodies, as neutral lumped chains (BLD-032); a rope ending at a case payload is
    noted and left to the case (the payload is placed at run time)."""
    out: list[dict[str, Any]] = []
    for rope in design.ropes:
        element = structure.rope_element(rope, registry.part(rope.part).values)
        if element is None:
            collector.notes.append(
                f"rope '{rope.name}': its to end is case payload '{rope.to_end.payload}'; not emitted until the case "
                "places the payload (BLD-032)"
            )
            continue
        out.append(element)
    return out


def payload_ropes(design: Design, registry: Registry) -> tuple[Rope, ...]:
    """The neutral rope of each design rope ending at a case payload, for the run to hang its payload on (BLD-032)."""
    out: list[Rope] = []
    for rope in design.ropes:
        element = structure.payload_rope_element(rope, registry.part(rope.part).values)
        if element is not None:
            out.append(Rope.model_validate(element))
    return tuple(out)


def _name(value: str | OpenValue) -> str:
    if isinstance(value, str):
        return value
    raise TypeError(f"open value {value} reached the physics build")
