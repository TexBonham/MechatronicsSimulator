"""Run inputs from a finished run: the build's member list and fastenings, the design's structural data, and the
recorded member end-loads and joint reactions (REC-004)."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np

from mechsim.integrity.plugin import (
    BodyPoses,
    CarriedLoad,
    FloatArray,
    IntegrityError,
    JointLoads,
    MaterialView,
    MemberInputs,
    NodeChain,
    RopeLoads,
    RunInputs,
    SectionProperties,
    Wrenches,
)
from mechsim.testing.driver import RecordView

MEMBER_SERIES = (
    "physics/members/start_force",
    "physics/members/start_moment",
    "physics/members/end_force",
    "physics/members/end_moment",
)
REACTION_FORCE = "physics/joints/reaction_force"
REACTION_MOMENT = "physics/joints/reaction_moment"
ROPE_MAX_TENSION = "physics/ropes/max_tension"
BODY_POSITION = "physics/bodies/position"
BODY_ORIENTATION = "physics/bodies/orientation"


def section_properties(section: Any) -> SectionProperties:
    """Both bending axes (ruling 25): I_z is the section's `second_moment_of_area`, I_y `second_moment_y`, c_y
    `fibre_y`, c_z `fibre_z`. A section missing any y-axis field is evaluated single-axis (ruling 25 fallback):
    I_y = I_z, c_y = c_z = the stated fibre or that of the solid rectangle with this A and I, sqrt(3 I / A); flagged."""
    area = float(section.area)
    i_z = float(section.second_moment_of_area)
    i_y = getattr(section, "second_moment_y", None)
    c_y = getattr(section, "fibre_y", None)
    c_z = getattr(section, "fibre_z", None)
    single = i_y is None or c_y is None or c_z is None
    fallback = math.sqrt(3.0 * i_z / area)
    if c_y is None:
        c_y = c_z
    if c_y is None:
        c_y = fallback
    if c_z is None:
        c_z = c_y
    if i_y is None:
        i_y = i_z
    return SectionProperties(
        area=area,
        elastic_modulus=float(section.elastic_modulus),
        density=float(section.density),
        length=float(section.length),
        i_y=float(i_y),
        i_z=i_z,
        c_y=float(c_y),
        c_z=float(c_z),
        single_axis=single,
    )


def _column(series: Mapping[str, Any], path: str, name: str, rows: int) -> FloatArray:
    if path not in series:
        raise IntegrityError(f"the recording has no {path}")
    stored = series[path]
    names = tuple(stored.names)
    if name not in names:
        raise IntegrityError(f"member {name!r}: no recorded end-loads in {path} (columns {list(names)})")
    values = np.asarray(stored.values, dtype=np.float64)
    column: FloatArray = values[:, names.index(name), :].reshape(rows, 3)
    return column


def _member_loads(series: Mapping[str, Any], name: str, rows: int, rigid: bool) -> Wrenches | None:
    """A rigid member (the build has a physics model and no lumped record for it, BLD-033) with no recorded end-loads
    gives None: not evaluated (rulings 29, 34). Some of its end-load series recorded and others not is a partial
    recording, an error (INT-005). A flexible member, or a build without a physics model, needs them all."""
    if rigid:
        recorded = [path for path in MEMBER_SERIES if path in series and name in tuple(series[path].names)]
        if len(recorded) == 0:
            return None
        if len(recorded) < len(MEMBER_SERIES):
            missing = [path for path in MEMBER_SERIES if path not in recorded]
            raise IntegrityError(f"member {name!r}: partial end-load recording, has {recorded}, lacks {missing}")
    return Wrenches(*(_column(series, path, name, rows) for path in MEMBER_SERIES))


def _series_reader(recording: RecordView) -> Any:
    def read(reference: str) -> tuple[FloatArray, FloatArray]:
        found = recording.series(reference)
        return np.asarray(found.times, dtype=np.float64), np.asarray(found.values, dtype=np.float64)

    return read


def member_axes(build: Any) -> dict[str, tuple[float, float, float]]:
    """Member x axis (start to end) in its parent body's frame, from the lumped member's placement, the rotation the
    recorder's member frame uses (ruling 24); keyed by structural member name. A member without a lumped record
    (a rigid build) is absent and keeps the body x axis."""
    physics = getattr(build, "physics", None)
    out: dict[str, tuple[float, float, float]] = {}
    if physics is None:
        return out
    for lumped in getattr(physics, "lumped_members", ()):
        w, x, y, z = (float(value) for value in lumped.orientation)
        axis = (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y))
        name = lumped.structural_member
        if name is None:
            name = lumped.name
        out[str(name)] = axis
    return out


WORLD = "world"


def node_chains(model: Any, names: tuple[str, ...] = ()) -> dict[str, NodeChain]:
    """Each lumped member's node chain in the native adapter's published bodies, keyed by structural member name: the
    neutral bodies first, then per lumped member in neutral order N node bodies (N + 1 with a pinned root's pin
    segment), named `<member>.node<k>` (`mechsim.physics.native.member_node_names`). With the published body `names`
    (the recorder's, or the prepared native model's) the chain and its parent are found by name, so bodies a case adds
    at run time (a payload and its shackle) shift nothing; without them by the build's layout. A parent that is neither
    the world nor a body, or that names another member's node, is an `IntegrityError` (INT-006), never the world. A
    model without lumped members (the bridge builds members rigid) gives none."""
    out: dict[str, NodeChain] = {}
    if model is None:
        return out
    bodies = tuple(getattr(model, "bodies", ()))
    lumped_members = tuple(getattr(model, "lumped_members", ()))
    node_owners = {str(lumped.name) for lumped in lumped_members}
    index = {str(body.name): position for position, body in enumerate(bodies)}
    if len(names) > 0 and len(lumped_members) > 0:
        index = {name: position for position, name in enumerate(names) if not _is_node(name, node_owners)}
        missing = [str(body.name) for body in bodies if str(body.name) not in index]
        if len(missing) > 0:
            raise IntegrityError(
                f"the published bodies ({len(names)}) name no {missing}, bodies of the model (INT-006)"
            )
    first = len(bodies)
    for lumped in lumped_members:
        count = int(lumped.segments)
        if lumped.root == "pinned":
            count += 1
        name = str(lumped.name)
        if lumped.structural_member is not None:
            name = str(lumped.structural_member)
        parent = _parent_body(str(lumped.name), str(lumped.parent), index, node_owners)
        start = first
        if len(names) > 0:
            start = _named_chain(str(lumped.name), count, names)
        x, y, z = (float(value) for value in lumped.position)
        w, i, j, k = (float(value) for value in lumped.orientation)
        out[name] = NodeChain(
            first_body=start,
            count=count,
            root=lumped.root,
            parent_body=parent,
            position=(x, y, z),
            orientation=(w, i, j, k),
            segment_length=float(lumped.section.length) / int(lumped.segments),
            stiffness=float(lumped.section.elastic_modulus)
            * float(lumped.section.second_moment_of_area)
            * int(lumped.segments)
            / float(lumped.section.length),
            carries=len(tuple(getattr(lumped, "carries", ()))) > 0,
            carried=_carried_places(lumped, bodies, index),
        )
        first += count
    return out


def _carried_places(lumped: Any, bodies: tuple[Any, ...], index: Mapping[str, int]) -> tuple[tuple[int, int], ...]:
    """Each carried child body's place among the published bodies and its joint's among the jointed bodies (the order
    of `PhysicsOutputs.joints` and the recorded reactions)."""
    names = tuple(getattr(lumped, "carries", ()))
    if len(names) == 0:
        return ()
    order = [str(body.joint.name) for body in bodies if getattr(body, "joint", None) is not None]
    out: list[tuple[int, int]] = []
    for name in names:
        child = next((body for body in bodies if str(body.name) == str(name)), None)
        if child is None or getattr(child, "joint", None) is None or str(child.name) not in index:
            raise IntegrityError(f"member {str(lumped.name)!r}: carried body {str(name)!r} has no joint (ruling 75)")
        out.append((index[str(child.name)], order.index(str(child.joint.name))))
    return tuple(out)


SPRING_OFFSET_EVENT = "spring_offset spring "
SPRINGS_PER_JOINT = {"revolute": 1, "prismatic": 1, "planar": 3, "fixed": 0, "spherical": 0, "free": 0}


def spring_offsets(
    model: Any, events: tuple[tuple[int, Any], ...], rows: int, first_springs: Sequence[int] | None = None
) -> dict[str, FloatArray]:
    """Each lumped member's node spring offsets (rows, N) rad in effect at each row (INT-028, ruling 35), keyed by
    structural member name, from the recorded `model_change` events `spring_offset spring <i> value <v>` (the adapter's
    listing; the offset is absolute, rest = built + offset, applied before the step whose row lists it, so in effect
    from that row on). Spring order is the native adapter's: every revolute and prismatic DOF (one each) and planar
    joint (three) of the model's bodies in order, then each member's N node springs root to tip
    (`core/physics/native/CLAUDE.md`), read from the prepared native document (`first_springs`, each member's first
    spring, which the driver takes from the document: bodies a case adds at run time shift it, ruling 72); without it
    from the model's bodies, where a joint type the order does not name is an `IntegrityError`. Only members with an
    offset appear. A spring index outside the members' springs is an `IntegrityError`."""
    out: dict[str, FloatArray] = {}
    if model is None:
        return out
    changes = [
        (int(row), str(event.detail))
        for row, event in events
        if str(event.kind) == "model_change" and str(event.detail).startswith(SPRING_OFFSET_EVENT)
    ]
    if len(changes) == 0:
        return out
    first = 0
    for body in getattr(model, "bodies", ()):
        joint = getattr(body, "joint", None)
        if joint is not None and first_springs is None:
            kind = str(joint.type)
            if kind not in SPRINGS_PER_JOINT:
                raise IntegrityError(f"body {body.name!r}: joint type {kind!r} has no place in the spring order")
            first += SPRINGS_PER_JOINT[kind]
    lumped_members = tuple(getattr(model, "lumped_members", ()))
    if first_springs is not None and len(first_springs) != len(lumped_members):
        raise IntegrityError(
            f"the native document lists {len(first_springs)} members, the physics model {len(lumped_members)}"
        )
    blocks: list[tuple[str, int, int]] = []
    for position, lumped in enumerate(lumped_members):
        name = str(lumped.name)
        if lumped.structural_member is not None:
            name = str(lumped.structural_member)
        if first_springs is not None:
            first = int(first_springs[position])
        blocks.append((name, first, int(lumped.segments)))
        first += int(lumped.segments)
    for row, detail in changes:
        index_text, _, value_text = detail[len(SPRING_OFFSET_EVENT) :].partition(" value ")
        index = int(index_text)
        owner = None
        for name, start, count in blocks:
            if start <= index < start + count:
                owner = (name, index - start, count)
        if owner is None:
            raise IntegrityError(f"model change {detail!r}: spring {index} is no member node spring (INT-028)")
        name, node, count = owner
        if name not in out:
            out[name] = np.zeros((rows, count), dtype=np.float64)
        out[name][row:, node] = float(value_text)
    return out


def recorded_body_count(model: Any) -> int:
    """Neutral bodies plus every lumped member's node bodies: what a native run publishes per sample."""
    total = len(tuple(getattr(model, "bodies", ())))
    for lumped in getattr(model, "lumped_members", ()):
        total += int(lumped.segments)
        if lumped.root == "pinned":
            total += 1
    return total


def _is_node(name: str, node_owners: set[str]) -> bool:
    owner, separator, node = name.rpartition(".")
    return separator != "" and owner in node_owners and node.startswith("node")


def _parent_body(member: str, parent: str, index: Mapping[str, int], node_owners: set[str]) -> int | None:
    if parent == WORLD:
        return None
    if parent in index:
        return index[parent]
    if _is_node(parent, node_owners):
        raise IntegrityError(f"member {member!r}: parent {parent!r} is another member's node (INT-006)")
    raise IntegrityError(f"member {member!r}: unknown parent {parent!r}, neither the world nor a body (INT-006)")


def _named_chain(member: str, count: int, names: tuple[str, ...]) -> int:
    expected = [f"{member}.node{k}" for k in range(count)]
    if expected[0] not in names:
        raise IntegrityError(f"member {member!r}: the recording names no body {expected[0]!r} (INT-006)")
    start = names.index(expected[0])
    found = list(names[start : start + count])
    if found != expected:
        raise IntegrityError(f"member {member!r}: recorded node bodies {found} are not {expected} (INT-006)")
    return start


def _body_poses(series: Mapping[str, Any], rows: int, expected: int | None) -> BodyPoses | None:
    if BODY_POSITION not in series or BODY_ORIENTATION not in series:
        return None
    positions = np.asarray(series[BODY_POSITION].values, dtype=np.float64).reshape(rows, -1, 3)
    orientations = np.asarray(series[BODY_ORIENTATION].values, dtype=np.float64).reshape(rows, -1, 4)
    if positions.shape[1] == 0:
        return None
    if expected is not None and positions.shape[1] != expected:
        raise IntegrityError(
            f"the recording has {positions.shape[1]} body poses per sample, the model {expected} "
            "(neutral bodies plus member nodes, INT-006)"
        )
    return BodyPoses(positions, orientations)


def body_names(series: Mapping[str, Any]) -> tuple[str, ...]:
    """The recorder's body `names` for the pose series (empty when the recording does not carry them)."""
    if BODY_POSITION not in series:
        return ()
    return tuple(str(name) for name in getattr(series[BODY_POSITION], "names", ()))


def run_inputs(
    build: Any, recording: RecordView, materials: Mapping[str, MaterialView], design_load_carried: bool = False
) -> RunInputs:
    """`build` is the builder's `BuildResult`; `recording` the run's `RecordView` (a `RecordingView` carries the raw
    recording the end-loads come from). The load factor is the design's rule 1 + allowance (INT-019), or 1 when the
    case states that the run carries the design load (`design_load_carried`, TST-043, ruling 52)."""
    times = np.asarray(recording.times(), dtype=np.float64)
    rows = times.shape[0]
    raw = getattr(recording, "recording", None)
    members_list = tuple(getattr(build, "members", ()))
    series: Mapping[str, Any] = {}
    if raw is not None:
        series = raw.series
    elif members_list:
        raise IntegrityError("the recording carries no member end-loads (not a recorder recording)")
    calibrated: dict[str, float] = {}
    for record in getattr(build, "lumped", ()):
        if getattr(record, "calibrated", False) and record.calibration_frequency is not None:
            calibrated[str(record.member)] = float(record.calibration_frequency)
    placements = member_axes(build)
    model = getattr(build, "physics", None)
    names = body_names(series)
    chains = node_chains(model, names)
    events: tuple[tuple[int, Any], ...] = ()
    if raw is not None:
        events = tuple(getattr(raw, "events", ()))
    offsets = spring_offsets(model, events, rows, getattr(recording, "first_springs", None))
    carried = carried_loads(model, series, rows, names)
    members: list[MemberInputs] = []
    holder: dict[str, str] = {}
    for member in members_list:
        name = str(member.name)
        loads = _member_loads(series, name, rows, model is not None and name not in placements)
        supports = tuple((float(support.at), support.condition) for support in member.supports)
        mounts = tuple(str(mount) for mount in getattr(member, "mounts", ()))
        for mount in mounts:
            holder[mount] = name
        members.append(
            MemberInputs(
                name=name,
                material=str(member.material),
                section=section_properties(member.section),
                supports=supports,
                loads=loads,
                natural_frequency=calibrated.get(name),
                axis_body=placements.get(name, (1.0, 0.0, 0.0)),
                mounts=mounts,
                nodes=chains.get(name),
                spring_offsets=offsets.get(name),
                carried=carried.get(name, ()),
            )
        )
    joints = _joints(build, series, holder, rows)
    design = build.design
    load_factor = 1.0
    minimum: float | None = None
    limit: float | None = None
    design_tension: float | None = None
    structural = design.structural
    if structural is not None:
        rule = 1.0 + float(structural.design_load.allowance)
        if not design_load_carried:
            load_factor = rule
        gravity = float(np.linalg.norm(np.asarray(design.environment.gravity, dtype=np.float64)))
        design_tension = float(structural.design_load.rated_load) * rule * gravity
        if structural.minimum_safety_factor is not None:
            minimum = float(structural.minimum_safety_factor)
        limit = float(structural.accuracy_target)
    return RunInputs(
        times=times,
        members=tuple(members),
        materials=materials,
        temperature=float(design.environment.ambient_temperature),
        load_factor=load_factor,
        minimum_safety_factor=minimum,
        deflection_limit=limit,
        joints=joints,
        series=_series_reader(recording),
        ropes=_ropes(build, series, rows),
        design_tension=design_tension,
        bodies=_bodies(series, rows, model, chains, names),
    )


def _bodies(
    series: Mapping[str, Any], rows: int, model: Any, chains: Mapping[str, NodeChain], names: tuple[str, ...]
) -> BodyPoses | None:
    """The recorded body poses; with node chains (a native build of flexible members) they are required and their
    count must equal the recorder's body names when it names them (bodies a case adds at run time included), else the
    model's neutral bodies plus member nodes."""
    if len(chains) == 0:
        return _body_poses(series, rows, None)
    expected = recorded_body_count(model)
    if len(names) > 0:
        expected = len(names)
    poses = _body_poses(series, rows, expected)
    if poses is None:
        raise IntegrityError(
            f"the run recorded no body poses ({BODY_POSITION}) but members {sorted(chains)} are built flexible with "
            "node bodies (INT-006)"
        )
    return poses


def carried_loads(
    model: Any, series: Mapping[str, Any], rows: int, names: tuple[str, ...] = ()
) -> dict[str, tuple[CarriedLoad, ...]]:
    """Per carrying member (structural name) each carried joint's load on the member from the recorded joint
    reactions (`CarriedLoad`, ruling 72): the reaction's place is the joint's among the model's jointed bodies (bodies
    a run adds come after them), the child body's place in the recorded poses by the recorder's names when given
    (`carries` names the carried child bodies). A carried body without a joint reaction is an `IntegrityError` (its
    couple could not be bounded)."""
    out: dict[str, tuple[CarriedLoad, ...]] = {}
    if model is None:
        return out
    lumped_members = [lumped for lumped in getattr(model, "lumped_members", ()) if len(getattr(lumped, "carries", ()))]
    if len(lumped_members) == 0:
        return out
    bodies = tuple(model.bodies)
    order = [str(body.joint.name) for body in bodies if body.joint is not None]
    if REACTION_FORCE not in series or REACTION_MOMENT not in series:
        raise IntegrityError("the recording has no joint reactions for the carried joints (ruling 72)")
    forces = np.asarray(series[REACTION_FORCE].values, dtype=np.float64).reshape(rows, -1, 3)
    moments = np.asarray(series[REACTION_MOMENT].values, dtype=np.float64).reshape(rows, -1, 3)
    for lumped in lumped_members:
        name = str(lumped.structural_member or lumped.name)
        found: list[CarriedLoad] = []
        for carried_body in lumped.carries:
            child = next((body for body in bodies if str(body.name) == str(carried_body)), None)
            if child is None or child.joint is None:
                raise IntegrityError(f"member {name!r}: carried body {str(carried_body)!r} has no recorded reaction")
            index = order.index(str(child.joint.name))
            place = [str(body.name) for body in bodies].index(str(child.name))
            if len(names) > 0:
                place = names.index(str(child.name))
            found.append(CarriedLoad(str(child.joint.name), place, -forces[:, index, :], -moments[:, index, :]))
        out[name] = tuple(found)
    return out


def _ropes(build: Any, series: Mapping[str, Any], rows: int) -> tuple[RopeLoads, ...]:
    physics = getattr(build, "physics", None)
    if physics is None:
        return ()
    ropes = tuple(getattr(physics, "ropes", ()))
    if not ropes:
        return ()
    if ROPE_MAX_TENSION not in series:
        raise IntegrityError(f"the recording has no {ROPE_MAX_TENSION}")
    stored = series[ROPE_MAX_TENSION]
    names = tuple(stored.names)
    values = np.asarray(stored.values, dtype=np.float64).reshape(rows, -1)
    out: list[RopeLoads] = []
    for rope in ropes:
        if rope.name not in names:
            raise IntegrityError(f"rope {rope.name!r}: no recorded tension in {ROPE_MAX_TENSION}")
        breaking = rope.breaking_tension
        if breaking is not None:
            breaking = float(breaking)
        out.append(RopeLoads(str(rope.name), values[:, names.index(rope.name)].copy(), breaking))
    return tuple(out)


def _joints(build: Any, series: Mapping[str, Any], holder: Mapping[str, str], rows: int) -> tuple[JointLoads, ...]:
    physics = getattr(build, "physics", None)
    if physics is None:
        return ()
    fastenings = tuple(getattr(physics, "fastenings", ()))
    if not fastenings:
        return ()
    order = [body.joint.name for body in physics.bodies if body.joint is not None]
    if REACTION_FORCE not in series or REACTION_MOMENT not in series:
        raise IntegrityError("the recording has no joint reactions for the fastenings")
    forces = np.asarray(series[REACTION_FORCE].values, dtype=np.float64)
    moments = np.asarray(series[REACTION_MOMENT].values, dtype=np.float64)
    out: list[JointLoads] = []
    for fastening in fastenings:
        if fastening.joint not in order:
            raise IntegrityError(f"fastening {fastening.name!r}: joint {fastening.joint!r} has no recorded reaction")
        index = order.index(fastening.joint)
        out.append(
            JointLoads(
                name=str(fastening.name),
                kind=fastening.kind,
                joint=str(fastening.joint),
                force=forces[:, index, :].reshape(rows, 3),
                moment=moments[:, index, :].reshape(rows, 3),
                member=holder.get(str(fastening.joint)),
            )
        )
    return tuple(out)
