"""Run inputs from a finished run: the build's member list and fastenings, the design's structural data, and the
recorded member end-loads and joint reactions (REC-004)."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

import numpy as np

from mechsim.integrity.plugin import (
    FloatArray,
    IntegrityError,
    JointLoads,
    MaterialView,
    MemberInputs,
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


def run_inputs(build: Any, recording: RecordView, materials: Mapping[str, MaterialView]) -> RunInputs:
    """`build` is the builder's `BuildResult`; `recording` the run's `RecordView` (a `RecordingView` carries the raw
    recording the end-loads come from)."""
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
    members: list[MemberInputs] = []
    holder: dict[str, str] = {}
    for member in members_list:
        name = str(member.name)
        loads = Wrenches(*(_column(series, path, name, rows) for path in MEMBER_SERIES))
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
        load_factor = 1.0 + float(structural.design_load.allowance)
        gravity = float(np.linalg.norm(np.asarray(design.environment.gravity, dtype=np.float64)))
        design_tension = float(structural.design_load.rated_load) * load_factor * gravity
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
    )


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
