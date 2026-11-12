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


def section_properties(section: Any) -> SectionProperties:
    """Both bending axes (ruling 25): `i_y`, `i_z`, `c_y`, `c_z` when the section carries them; otherwise the single
    second moment for both axes and c from `extreme_fibre`, else the solid rectangle with this A and I,
    sqrt(3 I / A)."""
    area = float(section.area)
    second = float(section.second_moment_of_area)
    fibre = getattr(section, "extreme_fibre", None)
    if fibre is None:
        fibre = math.sqrt(3.0 * second / area)
    values = {"i_y": second, "i_z": second, "c_y": float(fibre), "c_z": float(fibre)}
    for key in values:
        stated = getattr(section, key, None)
        if stated is not None:
            values[key] = float(stated)
    return SectionProperties(
        area=area,
        elastic_modulus=float(section.elastic_modulus),
        density=float(section.density),
        length=float(section.length),
        i_y=values["i_y"],
        i_z=values["i_z"],
        c_y=values["c_y"],
        c_z=values["c_z"],
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
                mounts=mounts,
            )
        )
    joints = _joints(build, series, holder, rows)
    design = build.design
    load_factor = 1.0
    minimum: float | None = None
    limit: float | None = None
    structural = design.structural
    if structural is not None:
        load_factor = 1.0 + float(structural.design_load.allowance)
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
    )


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
