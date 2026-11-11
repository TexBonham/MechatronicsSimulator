"""Probes on bodies, joints, pins and signals (VIE-003 to VIE-006): recorded values over time from an open archive's
record view, by the `RecordView` references (`pose:`, `joint:`, `signal:`). Raw body velocities and joint forces
come from the archived recording when the view carries one (the core record view does)."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Literal, Protocol

import numpy as np

from mechsim.record.layout import JOINT_FORCE, PINS

ProbeKind = Literal["body", "joint", "pin", "signal"]
POSE_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")
VELOCITY_FIELDS = (("linear_velocity", "v", "m/s"), ("angular_velocity", "w", "rad/s"))
JOINT_FIELDS = ("position", "velocity")


class ProbeError(KeyError):
    """The recording has nothing under the probed name."""


class SeriesLike(Protocol):
    @property
    def times(self) -> Sequence[float]: ...

    @property
    def values(self) -> Sequence[float]: ...

    @property
    def unit(self) -> str: ...


class ViewLike(Protocol):
    def times(self) -> Sequence[float]: ...

    def series(self, reference: str) -> Any: ...


@dataclass(frozen=True)
class ProbeColumn:
    name: str
    unit: str
    values: tuple[float, ...]


@dataclass(frozen=True)
class Probe:
    kind: ProbeKind
    name: str
    times: tuple[float, ...]
    columns: tuple[ProbeColumn, ...]

    def value_at(self, index: int) -> dict[str, float]:
        return {column.name: column.values[index] for column in self.columns}


def _column(view: ViewLike, reference: str, name: str) -> ProbeColumn | None:
    try:
        series = view.series(reference)
    except KeyError:
        return None
    return ProbeColumn(name, str(series.unit), tuple(float(value) for value in series.values))


def _raw(view: ViewLike) -> tuple[Any, Any] | None:
    recording = getattr(view, "recording", None)
    kinematics = getattr(view, "kinematics", None)
    if recording is None or kinematics is None:
        return None
    return recording, kinematics


def _body_velocities(view: ViewLike, name: str) -> list[ProbeColumn]:
    raw = _raw(view)
    if raw is None:
        return []
    recording, kinematics = raw
    if name not in kinematics.bodies:
        return []
    index = kinematics.bodies.index(name)
    columns: list[ProbeColumn] = []
    for field, prefix, unit in VELOCITY_FIELDS:
        values = np.asarray(recording.series[f"physics/bodies/{field}"].values, float)[:, index, :]
        for axis, label in enumerate("xyz"):
            columns.append(ProbeColumn(f"{prefix}{label}", unit, tuple(float(v) for v in values[:, axis])))
    return columns


def _joint_force(view: ViewLike, name: str) -> list[ProbeColumn]:
    """Generalized force columns of the joint; joints are one per body in body order (the archived kinematics)."""
    raw = _raw(view)
    if raw is None:
        return []
    recording, kinematics = raw
    names = [joint.name for joint in kinematics.joints]
    series = recording.series.get(JOINT_FORCE)
    if name not in names or series is None or len(series.counts) != len(names):
        return []
    position = names.index(name)
    start = sum(series.counts[:position])
    values = np.asarray(series.values, float)
    columns: list[ProbeColumn] = []
    for offset in range(series.counts[position]):
        column = values[:, start + offset]
        columns.append(ProbeColumn(f"force[{offset}]", series.unit, tuple(float(v) for v in column)))
    return columns


def probe(view: ViewLike, kind: ProbeKind, name: str) -> Probe:
    """Every recorded value of the probed body (pose, then velocities), joint (position, velocity, then generalized
    force), pin or signal, over the recording's sample times."""
    columns: list[ProbeColumn] = []
    if kind == "body":
        for field in POSE_FIELDS:
            column = _column(view, f"pose:{name}.{field}", field)
            if column is not None:
                columns.append(column)
        if len(columns) > 0:
            columns.extend(_body_velocities(view, name))
    elif kind == "joint":
        for field in JOINT_FIELDS:
            column = _column(view, f"joint:{name}.{field}", field)
            if column is not None:
                columns.append(column)
        if len(columns) > 0:
            columns.extend(_joint_force(view, name))
    else:
        if kind == "pin" and not _is_pin(view, name):
            raise ProbeError(f"no pin {name!r} in the recording")
        column = _column(view, f"signal:{name}", "value")
        if column is not None:
            columns.append(column)
    if len(columns) == 0:
        raise ProbeError(f"no {kind} {name!r} in the recording")
    return Probe(kind, name, tuple(float(t) for t in view.times()), tuple(columns))


def _is_pin(view: ViewLike, name: str) -> bool:
    """A pin is a signal of the pins group; a view without the raw recording cannot tell, so any signal counts."""
    raw = _raw(view)
    if raw is None:
        return True
    recording, _ = raw
    return name in recording.named.get(PINS, ())
