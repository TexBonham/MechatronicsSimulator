"""Replay overlays (VIE-010, VIE-011): member bending stress over time from the recorded end-loads and the run's
member list, and the swing angle of a suspended load over time from the recorded poses. Series the client draws;
computed from the recording alone (VIE-016)."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np

from mechsim.design.schema import yaml_loader
from mechsim.record.layout import MEMBER_SERIES
from mechsim.units import UnitError, parse_quantity

STRESS_UNIT = "Pa"
ANGLE_UNIT = "rad"
DOWN: tuple[float, float, float] = (0.0, 0.0, -1.0)


class OverlayError(KeyError):
    """The recording or the member list lacks what the overlay needs."""


class SectionLike(Protocol):
    @property
    def name(self) -> str: ...

    @property
    def area(self) -> float: ...

    @property
    def second_moment_y(self) -> float | None: ...

    @property
    def second_moment_z(self) -> float: ...

    @property
    def fibre_y(self) -> float | None: ...

    @property
    def fibre_z(self) -> float | None: ...


class ViewLike(Protocol):
    def times(self) -> Sequence[float]: ...

    def series(self, reference: str) -> Any: ...


@dataclass(frozen=True)
class MemberStress:
    """Peak normal stress at the member's start and end, Pa, per sample (ruling 25): |N| / A + |M_y| c_z / I_y +
    |M_z| c_y / I_z, with N the axial force and M_y, M_z the bending moments of the recorded end-loads in the member
    frame (x from the start to the end node, y and z the section's principal axes; ENG-030, ruling 24) and A, I_y,
    I_z, c_y, c_z from the run's member list."""

    member: str
    area: float
    second_moment_y: float
    second_moment_z: float
    fibre_y: float
    fibre_z: float
    start: tuple[float, ...]
    end: tuple[float, ...]


@dataclass(frozen=True)
class SwingAngle:
    """Angle of the line from the attachment point to the load from the downward vertical, rad, per sample."""

    load: str
    attachment: str
    values: tuple[float, ...]


def _stress(forces: Any, moments: Any, section: SectionLike, i_y: float, c_y: float, c_z: float) -> tuple[float, ...]:
    f = np.asarray(forces, dtype=float)
    m = np.asarray(moments, dtype=float)
    values = (
        np.abs(f[:, 0]) / section.area + np.abs(m[:, 1]) * c_z / i_y + np.abs(m[:, 2]) * c_y / section.second_moment_z
    )
    return tuple(float(value) for value in values)


def member_stress(view: ViewLike, members: Sequence[SectionLike]) -> tuple[MemberStress, ...]:
    """One `MemberStress` per recorded member column, in recorded order. The member columns must be named (REC-004
    labels); a recorded member missing from the member list, or listed without its y-axis second moment and fibre
    distances (a section given only by area and second moment), is an `OverlayError`."""
    recording = getattr(view, "recording", None)
    if recording is None:
        raise OverlayError("member stress needs the archived recording")
    series = [recording.series.get(path) for path in MEMBER_SERIES]
    if any(item is None for item in series):
        raise OverlayError("the recording has no member end-loads")
    start_moment, start_force, end_moment, end_force = (np.asarray(item.values, dtype=float) for item in series)
    names = series[0].names
    columns = 0
    if start_moment.ndim == 3:
        columns = int(start_moment.shape[1])
    if columns > 0 and len(names) != columns:
        raise OverlayError("the recorded member columns carry no names")
    sections = {member.name: member for member in members}
    found: list[MemberStress] = []
    for index, name in enumerate(names):
        if name not in sections:
            raise OverlayError(f"member {name!r} is not in the run's member list")
        section = sections[name]
        i_y = section.second_moment_y
        c_y = section.fibre_y
        c_z = section.fibre_z
        if i_y is None or c_y is None or c_z is None:
            raise OverlayError(f"member {name!r}: the section states no y-axis second moment or fibre distances")
        found.append(
            MemberStress(
                name,
                section.area,
                i_y,
                section.second_moment_z,
                c_y,
                c_z,
                _stress(start_force[:, index, :], start_moment[:, index, :], section, i_y, c_y, c_z),
                _stress(end_force[:, index, :], end_moment[:, index, :], section, i_y, c_y, c_z),
            )
        )
    return tuple(found)


def _position(view: ViewLike, body: str) -> list[tuple[float, float, float]]:
    try:
        axes = [view.series(f"pose:{body}.{axis}").values for axis in ("x", "y", "z")]
    except KeyError as error:
        raise OverlayError(f"no body {body!r} in the recording") from error
    return [(float(x), float(y), float(z)) for x, y, z in zip(*axes, strict=True)]


def down_from_design(design_file: bytes) -> tuple[float, float, float]:
    """The unit vector along the design's `environment.gravity`; -z when the file states none or zero gravity."""
    try:
        data = yaml_loader().load(design_file.decode("utf-8"))
        gravity = [parse_quantity(item, "[length] / [time] ** 2") for item in data["environment"]["gravity"]]
    except (UnicodeDecodeError, KeyError, TypeError, ValueError, UnitError):
        return DOWN
    norm = math.sqrt(sum(value * value for value in gravity))
    if len(gravity) != 3 or norm == 0.0:
        return DOWN
    return (gravity[0] / norm, gravity[1] / norm, gravity[2] / norm)


def swing_angle(
    view: ViewLike,
    load: str,
    attachment: str,
    point: tuple[float, float, float] = (0.0, 0.0, 0.0),
    down: tuple[float, float, float] = DOWN,
) -> SwingAngle:
    """The swing angle of `load` (its recorded body origin) about the attachment: `attachment` names the carrier body
    whose recorded origin is the attachment point, or is `world` with `point` the fixed attachment, m. With r the
    attachment-to-load vector and `down` the unit vector along gravity, the angle is atan2(|r x down|, r . down):
    0 hanging straight down, pi / 2 level."""
    loads = _position(view, load)
    anchors = [point] * len(loads)
    if attachment != "world":
        anchors = _position(view, attachment)
    values: list[float] = []
    for (lx, ly, lz), (ax, ay, az) in zip(loads, anchors, strict=True):
        r = (lx - ax, ly - ay, lz - az)
        along = r[0] * down[0] + r[1] * down[1] + r[2] * down[2]
        cross = (r[1] * down[2] - r[2] * down[1], r[2] * down[0] - r[0] * down[2], r[0] * down[1] - r[1] * down[0])
        values.append(math.atan2(math.sqrt(cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2), along))
    return SwingAngle(load, attachment, tuple(values))
