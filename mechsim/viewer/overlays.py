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
    def second_moment(self) -> float: ...

    def fibre(self) -> float: ...


class ViewLike(Protocol):
    def times(self) -> Sequence[float]: ...

    def series(self, reference: str) -> Any: ...


@dataclass(frozen=True)
class MemberStress:
    """Bending stress magnitude at the member's start and end, Pa, per sample: M c / I with M the bending moment
    sqrt(M_y^2 + M_z^2) of the recorded end moment (member frame, x along the member, ENG-030), I the section's second
    moment of area and c its extreme-fibre distance, both from the run's member list (`MemberSection.fibre`)."""

    member: str
    fibre: float
    second_moment: float
    start: tuple[float, ...]
    end: tuple[float, ...]


@dataclass(frozen=True)
class SwingAngle:
    """Angle of the line from the attachment point to the load from the downward vertical, rad, per sample."""

    load: str
    attachment: str
    values: tuple[float, ...]


def _bending(moments: Any) -> list[float]:
    values = np.asarray(moments, dtype=float)
    return [float(value) for value in np.hypot(values[:, 1], values[:, 2])]


def member_stress(view: ViewLike, members: Sequence[SectionLike]) -> tuple[MemberStress, ...]:
    """One `MemberStress` per recorded member column, in recorded order. The member columns must be named (REC-004
    labels); a recorded member missing from the member list is an `OverlayError`."""
    recording = getattr(view, "recording", None)
    if recording is None:
        raise OverlayError("member stress needs the archived recording")
    start_series = recording.series.get(MEMBER_SERIES[0])
    end_series = recording.series.get(MEMBER_SERIES[2])
    if start_series is None or end_series is None:
        raise OverlayError("the recording has no member end-loads")
    start_values = np.asarray(start_series.values, dtype=float)
    end_values = np.asarray(end_series.values, dtype=float)
    columns = 0
    if start_values.ndim == 3:
        columns = int(start_values.shape[1])
    if columns > 0 and len(start_series.names) != columns:
        raise OverlayError("the recorded member columns carry no names")
    sections = {member.name: member for member in members}
    found: list[MemberStress] = []
    for index, name in enumerate(start_series.names):
        if name not in sections:
            raise OverlayError(f"member {name!r} is not in the run's member list")
        section = sections[name]
        scale = section.fibre() / section.second_moment
        found.append(
            MemberStress(
                name,
                section.fibre(),
                section.second_moment,
                tuple(value * scale for value in _bending(start_values[:, index, :])),
                tuple(value * scale for value in _bending(end_values[:, index, :])),
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
