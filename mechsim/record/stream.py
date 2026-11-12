"""Streaming consumer: accepts tick records one at a time, forwards them to subscribers, flushes to an archive."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, fields
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from mechsim.record.archive import RunEnd, RunIdentity, write_archive
from mechsim.record.layout import (
    CONSTRAINT_FORCE,
    CONTROLLER_STATE,
    COUPLING_DIGEST,
    COUPLING_VALUE,
    DIGEST_LENGTH,
    DRIVE_DRIVEN,
    DRIVE_TORQUE,
    FIXED_SERIES,
    JOINT_FORCE,
    JOINT_POSITIONS,
    NAMED_GROUPS,
    ROPE_MAX_TENSION,
    TICK_AXIS,
    Columns,
    Recording,
    Series,
    check_columns,
    check_name,
    check_rate,
    check_text,
    label_series,
    seconds,
)
from mechsim.record.tick import COUPLING_KINDS, RecordedEvent, RecordError, SeriesSample, TickRecord

TickCallback = Callable[[TickRecord], None]

_INITIAL_ROWS = 1024


class _Buffer:
    """A numpy array growing by doubling along its first axis."""

    def __init__(self, tail: tuple[int, ...], dtype: str) -> None:
        self._data: NDArray[Any] = np.zeros((_INITIAL_ROWS, *tail), dtype=dtype)
        self._rows = 0
        self._empty_rows = int(np.prod(tail)) == 0

    def append(self, value: object) -> None:
        if self._rows == self._data.shape[0]:
            grown = np.zeros((2 * self._rows, *self._data.shape[1:]), dtype=self._data.dtype)
            grown[: self._rows] = self._data
            self._data = grown
        if not self._empty_rows:
            self._data[self._rows] = value
        self._rows += 1

    def array(self) -> NDArray[Any]:
        return self._data[: self._rows].copy()


@dataclass(frozen=True)
class _Layout:
    """Entity counts and named series fixed by the first row; every later row must match (array shapes)."""

    bodies: int = 0
    joint_dofs: tuple[int, ...] = ()
    constraint_rows: tuple[int, ...] = ()
    members: int = 0
    ropes: int = 0
    transmissions: int = 0
    joint_coordinates: int = 0
    payloads: int = 0
    named: tuple[tuple[tuple[str, str], ...], ...] = ((), (), ())
    coupling: tuple[str, ...] = ()

    def units(self) -> dict[str, str]:
        units: dict[str, str] = {}
        for group, entries in zip(NAMED_GROUPS, self.named, strict=True):
            for name, unit in entries:
                units[f"{group}/{name}"] = unit
        return units


def _layout(row: TickRecord) -> _Layout:
    physics = row.physics
    return _Layout(
        bodies=len(physics.bodies),
        joint_dofs=tuple(len(joint.generalized_force) for joint in physics.joints),
        constraint_rows=tuple(len(constraint.force) for constraint in physics.constraints),
        members=len(physics.members),
        ropes=len(physics.ropes),
        transmissions=len(physics.transmissions),
        joint_coordinates=len(physics.joint_positions),
        payloads=len(physics.payloads),
        named=tuple(tuple((sample.name, sample.unit) for sample in samples) for samples in _named(row)),
        coupling=tuple(sample.name for sample in row.coupling),
    )


def _drive_columns(row: TickRecord) -> tuple[list[float], list[bool]]:
    """Drive torques as one column per joint coordinate (0 where undriven) and which joints a drive acted on; a joint
    takes at most one drive per tick (KER-027)."""
    joints = len(row.physics.joint_positions)
    torques = [0.0] * joints
    driven = [False] * joints
    for drive in row.physics.joint_drive_torques:
        if drive.joint < 0 or drive.joint >= joints:
            raise RecordError(
                f"tick {row.time}: drive torque on joint {drive.joint} outside the {joints} joint positions"
            )
        if driven[drive.joint]:
            raise RecordError(f"tick {row.time}: two drive torques on joint {drive.joint}")
        torques[drive.joint] = drive.torque
        driven[drive.joint] = True
    return torques, driven


def _check_widths(layout: _Layout, columns: Mapping[str, Columns], time: int) -> None:
    for path, labels in columns.items():
        width = layout.payloads
        if path.startswith("physics/members/"):
            width = layout.members
        elif path.startswith("physics/ropes/"):
            width = layout.ropes
        elif path.startswith("physics/transmissions/"):
            width = layout.transmissions
        elif path.startswith("physics/joint"):
            width = layout.joint_coordinates
        if len(labels.names) != width:
            raise RecordError(f"tick {time}: {path} has {width} columns, {len(labels.names)} names given")


def _named(row: TickRecord) -> tuple[tuple[SeriesSample, ...], ...]:
    return (row.pin_signals, row.rails, row.actuator_efforts)


def _check_named(layout: _Layout) -> None:
    for group, entries in zip(NAMED_GROUPS, layout.named, strict=True):
        names = [entry[0] for entry in entries]
        for name, unit in entries:
            check_name(name, group)
            check_text(unit, f"{group} unit of {name!r}")
        if len(set(names)) != len(names):
            raise RecordError(f"{group}: duplicate series names in {names}")
    for name in layout.coupling:
        check_name(name, "coupling")
    if list(layout.coupling) != sorted(set(layout.coupling)):
        raise RecordError(f"coupling: names must be unique and in name order, got {list(layout.coupling)}")


class Recorder:
    """The Python side of the recorder sink (REC-019): `write_row` per physics tick, in time order; subscribers
    receive each row after it is stored, synchronously, in subscription order, and cannot change it (REC-020)."""

    def __init__(self, base_rate: Fraction, columns: Mapping[str, Columns] | None = None) -> None:
        """`columns`: per-column names and units of the index-column series, from the neutral model (REC-018); their
        counts are checked against the first row."""
        check_rate(base_rate, "base rate")
        self._columns = dict(columns or {})
        check_columns(self._columns)
        self._base_rate = base_rate
        self._layout: _Layout | None = None
        self._buffers: dict[str, _Buffer] = {}
        self._states: dict[str, int] = {}
        self._events: list[tuple[int, RecordedEvent]] = []
        self._coupling_kinds: dict[str, str] = {}
        self._last_time = -1
        self._rows = 0
        self._subscribers: dict[int, TickCallback] = {}
        self._next_id = 1

    @property
    def rows(self) -> int:
        return self._rows

    def subscribe(self, callback: TickCallback) -> int:
        subscription = self._next_id
        self._next_id += 1
        self._subscribers[subscription] = callback
        return subscription

    def unsubscribe(self, subscription: int) -> None:
        if subscription not in self._subscribers:
            raise RecordError(f"no subscription {subscription}")
        del self._subscribers[subscription]

    def write_row(self, row: TickRecord) -> None:
        if row.time <= self._last_time:
            raise RecordError(f"tick {row.time}: rows must advance in simulated time, previous {self._last_time}")
        layout = _layout(row)
        if self._layout is None:
            _check_named(layout)
            _check_widths(layout, self._columns, row.time)
            self._layout = layout
            self._buffers = _make_buffers(layout)
        else:
            for item in fields(_Layout):
                expected = getattr(self._layout, item.name)
                found = getattr(layout, item.name)
                if found != expected:
                    raise RecordError(f"tick {row.time}: {item.name} changed from {expected} to {found}")
        self._store(row)
        self._last_time = row.time
        self._rows += 1
        for callback in list(self._subscribers.values()):
            callback(row)

    def _store(self, row: TickRecord) -> None:
        check_text(row.controller_state, "controller state")
        for entry in row.coupling:
            self._check_coupling(row.time, entry.name, entry.kind, entry.digest)
        for event in row.events:
            check_text(event.source, "event source")
            check_text(event.kind, "event kind")
            check_text(event.detail, "event detail")
        physics = row.physics
        buffers = self._buffers
        buffers["time/tick"].append(row.time)
        buffers["physics/steps"].append(physics.steps)
        buffers["physics/bodies/position"].append([body.position for body in physics.bodies])
        buffers["physics/bodies/orientation"].append([body.orientation for body in physics.bodies])
        buffers["physics/bodies/angular_velocity"].append([body.angular_velocity for body in physics.bodies])
        buffers["physics/bodies/linear_velocity"].append([body.linear_velocity for body in physics.bodies])
        buffers[JOINT_FORCE].append([value for joint in physics.joints for value in joint.generalized_force])
        buffers["physics/joints/reaction_moment"].append([joint.reaction.moment for joint in physics.joints])
        buffers["physics/joints/reaction_force"].append([joint.reaction.force for joint in physics.joints])
        buffers[CONSTRAINT_FORCE].append([value for item in physics.constraints for value in item.force])
        buffers["physics/constraints/translational_error"].append(
            [item.translational_error for item in physics.constraints]
        )
        buffers["physics/constraints/rotational_error"].append([item.rotational_error for item in physics.constraints])
        for contact in physics.contacts:
            buffers["physics/contacts/row"].append(self._rows)
            buffers["physics/contacts/tick"].append(row.time)
            buffers["physics/contacts/body_a"].append(contact.body_a)
            buffers["physics/contacts/body_b"].append(contact.body_b)
            buffers["physics/contacts/point"].append(contact.point)
            buffers["physics/contacts/force"].append(contact.force)
        buffers["physics/members/start_moment"].append([member.start.moment for member in physics.members])
        buffers["physics/members/start_force"].append([member.start.force for member in physics.members])
        buffers["physics/members/end_moment"].append([member.end.moment for member in physics.members])
        buffers["physics/members/end_force"].append([member.end.force for member in physics.members])
        buffers["physics/ropes/tension"].append([rope.tension for rope in physics.ropes])
        buffers["physics/ropes/slack"].append([rope.slack for rope in physics.ropes])
        buffers[ROPE_MAX_TENSION].append([rope.max_tension for rope in physics.ropes])
        buffers["physics/transmissions/slip"].append([item.slip for item in physics.transmissions])
        buffers["physics/transmissions/slipping"].append([item.slipping for item in physics.transmissions])
        buffers["physics/transmissions/shaft_load_torque"].append(
            [item.shaft_load_torque for item in physics.transmissions]
        )
        buffers["physics/transmissions/force"].append([item.force for item in physics.transmissions])
        buffers["physics/transmissions/stretch"].append([item.stretch for item in physics.transmissions])
        buffers["physics/transmissions/power_flow"].append([item.power_flow for item in physics.transmissions])
        torques, driven = _drive_columns(row)
        buffers[JOINT_POSITIONS].append(list(physics.joint_positions))
        buffers[DRIVE_TORQUE].append(torques)
        buffers[DRIVE_DRIVEN].append(driven)
        buffers["physics/payloads/body"].append([item.body for item in physics.payloads])
        buffers["physics/payloads/carrier"].append([item.carrier for item in physics.payloads])
        buffers["physics/payloads/attached"].append([item.attached for item in physics.payloads])
        buffers["physics/payloads/force"].append([item.force for item in physics.payloads])
        for coupling in row.coupling:
            buffers[f"{COUPLING_VALUE}/{coupling.name}"].append(coupling.value)
            buffers[f"{COUPLING_DIGEST}/{coupling.name}"].append(coupling.digest.encode("ascii"))
        for group, samples in zip(NAMED_GROUPS, _named(row), strict=True):
            for sample in samples:
                buffers[f"{group}/{sample.name}"].append(sample.value)
        if row.controller_state not in self._states:
            self._states[row.controller_state] = len(self._states)
        buffers[CONTROLLER_STATE].append(self._states[row.controller_state])
        for event in row.events:
            self._events.append((self._rows, event))

    def _check_coupling(self, time: int, name: str, kind: str, digest: str) -> None:
        """A variable keeps one kind; `none` (not yet published) may precede or stand in for any kind."""
        if kind not in COUPLING_KINDS:
            raise RecordError(f"tick {time}: coupling {name!r} has unknown kind {kind!r}")
        if kind == "payload" and len(digest) != DIGEST_LENGTH:
            raise RecordError(f"tick {time}: coupling {name!r} payload digest must be {DIGEST_LENGTH} hex characters")
        if kind != "payload" and digest != "":
            raise RecordError(f"tick {time}: coupling {name!r} of kind {kind!r} carries no digest")
        known = self._coupling_kinds.get(name, "none")
        if kind == "none":
            self._coupling_kinds.setdefault(name, "none")
            return
        if known not in ("none", kind):
            raise RecordError(f"tick {time}: coupling {name!r} changed kind from {known} to {kind}")
        self._coupling_kinds[name] = kind

    def recording(self) -> Recording:
        """A snapshot of everything recorded so far, in canonical series order."""
        layout = self._layout
        buffers = self._buffers
        if layout is None:
            layout = _Layout()
            buffers = _make_buffers(layout)
        series: dict[str, Series] = {}
        for path, unit, axis, _ in FIXED_SERIES:
            counts: tuple[int, ...] = ()
            if path == JOINT_FORCE:
                counts = layout.joint_dofs
            if path == CONSTRAINT_FORCE:
                counts = layout.constraint_rows
            if path == "time/seconds":
                values = seconds(buffers["time/tick"].array(), self._base_rate)
            elif path == "physics/contacts/seconds":
                values = seconds(buffers["physics/contacts/tick"].array(), self._base_rate)
            else:
                values = buffers[path].array()
            series[path] = Series(values, unit, axis, counts)
        for path, unit in layout.units().items():
            series[path] = Series(buffers[path].array(), unit, TICK_AXIS)
        series[CONTROLLER_STATE] = Series(buffers[CONTROLLER_STATE].array(), "1", TICK_AXIS)
        for name in layout.coupling:
            series[f"{COUPLING_VALUE}/{name}"] = Series(buffers[f"{COUPLING_VALUE}/{name}"].array(), "SI", TICK_AXIS)
            series[f"{COUPLING_DIGEST}/{name}"] = Series(buffers[f"{COUPLING_DIGEST}/{name}"].array(), "1", TICK_AXIS)
        named = {
            group: tuple(entry[0] for entry in entries)
            for group, entries in zip(NAMED_GROUPS, layout.named, strict=True)
        }
        coupling = tuple((name, self._coupling_kinds.get(name, "none")) for name in layout.coupling)
        if self._layout is not None:
            series = label_series(series, self._columns)
        return Recording(self._base_rate, series, named, tuple(self._states), tuple(self._events), coupling)

    def flush(self, path: Path, identity: RunIdentity, end: RunEnd) -> None:
        """Write everything recorded to one archive (REC-012)."""
        write_archive(path, identity, end, self.recording())


def _make_buffers(layout: _Layout) -> dict[str, _Buffer]:
    joints = len(layout.joint_dofs)
    constraints = len(layout.constraint_rows)
    tails: dict[str, tuple[int, ...]] = {
        "time/tick": (),
        "physics/steps": (),
        "physics/bodies/position": (layout.bodies, 3),
        "physics/bodies/orientation": (layout.bodies, 4),
        "physics/bodies/angular_velocity": (layout.bodies, 3),
        "physics/bodies/linear_velocity": (layout.bodies, 3),
        JOINT_FORCE: (sum(layout.joint_dofs),),
        "physics/joints/reaction_moment": (joints, 3),
        "physics/joints/reaction_force": (joints, 3),
        CONSTRAINT_FORCE: (sum(layout.constraint_rows),),
        "physics/constraints/translational_error": (constraints,),
        "physics/constraints/rotational_error": (constraints,),
        "physics/contacts/row": (),
        "physics/contacts/tick": (),
        "physics/contacts/body_a": (),
        "physics/contacts/body_b": (),
        "physics/contacts/point": (3,),
        "physics/contacts/force": (3,),
        "physics/members/start_moment": (layout.members, 3),
        "physics/members/start_force": (layout.members, 3),
        "physics/members/end_moment": (layout.members, 3),
        "physics/members/end_force": (layout.members, 3),
        "physics/ropes/tension": (layout.ropes,),
        "physics/ropes/slack": (layout.ropes,),
        ROPE_MAX_TENSION: (layout.ropes,),
        "physics/transmissions/slip": (layout.transmissions,),
        "physics/transmissions/slipping": (layout.transmissions,),
        "physics/transmissions/shaft_load_torque": (layout.transmissions,),
        "physics/transmissions/force": (layout.transmissions,),
        "physics/transmissions/stretch": (layout.transmissions,),
        "physics/transmissions/power_flow": (layout.transmissions,),
        JOINT_POSITIONS: (layout.joint_coordinates,),
        DRIVE_TORQUE: (layout.joint_coordinates,),
        DRIVE_DRIVEN: (layout.joint_coordinates,),
        "physics/payloads/body": (layout.payloads,),
        "physics/payloads/carrier": (layout.payloads,),
        "physics/payloads/attached": (layout.payloads,),
        "physics/payloads/force": (layout.payloads, 3),
        CONTROLLER_STATE: (),
    }
    dtypes = {path: dtype for path, _, _, dtype in FIXED_SERIES}
    dtypes[CONTROLLER_STATE] = "int64"
    buffers = {path: _Buffer(tail, dtypes[path]) for path, tail in tails.items()}
    for path in layout.units():
        buffers[path] = _Buffer((), "float64")
    for name in layout.coupling:
        buffers[f"{COUPLING_VALUE}/{name}"] = _Buffer((), "float64")
        buffers[f"{COUPLING_DIGEST}/{name}"] = _Buffer((), f"S{DIGEST_LENGTH}")
    return buffers
