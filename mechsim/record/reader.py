"""Reading a run back without the engine (TST-033, VIE-016, ruling 17): the model kinematics an archive carries, the
record view scoring and replay read, and the archive reader. Imports nothing from `mechsim._core`; joint coordinates
and poses are derived from the recorded body poses (TST-038)."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from mechsim.physics.neutral import PhysicsModel
from mechsim.record.archive import RunArchive
from mechsim.record.layout import CONTROLLER_STATE, NAMED_GROUPS, Recording
from mechsim.testing.driver import (
    MATERIAL_OVERRIDES_KEY,
    MESSAGE_KIND,
    IntegrityResults,
    MaterialOverridesRecord,
    RecordedEvent,
    RunMetadata,
    RunReport,
    Series,
    members_from_json,
)
from mechsim.testing.runner import RunnerError

OPERATOR_EVENT_KINDS = ("operator_status", "operator_command")
Vec = tuple[float, float, float]
Quat = tuple[float, float, float, float]


class DriverError(RunnerError):
    """The driver cannot run this build or case as asked (stopped before t = 0, scored as not run)."""


# ===== Kinematics =======================================================


def _q_mul(a: Sequence[float], b: Sequence[float]) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _q_conj(q: Sequence[float]) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def _q_rot(q: Sequence[float], v: Sequence[float]) -> Vec:
    r = _q_mul(_q_mul(q, (0.0, v[0], v[1], v[2])), _q_conj(q))
    return (r[1], r[2], r[3])


def _unit(v: Sequence[float]) -> Vec:
    n = math.sqrt(sum(c * c for c in v))
    return (v[0] / n, v[1] / n, v[2] / n)


@dataclass(frozen=True)
class JointKinematics:
    name: str
    kind: str
    parent: int | None
    child: int
    position: Vec
    orientation: Quat
    axis: Vec


@dataclass(frozen=True)
class Kinematics:
    """Body names (neutral order = the adapter's body order), reference-configuration world poses, joints."""

    bodies: tuple[str, ...]
    reference: tuple[tuple[Vec, Quat], ...]
    joints: tuple[JointKinematics, ...]

    @staticmethod
    def of(model: PhysicsModel) -> Kinematics:
        index = {body.name: position for position, body in enumerate(model.bodies)}
        poses: dict[str, tuple[Vec, Quat]] = {}
        joints: list[JointKinematics] = []
        for body in model.bodies:
            joint = body.joint
            if joint is None:
                raise DriverError(f"body '{body.name}': no joint to its parent")
            parent_pose: tuple[Vec, Quat] = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
            parent: int | None = None
            if body.parent != "world":
                parent_pose = poses[body.parent]
                parent = index[body.parent]
            offset = _q_rot(parent_pose[1], joint.position)
            position = (
                parent_pose[0][0] + offset[0],
                parent_pose[0][1] + offset[1],
                parent_pose[0][2] + offset[2],
            )
            poses[body.name] = (position, _q_mul(parent_pose[1], joint.orientation))
            axis: Vec = (0.0, 0.0, 1.0)
            if joint.axis is not None:
                axis = _unit(joint.axis)
            joints.append(
                JointKinematics(
                    joint.name,
                    joint.type,
                    parent,
                    index[body.name],
                    tuple(joint.position),  # type: ignore[arg-type]
                    tuple(joint.orientation),  # type: ignore[arg-type]
                    axis,
                )
            )
        return Kinematics(tuple(index), tuple(poses[name] for name in index), tuple(joints))

    def to_json(self) -> str:
        return json.dumps(
            {
                "bodies": list(self.bodies),
                "reference": [[list(p), list(q)] for p, q in self.reference],
                "joints": [
                    [j.name, j.kind, j.parent, j.child, list(j.position), list(j.orientation), list(j.axis)]
                    for j in self.joints
                ],
            },
            sort_keys=True,
            separators=(",", ":"),
        )

    @staticmethod
    def from_json(text: str) -> Kinematics:
        data = json.loads(text)
        return Kinematics(
            tuple(data["bodies"]),
            tuple((tuple(p), tuple(q)) for p, q in data["reference"]),
            tuple(
                JointKinematics(n, k, parent, child, tuple(pos), tuple(ori), tuple(axis))
                for n, k, parent, child, pos, ori, axis in data["joints"]
            ),
        )


def _rotate_rows(q: NDArray[np.float64], v: NDArray[np.float64]) -> NDArray[np.float64]:
    """Rotate row vectors v (N x 3) by row quaternions q (N x 4, w first)."""
    w = q[:, :1]
    u = q[:, 1:]
    t = 2.0 * np.cross(u, v)
    result: NDArray[np.float64] = v + w * t + np.cross(u, t)
    return result


def _conj_rows(q: NDArray[np.float64]) -> NDArray[np.float64]:
    result = q.copy()
    result[:, 1:] = -result[:, 1:]
    return result


def _mul_rows(a: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
    aw, ax, ay, az = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    bw, bx, by, bz = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=1,
    )


# ===== Record view ======================================================


class RecordingView:
    """`RecordView` over a recorder `Recording` and the model's kinematics (SCO-001, TST-038)."""

    def __init__(
        self,
        recording: Recording,
        kinematics: Kinematics,
        approximations: Sequence[str] = (),
        labels: Mapping[str, str] | None = None,
        actuators: Mapping[str, str] | None = None,
        states: Mapping[str, str] | None = None,
    ) -> None:
        self.states = dict(states or {})
        self._labels = dict(labels or {})
        self.labels = self._labels
        self.actuators = dict(actuators or {})
        self._recording = recording
        self._kinematics = kinematics
        self.approximations = tuple(approximations)
        self._times = tuple(float(value) for value in recording.series["time/seconds"].values)

    @property
    def recording(self) -> Recording:
        return self._recording

    @property
    def kinematics(self) -> Kinematics:
        return self._kinematics

    def times(self) -> Sequence[float]:
        return self._times

    def controller_states(self) -> Sequence[str]:
        states = self._recording.controller_states
        codes = tuple(states[int(index)] for index in self._recording.series[CONTROLLER_STATE].values)
        return tuple(self.states.get(code, code) for code in codes)

    def events(self) -> Sequence[RecordedEvent]:
        rate = self._recording.base_rate
        return tuple(
            self._event(float(Fraction(event.time) / rate), event.source, event.kind, event.detail)
            for _, event in self._recording.events
        )

    def _event(self, time: float, source: str, kind: str, detail: str) -> RecordedEvent:
        """An operator command or status, recorded under its label verbatim (REC-009), reads as an operator message
        named by the design's label table; the longest matching label wins."""
        if kind in OPERATOR_EVENT_KINDS:
            for label in sorted(self._labels, key=len, reverse=True):
                if detail == label or detail.startswith(label + " "):
                    return RecordedEvent(time, source, MESSAGE_KIND, self._labels[label])
        return RecordedEvent(time, source, kind, detail)

    def _body(self, name: str, field: str) -> NDArray[np.float64]:
        if name not in self._kinematics.bodies:
            raise KeyError(f"no body {name!r} in the recording")
        values: NDArray[np.float64] = np.asarray(self._recording.series[f"physics/bodies/{field}"].values, float)
        return values[:, self._kinematics.bodies.index(name), :]

    def _pose(self, body: str, component: str) -> NDArray[np.float64]:
        reference_position, reference_orientation = self._kinematics.reference[self._kinematics.bodies.index(body)]
        if component in ("x", "y", "z"):
            axis = "xyz".index(component)
            result: NDArray[np.float64] = self._body(body, "position")[:, axis] - reference_position[axis]
            return result
        orientation = self._body(body, "orientation")
        rows = len(orientation)
        reference = np.tile(np.asarray(_q_conj(reference_orientation)), (rows, 1))
        q = _mul_rows(orientation, reference)
        w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        if component == "roll":
            roll: NDArray[np.float64] = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            return roll
        if component == "pitch":
            pitch: NDArray[np.float64] = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
            return pitch
        yaw: NDArray[np.float64] = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw

    def _parent(self, joint: JointKinematics, field: str, rows: int) -> NDArray[np.float64]:
        if joint.parent is not None:
            return self._body(self._kinematics.bodies[joint.parent], field)
        if field == "orientation":
            return np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (rows, 1))
        return np.zeros((rows, 3))

    def _joint_unit(self, name: str, quantity: str) -> str:
        """m or rad for a position, per second for a velocity, by the joint's kind (prismatic or revolute)."""
        unit = "rad"
        for joint in self._kinematics.joints:
            if joint.name == name and joint.kind == "prismatic":
                unit = "m"
        if quantity == "velocity":
            return f"{unit}/s"
        return unit

    def _joint(self, name: str, quantity: str) -> NDArray[np.float64]:
        found = [joint for joint in self._kinematics.joints if joint.name == name]
        if len(found) == 0:
            raise KeyError(f"no joint {name!r} in the recording")
        joint = found[0]
        if joint.kind not in ("revolute", "prismatic"):
            raise KeyError(f"joint {name!r} ({joint.kind}) has no single coordinate")
        child = self._kinematics.bodies[joint.child]
        position = self._body(child, "position")
        orientation = self._body(child, "orientation")
        rows = len(position)
        parent_position = self._parent(joint, "position", rows)
        parent_orientation = self._parent(joint, "orientation", rows)
        axis_rows = np.tile(np.asarray(_q_rot(joint.orientation, joint.axis)), (rows, 1))
        world_axis = _rotate_rows(parent_orientation, axis_rows)
        if quantity == "velocity":
            if joint.kind == "revolute":
                relative = self._body(child, "angular_velocity") - self._parent(joint, "angular_velocity", rows)
            else:
                lever = position - parent_position
                relative = (
                    self._body(child, "linear_velocity")
                    - self._parent(joint, "linear_velocity", rows)
                    - np.cross(self._parent(joint, "angular_velocity", rows), lever)
                )
            speed: NDArray[np.float64] = np.einsum("ij,ij->i", relative, world_axis)
            return speed
        local = _rotate_rows(_conj_rows(parent_orientation), position - parent_position)
        if joint.kind == "prismatic":
            placement = np.asarray(joint.position)
            slide: NDArray[np.float64] = np.einsum("ij,ij->i", local - placement, axis_rows)
            return slide
        relative_q = _mul_rows(_conj_rows(parent_orientation), orientation)
        delta = _mul_rows(np.tile(np.asarray(_q_conj(joint.orientation)), (rows, 1)), relative_q)
        local_axis = np.asarray(joint.axis)
        angle = 2.0 * np.arctan2(delta[:, 1:] @ local_axis, delta[:, 0])
        unwrapped: NDArray[np.float64] = np.unwrap(angle)
        return unwrapped

    def series(self, reference: str) -> Series:
        kind, _, rest = reference.partition(":")
        name, _, field = rest.rpartition(".")
        times = self._times
        if kind == "pose":
            if name not in self._kinematics.bodies or field not in ("x", "y", "z", "roll", "pitch", "yaw"):
                raise KeyError(reference)
            unit = "rad"
            if field in ("x", "y", "z"):
                unit = "m"
            return Series(times, tuple(float(v) for v in self._pose(name, field)), unit)
        if kind == "actuator" and field == "position" and name in self.actuators:
            values = self._joint(self.actuators[name], field)
            return Series(times, tuple(float(v) for v in values), "rad")
        if kind == "joint" and field in ("position", "velocity"):
            values = self._joint(name, field)
            return Series(times, tuple(float(v) for v in values), self._joint_unit(name, field))
        if kind == "signal":
            for group in NAMED_GROUPS:
                if rest in self._recording.named[group]:
                    stored = self._recording.series[f"{group}/{rest}"]
                    return Series(times, tuple(float(v) for v in stored.values), stored.unit)
        raise KeyError(reference)


# ===== Archive reader ===================================================


class RecordArchiveReader:
    """`ArchiveReader`: metadata and a record view from the archive alone, no engine (TST-033)."""

    def open(self, path: Path) -> tuple[RunMetadata, RecordingView]:
        with RunArchive(path) as archive:
            identity = archive.identity
            extra = archive.extra
            recording = archive.recording()
            end = archive.end
        if "kinematics" not in extra:
            raise DriverError(f"{path}: not written by the record archiver (no kinematics)")
        reason: Any = extra["end_reason"]
        error: str | None = extra["error"]
        if error == "":
            error = None
        sweep_data = json.loads(extra["sweep"])
        sweep: tuple[str, str] | None = None
        if sweep_data is not None:
            sweep = (sweep_data[0], sweep_data[1])
        mode: str | None = identity.controller_mode
        control_rate: Fraction | None = identity.control_rate
        if mode == "none":
            mode = None
            control_rate = None
        approximations = tuple(json.loads(extra.get("approximations", "[]")))
        metadata = RunMetadata(
            run_id=extra["run_id"],
            case_id=identity.test_case_id,
            design_name=identity.design_name,
            design_version=identity.design_version,
            design_hash=identity.design_source_hash,
            case_hash=identity.test_case_hash,
            seed=identity.seed,
            controller_mode=mode,
            sweep=sweep,
            report=RunReport(
                reason,
                end.end_time,
                recording.base_rate,
                error,
                tuple(json.loads(extra.get("sink_errors", "[]"))),
                tuple(json.loads(extra.get("hook_errors", "[]"))),
                approximations,
            ),
            bindings=tuple((item[0], item[1]) for item in json.loads(extra["bindings"])),
            control_rate=control_rate,
            members=members_from_json(extra.get("members", "[]")),
            material_overrides=_overrides(extra),
            variation=_variation(extra.get("variation", "null")),
            integrity=_integrity(extra),
            physics_adapter=identity.physics_adapter,
        )
        labels = json.loads(extra.get("labels", "{}"))
        actuators = json.loads(extra.get("actuators", "{}"))
        states = json.loads(extra.get("states", "{}"))
        return metadata, RecordingView(
            recording, Kinematics.from_json(extra["kinematics"]), approximations, labels, actuators, states
        )


def _overrides(extra: dict[str, str]) -> MaterialOverridesRecord | None:
    if MATERIAL_OVERRIDES_KEY not in extra:
        return None
    return MaterialOverridesRecord(extra[MATERIAL_OVERRIDES_KEY], extra[f"{MATERIAL_OVERRIDES_KEY}_sha256"])


def _variation(text: str) -> tuple[tuple[str, str], ...] | None:
    data = json.loads(text)
    if data is None:
        return None
    return tuple((str(item[0]), str(item[1])) for item in data)


def _integrity(extra: dict[str, str]) -> IntegrityResults | None:
    if "integrity" not in extra:
        return None
    return IntegrityResults.from_json(extra["integrity"])
