"""One HDF5 run archive per run through h5py (REC-012, L8): writer, reader and replay.

Byte-identical for identical inputs (REC-022, REC-023): fixed write order, fixed chunk rows, no compression, no
object timestamps, fixed-length strings in datasets, and nothing taken from the wall clock.
"""

from __future__ import annotations

import math
import re
import subprocess
from collections.abc import Iterator, Mapping
from dataclasses import dataclass
from fractions import Fraction
from importlib import metadata
from pathlib import Path
from types import TracebackType
from typing import Any

import h5py  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray

from mechsim.record.layout import (
    COLUMN_SERIES,
    CONSTRAINT_FORCE,
    CONTACT_ROW,
    CONTROLLER_STATE,
    COUPLING,
    COUPLING_DIGEST,
    COUPLING_VALUE,
    DRIVE_DRIVEN,
    DRIVE_TORQUE,
    JOINT_FORCE,
    JOINT_POSITIONS,
    NAMED_GROUPS,
    ROPE_MAX_TENSION,
    Columns,
    Recording,
    Series,
    check_rate,
    check_text,
    label_series,
    seconds,
)
from mechsim.record.tick import (
    BodyState,
    ConstraintForce,
    ContactForce,
    CouplingSample,
    JointDriveTorque,
    JointForce,
    MemberEndLoads,
    PayloadState,
    PhysicsOutputs,
    Quat,
    RecordedEvent,
    RecordError,
    RopeTension,
    SeriesSample,
    TickRecord,
    TransmissionState,
    Vec3,
    Wrench,
)

FORMAT = "mechsim-run-archive"
FORMAT_VERSION = 2
# Version 1 lacks the joint, drive and payload series and the column labels; they read as empty.
READABLE_VERSIONS = (1, 2)
CHUNK_ROWS = 1024
CONTROLLER_MODES = ("native", "emulated", "none")
END_CONDITION_MET = "end_condition_met"
RUN_ERROR = "run_error"
UNSET = "unset"
_HASH = re.compile(r"^([0-9a-f]{40}|unset)$")
_SEMVER = re.compile(r"^\d+\.\d+\.\d+(?:-[0-9A-Za-z.\-]+)?(?:\+[0-9A-Za-z.\-]+)?$")
_UINT64_MAX = 2**64 - 1
_STATE_NAMES = "controller/state_names"
_EVENTS = "events"
_DESIGN_FILE = "inputs/design_file"
_TEST_CASE_FILE = "inputs/test_case_file"


# ===== Run records ==========================================================
@dataclass(frozen=True)
class RunIdentity:
    """What produced the run (REC-013, REC-014): design, test case, seed, mode, rates and git hashes. Hashes are
    40-hex git hashes or `unset`; the design and test case file contents ride along for replay (REC-024)."""

    design_name: str
    design_version: str
    design_source_hash: str
    test_case_id: str
    test_case_hash: str
    seed: int
    controller_mode: str
    physics_rate: Fraction
    control_rate: Fraction
    repository_commit: str
    tool_version: str
    design_file: bytes = b""
    test_case_file: bytes = b""

    def __post_init__(self) -> None:
        for what, text in (("design name", self.design_name), ("test case id", self.test_case_id)):
            if text == "":
                raise RecordError(f"{what} is empty")
            check_text(text, what)
        if not _SEMVER.match(self.design_version):
            raise RecordError(f"design version {self.design_version!r} is not a semver string (D3)")
        hashes = (
            ("design source hash", self.design_source_hash),
            ("test case hash", self.test_case_hash),
            ("repository commit", self.repository_commit),
        )
        for what, value in hashes:
            if not _HASH.match(value):
                raise RecordError(f"{what} {value!r} is neither a 40-hex git hash nor 'unset'")
        if self.seed < 0 or self.seed > _UINT64_MAX:
            raise RecordError(f"seed {self.seed} is outside the unsigned 64-bit range (KER-012)")
        if self.controller_mode not in CONTROLLER_MODES:
            raise RecordError(f"controller mode {self.controller_mode!r} is not one of {CONTROLLER_MODES}")
        check_rate(self.physics_rate, "physics rate")
        check_rate(self.control_rate, "control rate")
        check_text(self.tool_version, "tool version")


@dataclass(frozen=True)
class RunFault:
    """The run error report (KER-015): failing party, simulated time of the failing step, cause."""

    party: str
    time: int
    cause: str


@dataclass(frozen=True)
class RunEnd:
    """How the run ended (REC-025), mirroring the kernel's `RunReport`."""

    outcome: str
    end_time: int
    ticks: int
    fault: RunFault | None = None

    def __post_init__(self) -> None:
        if self.outcome not in (END_CONDITION_MET, RUN_ERROR):
            raise RecordError(f"run outcome {self.outcome!r} is not {END_CONDITION_MET!r} or {RUN_ERROR!r}")
        if (self.outcome == RUN_ERROR) != (self.fault is not None):
            raise RecordError(f"outcome {self.outcome!r} must carry a fault exactly when it is {RUN_ERROR!r}")
        if self.fault is not None:
            check_text(self.fault.party, "fault party")
            check_text(self.fault.cause, "fault cause")


# ===== Repository and tool identity =========================================
def repository_commit(root: Path) -> str:
    """The repository's HEAD commit, or `unset` when there is no git or no repository."""
    try:
        result = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True, check=False
        )
    except FileNotFoundError:
        return UNSET
    commit = result.stdout.strip()
    if result.returncode != 0 or not re.fullmatch(r"[0-9a-f]{40}", commit):
        return UNSET
    return commit


def tool_version() -> str:
    try:
        return metadata.version("mechsim")
    except metadata.PackageNotFoundError:
        return UNSET


# ===== HDF5 helpers =========================================================
def _rate(rate: Fraction) -> NDArray[np.int64]:
    return np.array([rate.numerator, rate.denominator], dtype=np.int64)


def _fixed(texts: list[str]) -> NDArray[np.bytes_]:
    encoded = [text.encode("utf-8") for text in texts]
    width = max([1, *(len(item) for item in encoded)])
    return np.array(encoded, dtype=f"S{width}").reshape(len(encoded))


def _dataset(h5: h5py.File, path: str, values: NDArray[Any]) -> h5py.Dataset:
    chunks = (CHUNK_ROWS, *(max(1, size) for size in values.shape[1:]))
    dataset = h5.create_dataset(
        path,
        shape=values.shape,
        dtype=values.dtype,
        chunks=chunks,
        maxshape=(None,) * values.ndim,
        track_times=False,
    )
    if values.size > 0:
        dataset[...] = values
    return dataset


def _events_table(recording: Recording) -> NDArray[Any]:
    sources = _fixed([event.source for _, event in recording.events])
    kinds = _fixed([event.kind for _, event in recording.events])
    details = _fixed([event.detail for _, event in recording.events])
    dtype = np.dtype(
        [
            ("row", "<i8"),
            ("tick", "<i8"),
            ("seconds", "<f8"),
            ("source", sources.dtype),
            ("kind", kinds.dtype),
            ("detail", details.dtype),
        ]
    )
    table = np.zeros(len(recording.events), dtype=dtype)
    ticks = np.array([event.time for _, event in recording.events], dtype=np.int64)
    table["row"] = [row for row, _ in recording.events]
    table["tick"] = ticks
    table["seconds"] = seconds(ticks, recording.base_rate)
    table["source"] = sources
    table["kind"] = kinds
    table["detail"] = details
    return table


META_PREFIX = "meta_"


# ===== Writer ===============================================================
def write_archive(
    path: Path,
    identity: RunIdentity,
    end: RunEnd,
    recording: Recording,
    extra: Mapping[str, str] | None = None,
    columns: Mapping[str, Columns] | None = None,
) -> None:
    """Write one run archive; an existing file at `path` is replaced. `extra` lands as root attrs `meta_<key>` in key
    order: run metadata a caller keeps beside the identity (run id, end reason, bridge approximations). `columns`
    labels index-column series (REC-018) over any labels the recording carries."""
    series_map = label_series(recording.series, columns or {})
    with h5py.File(path, "w", libver=("v110", "v110"), track_order=False) as h5:
        attrs = h5.attrs
        for key in sorted(extra or {}):
            value = (extra or {})[key]
            check_text(value, f"metadata {key}")
            attrs[f"{META_PREFIX}{key}"] = value
        attrs["format"] = FORMAT
        attrs["format_version"] = np.int64(FORMAT_VERSION)
        attrs["design_name"] = identity.design_name
        attrs["design_version"] = identity.design_version
        attrs["design_source_hash"] = identity.design_source_hash
        attrs["test_case_id"] = identity.test_case_id
        attrs["test_case_hash"] = identity.test_case_hash
        attrs["seed"] = np.uint64(identity.seed)
        attrs["controller_mode"] = identity.controller_mode
        attrs["base_rate"] = _rate(recording.base_rate)
        attrs["physics_rate"] = _rate(identity.physics_rate)
        attrs["control_rate"] = _rate(identity.control_rate)
        attrs["repository_commit"] = identity.repository_commit
        attrs["tool_version"] = identity.tool_version
        attrs["end_outcome"] = end.outcome
        attrs["end_time"] = np.int64(end.end_time)
        attrs["end_ticks"] = np.int64(end.ticks)
        if end.fault is not None:
            attrs["fault_party"] = end.fault.party
            attrs["fault_time"] = np.int64(end.fault.time)
            attrs["fault_cause"] = end.fault.cause
        attrs["series"] = np.array(list(recording.series), dtype=h5py.string_dtype())
        _dataset(h5, _DESIGN_FILE, np.frombuffer(identity.design_file, dtype=np.uint8))
        _dataset(h5, _TEST_CASE_FILE, np.frombuffer(identity.test_case_file, dtype=np.uint8))
        for name, series in series_map.items():
            dataset = _dataset(h5, name, series.values)
            dataset.attrs["unit"] = series.unit
            dataset.attrs["axis"] = series.axis
            if name in (JOINT_FORCE, CONSTRAINT_FORCE):
                dataset.attrs["counts"] = np.array(series.counts, dtype=np.int64)
            if name in COLUMN_SERIES:
                dataset.attrs["names"] = np.array(list(series.names), dtype=h5py.string_dtype())
                dataset.attrs["units"] = np.array(list(series.units), dtype=h5py.string_dtype())
        for group in NAMED_GROUPS:
            h5.require_group(group).attrs["names"] = np.array(recording.named[group], dtype=h5py.string_dtype())
        coupling = h5.require_group(COUPLING)
        coupling.attrs["names"] = np.array([name for name, _ in recording.coupling], dtype=h5py.string_dtype())
        coupling.attrs["kinds"] = np.array([kind for _, kind in recording.coupling], dtype=h5py.string_dtype())
        _dataset(h5, _STATE_NAMES, _fixed(list(recording.controller_states)))
        _dataset(h5, _EVENTS, _events_table(recording))


# ===== Value conversions ====================================================
def _text(value: object) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return str(value)


def _fraction(value: NDArray[Any]) -> Fraction:
    return Fraction(int(value[0]), int(value[1]))


def _vec3(values: list[float]) -> Vec3:
    return (values[0], values[1], values[2])


def _quat(values: list[float]) -> Quat:
    return (values[0], values[1], values[2], values[3])


def _split(values: list[float], counts: tuple[int, ...]) -> list[tuple[float, ...]]:
    parts: list[tuple[float, ...]] = []
    start = 0
    for count in counts:
        parts.append(tuple(values[start : start + count]))
        start += count
    return parts


# ===== Reader ===============================================================
class RunArchive:
    """Reads a run archive: identity, end, every series with its unit, events, and replay in tick order (REC-024)."""

    def __init__(self, path: Path) -> None:
        self._file = h5py.File(path, "r")
        attrs = self._file.attrs
        if _text(attrs.get("format", "")) != FORMAT:
            self._file.close()
            raise RecordError(f"{path} is not a {FORMAT}")
        self.format_version = int(attrs.get("format_version", 0))
        if self.format_version not in READABLE_VERSIONS:
            self._file.close()
            raise RecordError(f"{path}: format version {self.format_version}, readable {list(READABLE_VERSIONS)}")
        fault: RunFault | None = None
        if "fault_party" in attrs:
            fault = RunFault(_text(attrs["fault_party"]), int(attrs["fault_time"]), _text(attrs["fault_cause"]))
        self.identity = RunIdentity(
            design_name=_text(attrs["design_name"]),
            design_version=_text(attrs["design_version"]),
            design_source_hash=_text(attrs["design_source_hash"]),
            test_case_id=_text(attrs["test_case_id"]),
            test_case_hash=_text(attrs["test_case_hash"]),
            seed=int(attrs["seed"]),
            controller_mode=_text(attrs["controller_mode"]),
            physics_rate=_fraction(attrs["physics_rate"]),
            control_rate=_fraction(attrs["control_rate"]),
            repository_commit=_text(attrs["repository_commit"]),
            tool_version=_text(attrs["tool_version"]),
            design_file=self._file[_DESIGN_FILE][...].tobytes(),
            test_case_file=self._file[_TEST_CASE_FILE][...].tobytes(),
        )
        self.end = RunEnd(_text(attrs["end_outcome"]), int(attrs["end_time"]), int(attrs["end_ticks"]), fault)
        self.base_rate = _fraction(attrs["base_rate"])
        self._names = tuple(_text(name) for name in attrs["series"])
        self.extra = {
            _text(key)[len(META_PREFIX) :]: _text(attrs[key]) for key in sorted(attrs) if key.startswith(META_PREFIX)
        }

    def __enter__(self) -> RunArchive:
        return self

    def __exit__(
        self, kind: type[BaseException] | None, error: BaseException | None, trace: TracebackType | None
    ) -> None:
        self.close()

    def close(self) -> None:
        self._file.close()

    def series_names(self) -> tuple[str, ...]:
        return self._names

    def series(self, name: str) -> Series:
        if name not in self._names:
            raise RecordError(f"no series {name!r} in the archive")
        dataset = self._file[name]
        counts = tuple(int(count) for count in dataset.attrs.get("counts", np.zeros(0, dtype=np.int64)))
        names = tuple(_text(item) for item in dataset.attrs.get("names", ()))
        units = tuple(_text(item) for item in dataset.attrs.get("units", ()))
        return Series(dataset[...], _text(dataset.attrs["unit"]), _text(dataset.attrs["axis"]), counts, names, units)

    def named(self, group: str) -> tuple[str, ...]:
        return tuple(_text(name) for name in self._file[group].attrs["names"])

    def controller_states(self) -> tuple[str, ...]:
        return tuple(_text(name) for name in self._file[_STATE_NAMES][...])

    def events_table(self) -> NDArray[Any]:
        """The events compound dataset: row, tick, seconds, source, kind, detail, in recording order."""
        table: NDArray[Any] = self._file[_EVENTS][...]
        return table

    def events(self) -> tuple[tuple[int, RecordedEvent], ...]:
        table = self.events_table()
        return tuple(
            (
                int(item["row"]),
                RecordedEvent(int(item["tick"]), _text(item["source"]), _text(item["kind"]), _text(item["detail"])),
            )
            for item in table
        )

    def coupling(self) -> tuple[tuple[str, str], ...]:
        """(name, kind) of every recorded coupling variable, name order; empty for an archive without the group."""
        if COUPLING not in self._file:
            return ()
        attrs = self._file[COUPLING].attrs
        return tuple((_text(name), _text(kind)) for name, kind in zip(attrs["names"], attrs["kinds"], strict=True))

    def recording(self) -> Recording:
        return Recording(
            self.base_rate,
            {name: self.series(name) for name in self._names},
            {group: self.named(group) for group in NAMED_GROUPS},
            self.controller_states(),
            self.events(),
            self.coupling(),
        )

    def records(self) -> Iterator[TickRecord]:
        """Replay: the recorded rows in tick order, rebuilt from the archive alone (REC-024)."""
        recording = self.recording()
        values = {name: series.values.tolist() for name, series in recording.series.items()}
        joint_counts = recording.series[JOINT_FORCE].counts
        constraint_counts = recording.series[CONSTRAINT_FORCE].counts
        contact_rows = recording.series[CONTACT_ROW].values
        event_rows = np.array([row for row, _ in recording.events], dtype=np.int64)
        states = recording.controller_states
        for index, tick in enumerate(values["time/tick"]):
            yield TickRecord(
                time=tick,
                physics=self._physics(values, index, joint_counts, constraint_counts, contact_rows),
                pin_signals=self._samples(recording, values, NAMED_GROUPS[0], index),
                rails=self._samples(recording, values, NAMED_GROUPS[1], index),
                actuator_efforts=self._samples(recording, values, NAMED_GROUPS[2], index),
                controller_state=states[values[CONTROLLER_STATE][index]],
                events=tuple(
                    recording.events[position][1]
                    for position in range(
                        int(np.searchsorted(event_rows, index, side="left")),
                        int(np.searchsorted(event_rows, index, side="right")),
                    )
                ),
                coupling=self._coupling(recording, values, index),
            )

    @staticmethod
    def _coupling(recording: Recording, values: dict[str, Any], index: int) -> tuple[CouplingSample, ...]:
        samples: list[CouplingSample] = []
        for name, kind in recording.coupling:
            digest = _text(values[f"{COUPLING_DIGEST}/{name}"][index])
            value = float(values[f"{COUPLING_VALUE}/{name}"][index])
            sample_kind = kind
            if kind == "payload" and digest == "":
                sample_kind = "none"
            if kind != "payload" and math.isnan(value):
                sample_kind = "none"
            samples.append(CouplingSample(name, sample_kind, value, digest))
        return tuple(samples)

    @staticmethod
    def _samples(recording: Recording, values: dict[str, Any], group: str, index: int) -> tuple[SeriesSample, ...]:
        return tuple(
            SeriesSample(name, recording.series[f"{group}/{name}"].unit, values[f"{group}/{name}"][index])
            for name in recording.named[group]
        )

    @staticmethod
    def _physics(
        values: dict[str, Any],
        index: int,
        joint_counts: tuple[int, ...],
        constraint_counts: tuple[int, ...],
        contact_rows: NDArray[Any],
    ) -> PhysicsOutputs:
        def at(name: str) -> Any:
            if name not in values:
                return []
            return values[name][index]

        bodies = tuple(
            BodyState(_vec3(position), _quat(orientation), _vec3(angular), _vec3(linear))
            for position, orientation, angular, linear in zip(
                at("physics/bodies/position"),
                at("physics/bodies/orientation"),
                at("physics/bodies/angular_velocity"),
                at("physics/bodies/linear_velocity"),
                strict=True,
            )
        )
        joints = tuple(
            JointForce(force, Wrench(_vec3(moment), _vec3(reaction)))
            for force, moment, reaction in zip(
                _split(at(JOINT_FORCE), joint_counts),
                at("physics/joints/reaction_moment"),
                at("physics/joints/reaction_force"),
                strict=True,
            )
        )
        constraints = tuple(
            ConstraintForce(force, translational, rotational)
            for force, translational, rotational in zip(
                _split(at(CONSTRAINT_FORCE), constraint_counts),
                at("physics/constraints/translational_error"),
                at("physics/constraints/rotational_error"),
                strict=True,
            )
        )
        first = int(np.searchsorted(contact_rows, index, side="left"))
        last = int(np.searchsorted(contact_rows, index, side="right"))
        contacts = tuple(
            ContactForce(
                values["physics/contacts/body_a"][position],
                values["physics/contacts/body_b"][position],
                _vec3(values["physics/contacts/point"][position]),
                _vec3(values["physics/contacts/force"][position]),
            )
            for position in range(first, last)
        )
        members = tuple(
            MemberEndLoads(Wrench(_vec3(sm), _vec3(sf)), Wrench(_vec3(em), _vec3(ef)))
            for sm, sf, em, ef in zip(
                at("physics/members/start_moment"),
                at("physics/members/start_force"),
                at("physics/members/end_moment"),
                at("physics/members/end_force"),
                strict=True,
            )
        )
        tensions = at("physics/ropes/tension")
        maxima = at(ROPE_MAX_TENSION)
        if len(maxima) != len(tensions):
            maxima = tensions
        ropes = tuple(
            RopeTension(tension, slack, maximum)
            for tension, slack, maximum in zip(tensions, at("physics/ropes/slack"), maxima, strict=True)
        )
        transmissions = tuple(
            TransmissionState(slip, slipping, load, force, stretch, flow)
            for slip, slipping, load, force, stretch, flow in zip(
                at("physics/transmissions/slip"),
                at("physics/transmissions/slipping"),
                at("physics/transmissions/shaft_load_torque"),
                at("physics/transmissions/force"),
                at("physics/transmissions/stretch"),
                at("physics/transmissions/power_flow"),
                strict=True,
            )
        )
        drive_torques = tuple(
            JointDriveTorque(joint, torque)
            for joint, (torque, driven) in enumerate(zip(at(DRIVE_TORQUE), at(DRIVE_DRIVEN), strict=True))
            if driven
        )
        payloads = tuple(
            PayloadState(body, carrier, attached, _vec3(force))
            for body, carrier, attached, force in zip(
                at("physics/payloads/body"),
                at("physics/payloads/carrier"),
                at("physics/payloads/attached"),
                at("physics/payloads/force"),
                strict=True,
            )
        )
        return PhysicsOutputs(
            at("physics/steps"),
            bodies,
            joints,
            constraints,
            contacts,
            members,
            ropes,
            transmissions,
            tuple(at(JOINT_POSITIONS)),
            drive_torques,
            payloads,
        )
