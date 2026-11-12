"""Recorder, run archive and replay (REC spec, unit tier), over synthetic tick records.

Every comparison is exact: the archive stores the float64 values it was handed and replay returns them unchanged,
so no tolerance applies (REC tolerances are n/a).
"""

from __future__ import annotations

import dataclasses
import time
from fractions import Fraction
from pathlib import Path

import h5py
import numpy as np
import pytest

from mechsim.record import (
    Recorder,
    RecordError,
    RunArchive,
    RunEnd,
    RunFault,
    TickRecord,
    archives_identical,
    repository_commit,
    tool_version,
)
from mechsim.record.archive import CHUNK_ROWS, FORMAT_VERSION, write_archive
from mechsim.record.layout import COLUMN_SERIES, JOINT_POSITIONS, Columns
from mechsim.record.tick import (
    WORLD_BODY,
    BodyState,
    JointDriveTorque,
    PayloadState,
    PhysicsOutputs,
    RecordedEvent,
    SeriesSample,
)
from tests.unit.test_record_support import (
    BASE_RATE,
    CONSTRAINT_ROWS,
    EFFORTS,
    JOINT_COORDINATES,
    JOINT_DOFS,
    PINS,
    RAILS,
    STATES,
    finished,
    identity,
    synthetic_rows,
)

TICKS = 120


# ===== Fixtures and helpers =================================================
def _record(rows: list[TickRecord], path: Path, base_rate: Fraction = BASE_RATE, end: RunEnd | None = None) -> Path:
    recorder = Recorder(base_rate)
    for row in rows:
        recorder.write_row(row)
    if end is None:
        end = finished(len(rows))
    recorder.flush(path, identity(), end)
    return path


@pytest.fixture(scope="module")
def rows() -> list[TickRecord]:
    return synthetic_rows(7, TICKS)


@pytest.fixture(scope="module")
def archive_path(rows: list[TickRecord], tmp_path_factory: pytest.TempPathFactory) -> Path:
    return _record(rows, tmp_path_factory.mktemp("rec") / "run.h5")


@pytest.fixture
def archive(archive_path: Path) -> RunArchive:
    return RunArchive(archive_path)


# ===== Body, joint and contact channels =====================================
@pytest.mark.req("REC-002")
def test_rec_002_body_pose_and_velocity_every_tick(rows: list[TickRecord], archive: RunArchive) -> None:
    """The recorder shall record every body's pose and velocity on every physics tick."""
    with archive:
        expected = {
            "physics/bodies/position": ("m", [[b.position for b in r.physics.bodies] for r in rows]),
            "physics/bodies/orientation": ("1", [[b.orientation for b in r.physics.bodies] for r in rows]),
            "physics/bodies/angular_velocity": (
                "rad/s",
                [[b.angular_velocity for b in r.physics.bodies] for r in rows],
            ),
            "physics/bodies/linear_velocity": ("m/s", [[b.linear_velocity for b in r.physics.bodies] for r in rows]),
        }
        for name, (unit, values) in expected.items():
            series = archive.series(name)
            assert series.unit == unit
            assert series.values.shape[0] == TICKS
            np.testing.assert_array_equal(series.values, np.array(values))
        assert archive.series("physics/steps").values.tolist() == [r.physics.steps for r in rows]


@pytest.mark.req("REC-003")
def test_rec_003_joint_and_contact_forces_every_tick(rows: list[TickRecord], archive: RunArchive) -> None:
    """The recorder shall record every joint force and contact force on every physics tick."""
    with archive:
        force = archive.series("physics/joints/generalized_force")
        assert force.counts == JOINT_DOFS
        assert force.unit == "N or N m"
        flat = [[v for j in r.physics.joints for v in j.generalized_force] for r in rows]
        np.testing.assert_array_equal(force.values, np.array(flat))
        reaction = archive.series("physics/joints/reaction_force").values
        np.testing.assert_array_equal(reaction, np.array([[j.reaction.force for j in r.physics.joints] for r in rows]))
        moment = archive.series("physics/joints/reaction_moment").values
        np.testing.assert_array_equal(moment, np.array([[j.reaction.moment for j in r.physics.joints] for r in rows]))
        constraint = archive.series("physics/constraints/force")
        assert constraint.counts == CONSTRAINT_ROWS
        contacts = [(i, r.time, c) for i, r in enumerate(rows) for c in r.physics.contacts]
        assert len(contacts) > 0
        assert archive.series("physics/contacts/row").values.tolist() == [i for i, _, _ in contacts]
        assert archive.series("physics/contacts/tick").values.tolist() == [t for _, t, _ in contacts]
        assert archive.series("physics/contacts/force").values.tolist() == [list(c.force) for _, _, c in contacts]
        assert archive.series("physics/contacts/point").values.tolist() == [list(c.point) for _, _, c in contacts]
        assert archive.series("physics/contacts/body_a").values.tolist() == [c.body_a for _, _, c in contacts]
        assert archive.series("physics/contacts/force").axis == "contact"


@pytest.mark.req("REC-004")
def test_rec_004_member_end_loads_every_tick(rows: list[TickRecord], archive: RunArchive) -> None:
    """The recorder shall record every structural member's end-loads on every physics tick."""
    with archive:
        for name, unit, pick in (
            ("start_moment", "N m", lambda m: m.start.moment),
            ("start_force", "N", lambda m: m.start.force),
            ("end_moment", "N m", lambda m: m.end.moment),
            ("end_force", "N", lambda m: m.end.force),
        ):
            series = archive.series(f"physics/members/{name}")
            assert series.unit == unit
            np.testing.assert_array_equal(series.values, np.array([[pick(m) for m in r.physics.members] for r in rows]))


# ===== Named signal checks ==================================================
def _named_check(archive: RunArchive, rows: list[TickRecord], group: str, spec: tuple[tuple[str, str], ...]) -> None:
    attribute = {
        "signals/pins": "pin_signals",
        "signals/rails": "rails",
        "signals/actuator_efforts": "actuator_efforts",
    }
    assert archive.named(group) == tuple(name for name, _ in spec)
    for position, (name, unit) in enumerate(spec):
        series = archive.series(f"{group}/{name}")
        assert series.unit == unit
        assert series.values.tolist() == [getattr(r, attribute[group])[position].value for r in rows]


@pytest.mark.req("REC-005")
def test_rec_005_every_signal_on_every_pin(rows: list[TickRecord], archive: RunArchive) -> None:
    """The recorder shall record every signal on every pin, in both directions between the signal layer and
    the controller. The row carries both directions as one named set (tick_record.hpp); every name is kept."""
    with archive:
        _named_check(archive, rows, "signals/pins", PINS)


@pytest.mark.req("REC-006")
def test_rec_006_every_rail(rows: list[TickRecord], archive: RunArchive) -> None:
    """The recorder shall record the state of every power rail."""
    with archive:
        _named_check(archive, rows, "signals/rails", RAILS)


# ===== Actuators, joints and payloads =======================================
@pytest.mark.req("REC-027")
def test_rec_027_actuator_efforts_every_tick(rows: list[TickRecord], archive: RunArchive) -> None:
    """The recorder shall record the actuator torques and forces the signal layer applies to physics on
    every tick."""
    with archive:
        _named_check(archive, rows, "signals/actuator_efforts", EFFORTS)


@pytest.mark.req("REC-027")
def test_rec_027_joint_positions_and_drive_torques_every_tick(rows: list[TickRecord], archive: RunArchive) -> None:
    """Joint coordinates and the torque or force each joint drive applied, one column per joint, every tick; an
    undriven joint reads 0 with `driven` false."""
    with archive:
        positions = archive.series("physics/joint_positions")
        torques = archive.series("physics/joint_drive_torques/torque")
        driven = archive.series("physics/joint_drive_torques/driven")
    assert (positions.unit, torques.unit, driven.unit) == ("m or rad", "N or N m", "1")
    assert positions.axis == torques.axis == driven.axis == "tick"
    assert positions.values.shape == torques.values.shape == driven.values.shape == (TICKS, JOINT_COORDINATES)
    for index, row in enumerate(rows):
        assert tuple(positions.values[index]) == row.physics.joint_positions
        expected_torque = [0.0] * JOINT_COORDINATES
        expected_driven = [False] * JOINT_COORDINATES
        for drive in row.physics.joint_drive_torques:
            expected_torque[drive.joint] = drive.torque
            expected_driven[drive.joint] = True
        assert list(torques.values[index]) == expected_torque
        assert list(driven.values[index]) == expected_driven
    assert driven.values[:, 0].all()
    assert driven.values[::2, 1].all()
    assert not driven.values[1::2, 1].any()


@pytest.mark.req("REC-027")
def test_rec_027_payload_attachment_and_force_every_tick(rows: list[TickRecord], archive: RunArchive) -> None:
    """Each payload's body, carrier (-1 once released), attachment and attachment force (N), every tick."""
    with archive:
        body = archive.series("physics/payloads/body")
        carrier = archive.series("physics/payloads/carrier")
        attached = archive.series("physics/payloads/attached")
        force = archive.series("physics/payloads/force")
    assert force.unit == "N"
    assert force.values.shape == (TICKS, 1, 3)
    for index, row in enumerate(rows):
        (payload,) = row.physics.payloads
        assert body.values[index, 0] == payload.body
        assert carrier.values[index, 0] == payload.carrier
        assert bool(attached.values[index, 0]) is payload.attached
        assert tuple(force.values[index, 0]) == payload.force
    assert carrier.values[-1, 0] == WORLD_BODY


@pytest.mark.req("REC-027")
@pytest.mark.parametrize(
    ("physics", "message"),
    [
        (PhysicsOutputs(joint_positions=(0.0,), joint_drive_torques=(JointDriveTorque(1, 2.0),)), "outside the 1"),
        (PhysicsOutputs(joint_drive_torques=(JointDriveTorque(0, 2.0),)), "outside the 0"),
        (PhysicsOutputs(joint_positions=(0.0,), joint_drive_torques=(JointDriveTorque(-1, 2.0),)), "outside"),
        (
            PhysicsOutputs(
                joint_positions=(0.0, 0.0),
                joint_drive_torques=(JointDriveTorque(1, 2.0), JointDriveTorque(1, 3.0)),
            ),
            "two drive torques on joint 1",
        ),
    ],
)
def test_rec_027_rejects_drive_torques_off_the_joints(physics: PhysicsOutputs, message: str) -> None:
    with pytest.raises(RecordError, match=message):
        Recorder(BASE_RATE).write_row(TickRecord(time=0, physics=physics))


@pytest.mark.req("REC-027")
def test_rec_027_drives_out_of_joint_order_replay_in_joint_order(tmp_path: Path) -> None:
    drives = (JointDriveTorque(1, 2.0), JointDriveTorque(0, -1.0))
    row = TickRecord(time=0, physics=PhysicsOutputs(joint_positions=(0.5, 0.25), joint_drive_torques=drives))
    with RunArchive(_record([row], tmp_path / "drives.h5")) as archive:
        (replayed,) = archive.records()
    assert replayed.physics.joint_drive_torques == (JointDriveTorque(0, -1.0), JointDriveTorque(1, 2.0))
    assert replayed.physics.joint_positions == (0.5, 0.25)


JOINT_LABELS = Columns(("trolley", "hoist"), ("m", "rad"))
TORQUE_LABELS = Columns(("trolley", "hoist"), ("N", "N m"))
SLIP_LABELS = Columns(("belt",), ("rad",))
PAYLOAD_LABELS = Columns(("load",))


# ===== Index columns and writer labels ======================================
@pytest.mark.req("REC-018")
@pytest.mark.req("REC-027")
def test_rec_018_index_columns_carry_names_and_units(rows: list[TickRecord], tmp_path: Path) -> None:
    """Driver-supplied labels land as per-series attrs `names` and `units`; unlabelled index series keep the
    conditional unit with empty `names` and `units`."""
    columns = {
        "physics/joint_positions": JOINT_LABELS,
        "physics/joint_drive_torques/torque": TORQUE_LABELS,
        "physics/transmissions/slip": SLIP_LABELS,
        "physics/payloads/force": PAYLOAD_LABELS,
    }
    recorder = Recorder(BASE_RATE, columns)
    for row in rows:
        recorder.write_row(row)
    path = tmp_path / "labelled.h5"
    recorder.flush(path, identity(), finished(len(rows)))
    with RunArchive(path) as archive:
        for name, labels in columns.items():
            series = archive.series(name)
            assert (series.names, series.units) == (labels.names, labels.units)
        assert archive.series("physics/joint_positions").unit == "m or rad"
        unlabelled = archive.series("physics/joint_drive_torques/driven")
        assert (unlabelled.names, unlabelled.units) == ((), ())
        assert archive.series("physics/bodies/position").names == ()
        assert list(archive.records()) == rows
    with h5py.File(path, "r") as h5:
        for name in COLUMN_SERIES:
            assert "names" in h5[name].attrs
            assert "units" in h5[name].attrs
        assert "names" not in h5["physics/bodies/position"].attrs


@pytest.mark.req("REC-018")
def test_rec_018_archive_writer_labels_override_recording(rows: list[TickRecord], tmp_path: Path) -> None:
    recorder = Recorder(BASE_RATE, {"physics/joint_positions": Columns(("a", "b"))})
    for row in rows[:3]:
        recorder.write_row(row)
    path = tmp_path / "override.h5"
    write_archive(
        path, identity(), finished(3), recorder.recording(), columns={"physics/joint_positions": JOINT_LABELS}
    )
    with RunArchive(path) as archive:
        series = archive.series("physics/joint_positions")
    assert (series.names, series.units) == (JOINT_LABELS.names, JOINT_LABELS.units)
    with pytest.raises(RecordError, match="1 column names for 2 columns"):
        write_archive(path, identity(), finished(3), recorder.recording(), columns={JOINT_POSITIONS: Columns(("a",))})


@pytest.mark.req("REC-018")
@pytest.mark.parametrize(
    ("columns", "message"),
    [
        ({"physics/bodies/position": Columns(("a",))}, "not an index-column series"),
        ({JOINT_POSITIONS: Columns(("a", "a"))}, "duplicate column names"),
        ({JOINT_POSITIONS: Columns(("a", "b"), ("m",))}, "1 units for 2 columns"),
        ({JOINT_POSITIONS: Columns(("a", "b"), ("m", ""))}, "empty column unit"),
        ({JOINT_POSITIONS: Columns(("a", "b/c"))}, "contains '/'"),
        ({JOINT_POSITIONS: Columns(("a", ""))}, "empty series name"),
        ({JOINT_POSITIONS: Columns(("a", "b"), ("m", "N\0"))}, "NUL"),
    ],
)
def test_rec_018_rejects_bad_column_labels(columns: dict[str, Columns], message: str) -> None:
    with pytest.raises(RecordError, match=message):
        Recorder(BASE_RATE, columns)


@pytest.mark.req("REC-018")
@pytest.mark.parametrize(
    ("path", "labels", "width"),
    [
        (JOINT_POSITIONS, Columns(("a",)), JOINT_COORDINATES),
        ("physics/joint_drive_torques/driven", Columns(("a", "b", "c")), JOINT_COORDINATES),
        ("physics/transmissions/force", Columns(("a", "b")), 1),
        ("physics/payloads/attached", Columns(()), 1),
    ],
)
def test_rec_018_column_labels_must_match_the_first_row(path: str, labels: Columns, width: int) -> None:
    recorder = Recorder(BASE_RATE, {path: labels})
    with pytest.raises(RecordError, match=f"{path} has {width} columns, {len(labels.names)} names given"):
        recorder.write_row(synthetic_rows(3, 1)[0])


# ===== Archive format version ===============================================
@pytest.mark.req("REC-012")
def test_rec_012_archive_format_version_2_and_version_1_still_reads(rows: list[TickRecord], tmp_path: Path) -> None:
    """Version 2 adds the joint, drive and payload series and column labels; a version 1 archive lacks them and its
    rows replay with those fields empty. Other versions are refused."""
    assert FORMAT_VERSION == 2
    path = _record(rows[:5], tmp_path / "v1.h5")
    with h5py.File(path, "r+") as h5:
        assert int(h5.attrs["format_version"]) == 2
        added = ("physics/joint_positions", "physics/joint_drive_torques", "physics/payloads")
        kept = [str(name) for name in h5.attrs["series"] if not str(name).startswith(added)]
        for group in added:
            del h5[group]
        for name in COLUMN_SERIES:
            if name in h5:
                del h5[name].attrs["names"]
                del h5[name].attrs["units"]
        h5.attrs["series"] = np.array(kept, dtype=h5py.string_dtype())
        h5.attrs["format_version"] = np.int64(1)
    with RunArchive(path) as archive:
        assert archive.format_version == 1
        assert archive.series("physics/transmissions/slip").names == ()
        replayed = list(archive.records())
    expected = [
        dataclasses.replace(
            row, physics=dataclasses.replace(row.physics, joint_positions=(), joint_drive_torques=(), payloads=())
        )
        for row in rows[:5]
    ]
    assert replayed == expected
    for version in (0, 3):
        with h5py.File(path, "r+") as h5:
            h5.attrs["format_version"] = np.int64(version)
        with pytest.raises(RecordError, match=f"format version {version}"):
            RunArchive(path)


# ===== Controller state and fault log =======================================
@pytest.mark.req("REC-007")
def test_rec_007_controller_state_and_fault_log(rows: list[TickRecord], archive: RunArchive) -> None:
    """The recorder shall record the controller states and the controller fault log the controller runtime
    reports."""
    with archive:
        states = archive.controller_states()
        assert set(states) == set(STATES)
        index = archive.series("controller/state").values.tolist()
        assert [states[i] for i in index] == [r.controller_state for r in rows]
        faults = [e for _, e in archive.events() if e.kind == "controller_fault"]
        assert faults == [e for r in rows for e in r.events if e.kind == "controller_fault"]
        assert len(faults) > 0


def _events_of(rows: list[TickRecord], kind: str) -> list[RecordedEvent]:
    return [e for r in rows for e in r.events if e.kind == kind]


@pytest.mark.req("REC-008")
def test_rec_008_faults_and_safety_events_with_time(rows: list[TickRecord], archive: RunArchive) -> None:
    """The recorder shall record every injected fault and every safety event with its simulated time."""
    with archive:
        table = archive.events_table()
        for kind in ("injected_fault", "safety_event"):
            expected = _events_of(rows, kind)
            assert len(expected) > 0
            found = table[table["kind"] == kind.encode()]
            assert found["tick"].tolist() == [e.time for e in expected]
            assert found["seconds"].tolist() == [e.time / 1000 for e in expected]


@pytest.mark.req("REC-009")
def test_rec_009_operator_messages_verbatim(rows: list[TickRecord], archive: RunArchive) -> None:
    """The recorder shall record every operator command and status message with its simulated time, using
    the message labels verbatim (UTF-8 kept byte for byte)."""
    with archive:
        recorded = [e for _, e in archive.events() if e.source == "operator"]
        expected = [e for r in rows for e in r.events if e.source == "operator"]
        assert recorded == expected
        assert "Not-Aus ausgelöst: Hubwerk gestoppt" in [e.detail for e in recorded]


@pytest.mark.req("REC-009")
def test_rec_009_events_keep_recording_order(rows: list[TickRecord], archive: RunArchive) -> None:
    """Events come back in the order recorded, tick by tick, and within a tick in emission order."""
    with archive:
        assert [e for _, e in archive.events()] == [e for r in rows for e in r.events]
        assert [row for row, _ in archive.events()] == [i for i, r in enumerate(rows) for _ in r.events]


@pytest.mark.req("REC-011")
def test_rec_011_simulated_time_on_every_sample_and_event(rows: list[TickRecord], tmp_path: Path) -> None:
    """The recorder shall stamp every recorded sample and event with the simulated time.

    Base rate 3 kHz: seconds = tick / 3000 exactly as one float64 division of the integer tick count.
    """
    path = _record(rows, tmp_path / "run.h5", base_rate=Fraction(3000))
    with RunArchive(path) as archive:
        assert archive.series_names()[:2] == ("time/tick", "time/seconds")
        ticks = archive.series("time/tick").values
        assert ticks.tolist() == [r.time for r in rows]
        assert archive.series("time/seconds").values.tolist() == [t / 3000 for t in ticks.tolist()]
        contact_ticks = archive.series("physics/contacts/tick").values.tolist()
        assert archive.series("physics/contacts/seconds").values.tolist() == [t / 3000 for t in contact_ticks]
        table = archive.events_table()
        assert table["seconds"].tolist() == [t / 3000 for t in table["tick"].tolist()]
        for name in archive.series_names():
            assert archive.series(name).axis in ("tick", "contact")
            assert archive.series(name).values.shape[0] in (len(rows), len(contact_ticks))


@pytest.mark.req("REC-011")
@pytest.mark.parametrize("times", [(0, 0), (5, 4), (-1,)])
def test_rec_011_rows_must_advance_in_time(times: tuple[int, ...]) -> None:
    recorder = Recorder(BASE_RATE)
    for t in times[:-1]:
        recorder.write_row(TickRecord(time=t))
    with pytest.raises(RecordError, match="advance in simulated time"):
        recorder.write_row(TickRecord(time=times[-1]))


@pytest.mark.req("REC-012")
def test_rec_012_one_hdf5_file_per_run(archive_path: Path, tmp_path: Path) -> None:
    """The recorder shall write one HDF5 archive per run through h5py."""
    assert h5py.is_hdf5(archive_path)
    second = _record(synthetic_rows(8, 5), tmp_path / "other.h5")
    assert sorted(p.name for p in tmp_path.iterdir()) == ["other.h5"]
    with h5py.File(second, "r") as h5:
        assert h5.attrs["format"] == "mechsim-run-archive"
        assert h5["time/tick"].shape == (5,)


@pytest.mark.req("REC-013")
def test_rec_013_identity_attributes_present_and_typed(archive_path: Path) -> None:
    """The run archive shall carry the design file, the test case, the seed, the controller mode and the
    git hashes as attributes."""
    with h5py.File(archive_path, "r") as h5:
        attrs = h5.attrs
        for name in (
            "design_name",
            "design_version",
            "design_source_hash",
            "test_case_id",
            "test_case_hash",
            "controller_mode",
            "repository_commit",
            "tool_version",
            "end_outcome",
        ):
            assert isinstance(attrs[name], str), name
        assert attrs["seed"].dtype == np.uint64
        assert int(attrs["seed"]) == 42
        for name in ("base_rate", "physics_rate", "control_rate"):
            assert attrs[name].dtype == np.int64
            assert attrs[name].tolist() == [1000, 1]
        assert attrs["format_version"].dtype == np.int64
        assert h5["inputs/design_file"][...].tobytes() == identity().design_file
        assert h5["inputs/test_case_file"][...].tobytes() == identity().test_case_file
    with RunArchive(archive_path) as archive:
        assert archive.identity == identity()
        assert archive.base_rate == BASE_RATE


@pytest.mark.req("REC-013")
def test_rec_013_repository_commit_and_tool_version(tmp_path: Path) -> None:
    repository = Path(__file__).resolve().parents[2]
    commit = repository_commit(repository)
    assert commit == "unset" or (len(commit) == 40 and int(commit, 16) >= 0)
    assert repository_commit(tmp_path) == "unset"
    assert tool_version() != ""


@pytest.mark.req("REC-013")
@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("design_name", "", "design name is empty"),
        ("test_case_id", "", "test case id is empty"),
        ("test_case_hash", "abc", "test case hash"),
        ("repository_commit", "A" * 40, "repository commit"),
        ("seed", -1, "seed"),
        ("seed", 2**64, "seed"),
        ("controller_mode", "hybrid", "controller mode"),
        ("physics_rate", Fraction(0), "physics rate"),
        ("control_rate", Fraction(-1), "control rate"),
        ("tool_version", "a\0b", "NUL"),
    ],
)
def test_rec_013_identity_rejects_bad_values(field: str, value: object, message: str) -> None:
    with pytest.raises(RecordError, match=message):
        identity(**{field: value})


@pytest.mark.req("REC-013")
def test_rec_013_identity_seed_extremes_round_trip(tmp_path: Path) -> None:
    for seed in (0, 2**64 - 1):
        path = tmp_path / f"seed{seed}.h5"
        recorder = Recorder(BASE_RATE)
        recorder.flush(path, identity(seed=seed), finished(0))
        with RunArchive(path) as archive:
            assert archive.identity.seed == seed


@pytest.mark.req("REC-014")
def test_rec_014_design_version_and_blob_hash(archive_path: Path) -> None:
    """The run archive shall carry the design version as the semver string plus the design file's git blob
    hash."""
    with RunArchive(archive_path) as archive:
        assert archive.identity.design_version == "1.2.3"
        assert archive.identity.design_source_hash == "0123456789abcdef0123456789abcdef01234567"
    identity(design_version="2.0.0-rc.1+build.5", design_source_hash="unset")
    for bad in ("1.2", "v1.2.3", "1.2.3.4", ""):
        with pytest.raises(RecordError, match="semver"):
            identity(design_version=bad)


@pytest.mark.req("REC-018")
def test_rec_018_every_series_states_its_unit(archive_path: Path) -> None:
    """Every archive series and export shall hold SI values and state the unit of each quantity explicitly."""
    si = {"1", "s", "m", "rad", "rad/s", "m/s", "N", "N m", "N or N m", "m or rad", "V"}
    with h5py.File(archive_path, "r") as h5:
        names = [str(n) for n in h5.attrs["series"]]
        assert len(names) > 0
        for name in names:
            assert h5[name].attrs["unit"] in si, name


@pytest.mark.req("REC-019")
def test_rec_019_streaming_subscribers_receive_rows_in_order() -> None:
    """The recorder shall expose a streaming interface through which a consumer receives recorded ticks during
    a run: synchronously, after the row is stored, in subscription order."""
    rows = synthetic_rows(3, 12)
    recorder = Recorder(BASE_RATE)
    seen: list[tuple[str, int, int]] = []
    first = recorder.subscribe(lambda r: seen.append(("first", r.time, recorder.rows)))
    recorder.subscribe(lambda r: seen.append(("second", r.time, recorder.rows)))
    for row in rows[:6]:
        recorder.write_row(row)
    recorder.unsubscribe(first)
    for row in rows[6:]:
        recorder.write_row(row)
    expected = [(who, t, t + 1) for t in range(6) for who in ("first", "second")]
    expected += [("second", t, t + 1) for t in range(6, 12)]
    assert seen == expected
    late: list[int] = []
    recorder.subscribe(lambda r: late.append(r.time))
    assert late == []
    with pytest.raises(RecordError, match="no subscription"):
        recorder.unsubscribe(first)


@pytest.mark.req("REC-020")
def test_rec_020_consumer_cannot_change_recording(tmp_path: Path) -> None:
    """A streaming consumer shall not change the recording or the run it observes: rows are frozen, and an
    archive recorded with a consumer attached is byte-identical to one recorded without."""
    rows = synthetic_rows(5, 40)

    def meddle(row: TickRecord) -> None:
        with pytest.raises(dataclasses.FrozenInstanceError):
            row.time = 99  # type: ignore[misc]
        with pytest.raises(dataclasses.FrozenInstanceError):
            row.physics.bodies[0].position = (0.0, 0.0, 0.0)  # type: ignore[misc]
        with pytest.raises(TypeError):
            row.pin_signals[0] = SeriesSample("x", "V", 0.0)  # type: ignore[index]

    watched = Recorder(BASE_RATE)
    watched.subscribe(meddle)
    for row in rows:
        watched.write_row(row)
    watched.flush(tmp_path / "watched.h5", identity(), finished(40))
    plain = _record(rows, tmp_path / "plain.h5")
    assert archives_identical(tmp_path / "watched.h5", plain).identical
    assert rows == synthetic_rows(5, 40)


@pytest.mark.req("REC-023")
def test_rec_023_no_wall_clock_in_archive(tmp_path: Path) -> None:
    """The recorder shall write no wall-clock time or other nondeterministic value into the recorded series
    or attributes. HDF5 object timestamps have one-second resolution, so two writes 1.1 s apart differ in bytes
    if any timestamp is written."""
    rows = synthetic_rows(9, 30)
    first = _record(rows, tmp_path / "a.h5")
    time.sleep(1.1)
    second = _record(rows, tmp_path / "b.h5")
    assert first.read_bytes() == second.read_bytes()
    with h5py.File(first, "r") as h5:
        expected = {
            "format", "format_version", "design_name", "design_version", "design_source_hash", "test_case_id",
            "test_case_hash", "seed", "controller_mode", "base_rate", "physics_rate", "control_rate",
            "repository_commit", "tool_version", "end_outcome", "end_time", "end_ticks", "series",
        }  # fmt: skip
        assert set(h5.attrs) == expected


@pytest.mark.req("REC-024")
def test_rec_024_replay_from_archive_alone(rows: list[TickRecord], archive_path: Path) -> None:
    """A run archive shall hold everything replay needs, so that a run replays without the engine that
    produced it: replay yields the recorded rows, equal field for field, in tick order."""
    with RunArchive(archive_path) as archive:
        replayed = list(archive.records())
        assert archive.identity.design_file == identity().design_file
    assert replayed == rows


@pytest.mark.req("REC-024")
def test_rec_024_replay_without_contacts_or_events(tmp_path: Path) -> None:
    rows = synthetic_rows(11, 25, events=False, contacts=False)
    path = _record(rows, tmp_path / "quiet.h5")
    with RunArchive(path) as archive:
        assert archive.events() == ()
        assert archive.events_table().shape == (0,)
        assert archive.series("physics/contacts/force").values.shape == (0, 3)
        assert list(archive.records()) == rows


@pytest.mark.req("REC-024")
def test_rec_024_empty_run(tmp_path: Path) -> None:
    path = tmp_path / "empty.h5"
    Recorder(BASE_RATE).flush(path, identity(), finished(0))
    with RunArchive(path) as archive:
        assert list(archive.records()) == []
        assert archive.series("time/tick").values.shape == (0,)
        assert archive.series("physics/bodies/position").values.shape == (0, 0, 3)
        assert archive.named("signals/pins") == ()
        assert archive.controller_states() == ()
        assert archive.events() == ()
    again = tmp_path / "empty2.h5"
    Recorder(BASE_RATE).flush(again, identity(), finished(0))
    assert archives_identical(path, again).identical


@pytest.mark.req("REC-025")
def test_rec_025_run_end_recorded(tmp_path: Path) -> None:
    """The run archive shall record how the run ended: end condition reached, or the run error report with
    its simulated time."""
    rows = synthetic_rows(2, 10)
    done = _record(rows, tmp_path / "done.h5", end=RunEnd("end_condition_met", 10, 10))
    failed_end = RunEnd("run_error", 9, 9, RunFault("physics", 9, "solver did not converge"))
    failed = _record(rows, tmp_path / "failed.h5", end=failed_end)
    with RunArchive(done) as archive:
        assert archive.end == RunEnd("end_condition_met", 10, 10)
    with RunArchive(failed) as archive:
        assert archive.end == failed_end
    with h5py.File(failed, "r") as h5:
        assert h5.attrs["fault_time"].dtype == np.int64
        assert h5.attrs["fault_party"] == "physics"


@pytest.mark.req("REC-025")
@pytest.mark.parametrize(
    ("outcome", "fault", "message"),
    [
        ("stopped", None, "run outcome"),
        ("run_error", None, "must carry a fault"),
        ("end_condition_met", RunFault("p", 0, "c"), "must carry a fault"),
        ("run_error", RunFault("p", 0, "a\0b"), "NUL"),
    ],
)
def test_rec_025_run_end_rejects_inconsistent(outcome: str, fault: RunFault | None, message: str) -> None:
    with pytest.raises(RecordError, match=message):
        RunEnd(outcome, 0, 0, fault)


def _row(time_: int, **changes: object) -> TickRecord:
    return dataclasses.replace(TickRecord(time=time_), **changes)  # type: ignore[arg-type]


@pytest.mark.req("REC-005")
@pytest.mark.parametrize(
    ("first", "second", "message"),
    [
        ({"physics": PhysicsOutputs(bodies=(BodyState(),))}, {}, "bodies changed"),
        ({"physics": PhysicsOutputs(joint_positions=(0.0,))}, {}, "joint_coordinates changed"),
        ({"physics": PhysicsOutputs(payloads=(PayloadState(),))}, {}, "payloads changed"),
        ({"pin_signals": (SeriesSample("a.b", "V", 0.0),)}, {"pin_signals": (SeriesSample("a.c", "V", 0.0),)}, "named"),
        ({"rails": (SeriesSample("r", "V", 0.0),)}, {"rails": (SeriesSample("r", "A", 0.0),)}, "named"),
    ],
)
def test_rec_005_layout_fixed_by_first_row(first: dict[str, object], second: dict[str, object], message: str) -> None:
    recorder = Recorder(BASE_RATE)
    recorder.write_row(_row(0, **first))
    with pytest.raises(RecordError, match=message):
        recorder.write_row(_row(1, **second))


@pytest.mark.req("REC-005")
@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"pin_signals": (SeriesSample("a", "V", 0.0), SeriesSample("a", "V", 1.0))}, "duplicate"),
        ({"pin_signals": (SeriesSample("", "V", 0.0),)}, "empty series name"),
        ({"rails": (SeriesSample("a/b", "V", 0.0),)}, "contains '/'"),
        ({"actuator_efforts": (SeriesSample("m", "N\0", 0.0),)}, "NUL"),
        ({"controller_state": "run\0"}, "NUL"),
        ({"events": (RecordedEvent(0, "s", "k", "d\0"),)}, "NUL"),
    ],
)
def test_rec_005_rejects_unrecordable_names(changes: dict[str, object], message: str) -> None:
    with pytest.raises(RecordError, match=message):
        Recorder(BASE_RATE).write_row(_row(0, **changes))


@pytest.mark.req("REC-012")
def test_rec_012_reader_rejects_foreign_files(tmp_path: Path, archive_path: Path) -> None:
    foreign = tmp_path / "foreign.h5"
    with h5py.File(foreign, "w") as h5:
        h5.attrs["format"] = "something else"
    with pytest.raises(RecordError, match="is not a mechsim-run-archive"):
        RunArchive(foreign)
    with RunArchive(archive_path) as archive, pytest.raises(RecordError, match="no series"):
        archive.series("physics/nothing")
    with pytest.raises(RecordError, match="base rate"):
        Recorder(Fraction(0))


@pytest.mark.req("REC-012")
def test_rec_012_large_run_chunking_and_time_bound(tmp_path: Path) -> None:
    """100 000 ticks: record, write and read back within 60 s. The work is linear in rows (one buffer append per
    series per row, doubling growth, one chunked write); the bound leaves an order of magnitude over a local run on
    shared CI. Every per-tick dataset is chunked in fixed CHUNK_ROWS rows, uncompressed."""
    ticks = 100_000
    start = time.perf_counter()
    recorder = Recorder(BASE_RATE)
    for tick in range(ticks):
        recorder.write_row(
            TickRecord(
                time=tick,
                physics=PhysicsOutputs(steps=tick + 1, bodies=(BodyState(position=(tick * 1e-3, 0.0, 0.0)),)),
                pin_signals=(SeriesSample("mcu.pa0", "V", float(tick % 2)),),
                controller_state="run",
            )
        )
    path = tmp_path / "large.h5"
    recorder.flush(path, identity(), finished(ticks))
    with RunArchive(path) as archive:
        position = archive.series("physics/bodies/position").values
        replayed = sum(1 for _ in archive.records())
    elapsed = time.perf_counter() - start
    print(f"100 000 ticks recorded, written and replayed in {elapsed:.2f} s (bound 60 s)")
    assert elapsed < 60.0
    assert replayed == ticks
    assert position[:, 0, 0].tolist() == [tick * 1e-3 for tick in range(ticks)]
    with h5py.File(path, "r") as h5:
        assert h5["physics/bodies/position"].chunks == (CHUNK_ROWS, 1, 3)
        assert h5["time/tick"].chunks == (CHUNK_ROWS,)
        assert h5["time/tick"].compression is None
