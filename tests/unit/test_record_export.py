"""CSV, JSON and MAT exports and the archive comparison (REC-015 to REC-018, REC-022), over synthetic runs.

Exports must read back exactly: CSV and JSON carry floats in shortest round-trip form and MAT stores float64, so
every comparison is exact equality.
"""

from __future__ import annotations

import csv
import dataclasses
import json
import math
from pathlib import Path

import h5py
import numpy as np
import pytest
import scipy.io

from mechsim.record import (
    Recorder,
    RecordError,
    RunArchive,
    TickRecord,
    archives_identical,
    export_csv,
    export_json,
    export_mat,
)
from mechsim.record.export import columns
from mechsim.record.layout import JOINT_POSITIONS, TICK_AXIS, Columns, Series
from mechsim.record.tick import BodyState, PhysicsOutputs
from tests.unit.test_record_support import BASE_RATE, finished, identity, synthetic_rows

SET = ("physics/bodies/position", "signals/pins/mcu.pa0", "physics/ropes/slack", "controller/state")


def _record(rows: list[TickRecord], path: Path) -> Path:
    recorder = Recorder(BASE_RATE)
    for row in rows:
        recorder.write_row(row)
    recorder.flush(path, identity(), finished(len(rows)))
    return path


@pytest.fixture(scope="module")
def archive_path(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return _record(synthetic_rows(13, 50), tmp_path_factory.mktemp("export") / "run.h5")


def _read_csv(path: Path) -> tuple[list[str], list[list[str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        table = list(csv.reader(handle))
    return table[0], table[1:]


@pytest.mark.req("REC-015")
def test_rec_015_csv_round_trip(archive_path: Path, tmp_path: Path) -> None:
    """The recorder shall export any recorded series as CSV."""
    out = tmp_path / "run.csv"
    with RunArchive(archive_path) as archive:
        export_csv(archive, out, SET)
        position = archive.series("physics/bodies/position").values
        pin = archive.series("signals/pins/mcu.pa0").values
        slack = archive.series("physics/ropes/slack").values
        ticks = archive.series("time/tick").values
        seconds = archive.series("time/seconds").values
    header, body = _read_csv(out)
    assert header[:2] == ["time/tick [1]", "time/seconds [s]"]
    assert header[2:8] == [f"physics/bodies/position[{b},{a}] [m]" for b in range(2) for a in range(3)]
    assert header[8:] == ["signals/pins/mcu.pa0 [V]", "physics/ropes/slack[0] [1]", "controller/state [1]"]
    assert len(body) == 50
    columns = list(zip(*body, strict=True))
    assert [int(v) for v in columns[0]] == ticks.tolist()
    assert [float(v) for v in columns[1]] == seconds.tolist()
    assert [float(v) for v in columns[3]] == position[:, 0, 1].tolist()
    assert [float(v) for v in columns[8]] == pin.tolist()
    assert [bool(int(v)) for v in columns[9]] == slack[:, 0].tolist()


@pytest.mark.req("REC-015")
def test_rec_015_csv_of_contacts_and_of_time_only(archive_path: Path, tmp_path: Path) -> None:
    with RunArchive(archive_path) as archive:
        export_csv(archive, tmp_path / "contacts.csv", ["physics/contacts/force"])
        export_csv(archive, tmp_path / "time.csv", [])
        contacts = archive.series("physics/contacts/force").values
    header, body = _read_csv(tmp_path / "contacts.csv")
    assert header[:2] == ["physics/contacts/tick [1]", "physics/contacts/seconds [s]"]
    assert header[2] == "physics/contacts/force[0] [N]"
    assert [[float(v) for v in row[2:]] for row in body] == contacts.tolist()
    header, body = _read_csv(tmp_path / "time.csv")
    assert header == ["time/tick [1]", "time/seconds [s]"]
    assert len(body) == 50


@pytest.mark.req("REC-015")
def test_rec_015_mixed_axes_and_unknown_series_rejected(archive_path: Path, tmp_path: Path) -> None:
    with RunArchive(archive_path) as archive:
        with pytest.raises(RecordError, match="contact axis"):
            export_csv(archive, tmp_path / "x.csv", ["physics/contacts/force", "physics/steps"])
        with pytest.raises(RecordError, match="no series"):
            export_csv(archive, tmp_path / "x.csv", ["physics/nothing"])


@pytest.mark.req("REC-015")
def test_rec_015_csv_of_empty_run(tmp_path: Path) -> None:
    path = _record([], tmp_path / "empty.h5")
    with RunArchive(path) as archive:
        export_csv(archive, tmp_path / "empty.csv", ["physics/bodies/position", "physics/steps"])
    header, body = _read_csv(tmp_path / "empty.csv")
    assert header == ["time/tick [1]", "time/seconds [s]", "physics/steps [1]"]
    assert body == []


@pytest.mark.req("REC-016")
def test_rec_016_json_events_and_series(archive_path: Path, tmp_path: Path) -> None:
    """The recorder shall export any recorded series as JSON, with every event."""
    out = tmp_path / "run.json"
    with RunArchive(archive_path) as archive:
        export_json(archive, out, SET)
        events = archive.events()
        table = archive.events_table()
        position = archive.series("physics/bodies/position").values
        identity_ = archive.identity
    document = json.loads(out.read_text(encoding="utf-8"))
    assert list(document["series"]) == ["time/tick", "time/seconds", *SET]
    assert document["series"]["physics/bodies/position"]["unit"] == "m"
    assert document["series"]["physics/bodies/position"]["values"] == position.tolist()
    assert document["series"]["signals/pins/mcu.pa0"]["unit"] == "V"
    assert [(e["row"], e["tick"], e["source"], e["kind"], e["detail"]) for e in document["events"]] == [
        (row, e.time, e.source, e.kind, e.detail) for row, e in events
    ]
    assert [e["seconds"] for e in document["events"]] == table["seconds"].tolist()
    assert document["identity"]["seed"] == identity_.seed
    assert document["identity"]["base_rate_hz"] == [1000, 1]
    assert document["end"] == {"outcome": "end_condition_met", "end_time": 50, "ticks": 50, "fault": None}


@pytest.mark.req("REC-016")
def test_rec_016_json_rejects_nan(tmp_path: Path) -> None:
    row = TickRecord(time=0, physics=PhysicsOutputs(bodies=(BodyState(position=(math.nan, 0.0, 0.0)),)))
    path = _record([row], tmp_path / "nan.h5")
    with RunArchive(path) as archive, pytest.raises(RecordError, match="JSON export"):
        export_json(archive, tmp_path / "nan.json", ["physics/bodies/position"])


@pytest.mark.req("REC-017")
def test_rec_017_mat_round_trip(archive_path: Path, tmp_path: Path) -> None:
    """The recorder shall export any recorded series as a MAT file, under the archive names."""
    out = tmp_path / "run.mat"
    with RunArchive(archive_path) as archive:
        export_mat(archive, out, SET)
        expected = {name: archive.series(name) for name in ("time/tick", "time/seconds", *SET)}
    mat = scipy.io.loadmat(out)
    names = [str(item[0]) for item in mat["series_names"][0]]
    units = [str(item[0]) for item in mat["series_units"][0]]
    assert names == list(expected)
    assert units == [series.unit for series in expected.values()]
    for position, series in enumerate(expected.values()):
        values = mat["series_values"][0, position]
        np.testing.assert_array_equal(values.reshape(series.values.shape), series.values)


@pytest.mark.req("REC-018")
def test_rec_018_exports_state_units(archive_path: Path, tmp_path: Path) -> None:
    """Every export states the unit of each quantity explicitly."""
    with RunArchive(archive_path) as archive:
        names = [n for n in archive.series_names() if archive.series(n).axis == "tick"]
        export_csv(archive, tmp_path / "all.csv", names)
        export_json(archive, tmp_path / "all.json", names)
    header, _ = _read_csv(tmp_path / "all.csv")
    assert all(cell.endswith("]") and " [" in cell for cell in header)
    document = json.loads((tmp_path / "all.json").read_text(encoding="utf-8"))
    assert all(entry["unit"] != "" for entry in document["series"].values())


@pytest.mark.req("REC-022")
def test_rec_022_one_value_change_is_named(tmp_path: Path) -> None:
    """A difference in one recorded value names the dataset and the first differing index."""
    rows = synthetic_rows(21, 30)
    first = _record(rows, tmp_path / "a.h5")
    changed = list(rows)
    old = changed[17]
    bodies = (old.physics.bodies[0], BodyState((9.0, 9.0, 9.0), *(_rest(old.physics.bodies[1]))))
    changed[17] = TickRecord(
        old.time,
        dataclasses.replace(old.physics, bodies=bodies),
        old.pin_signals,
        old.rails,
        old.actuator_efforts,
        old.controller_state,
        old.events,
    )
    second = _record(changed, tmp_path / "b.h5")
    report = archives_identical(first, second)
    assert not report.identical
    assert not report.content_identical
    assert report.first_difference == "dataset physics/bodies/position: first difference at index (17, 1, 0)"


def _rest(body: BodyState) -> tuple[object, ...]:
    return (body.orientation, body.angular_velocity, body.linear_velocity)


@pytest.mark.req("REC-022")
def test_rec_022_attribute_change_is_named(tmp_path: Path) -> None:
    rows = synthetic_rows(22, 10)
    first = _record(rows, tmp_path / "a.h5")
    recorder = Recorder(BASE_RATE)
    for row in rows:
        recorder.write_row(row)
    second = tmp_path / "b.h5"
    recorder.flush(second, identity(seed=43), finished(10))
    report = archives_identical(first, second)
    assert not report.content_identical
    assert report.first_difference is not None
    assert report.first_difference.startswith("root attribute 'seed'")


@pytest.mark.req("REC-022")
def test_rec_022_same_content_different_bytes(tmp_path: Path) -> None:
    """A copy with the same attributes and data but another file layout is content-identical, not identical."""
    first = _record(synthetic_rows(23, 10), tmp_path / "a.h5")
    copy = tmp_path / "copy.h5"
    with h5py.File(first, "r") as source, h5py.File(copy, "w", libver="latest") as target:
        for name, value in source.attrs.items():
            target.attrs[name] = value
        for name in source:
            source.copy(source[name], target, name=name)
    report = archives_identical(first, copy)
    assert not report.identical
    assert report.content_identical
    assert report.first_difference is not None
    assert "recorded content identical" in report.first_difference


@pytest.mark.req("REC-022")
def test_rec_022_structure_difference_is_named(tmp_path: Path) -> None:
    first = _record(synthetic_rows(24, 10), tmp_path / "a.h5")
    fewer = _record(synthetic_rows(24, 9), tmp_path / "b.h5")
    report = archives_identical(first, fewer)
    assert report.first_difference is not None
    assert report.first_difference.startswith("root attribute 'end_")
    extra = tmp_path / "c.h5"
    extra.write_bytes(first.read_bytes())
    with h5py.File(extra, "a") as h5:
        h5.create_dataset("extra", data=np.zeros(1))
    report = archives_identical(first, extra)
    assert report.first_difference == "extra: only in the second archive"


# ===== Per-column names and units (REC-018, ruling 18) ==================

LABELS = Columns(("trolley", "hoist"), ("m", "rad"))


def _labelled(tmp_path: Path, columns: dict[str, Columns]) -> Path:
    rows = [
        TickRecord(time=tick, physics=PhysicsOutputs(joint_positions=(0.5 * tick, 0.25 * tick))) for tick in range(3)
    ]
    recorder = Recorder(BASE_RATE, columns)
    for row in rows:
        recorder.write_row(row)
    path = tmp_path / "labelled.h5"
    recorder.flush(path, identity(), finished(len(rows)))
    return path


@pytest.mark.req("REC-015", "REC-018")
def test_csv_header_names_each_index_column_and_its_unit(tmp_path: Path) -> None:
    path = _labelled(tmp_path, {JOINT_POSITIONS: LABELS})
    out = tmp_path / "joints.csv"
    with RunArchive(path) as archive:
        export_csv(archive, out, [JOINT_POSITIONS])
    header, rows = _read_csv(out)
    assert header[-2:] == [f"{JOINT_POSITIONS}[trolley] [m]", f"{JOINT_POSITIONS}[hoist] [rad]"]
    assert [row[-2:] for row in rows] == [["0.0", "0.0"], ["0.5", "0.25"], ["1.0", "0.5"]]


@pytest.mark.req("REC-015", "REC-018")
def test_csv_header_without_column_labels_keeps_indices_and_the_series_unit(tmp_path: Path) -> None:
    path = _labelled(tmp_path, {})
    out = tmp_path / "joints.csv"
    with RunArchive(path) as archive:
        export_csv(archive, out, [JOINT_POSITIONS])
    header, _ = _read_csv(out)
    assert header[-2:] == [f"{JOINT_POSITIONS}[0] [m or rad]", f"{JOINT_POSITIONS}[1] [m or rad]"]


@pytest.mark.req("REC-018")
def test_names_without_units_label_the_columns_with_the_series_unit() -> None:
    series = Series(np.zeros((2, 1, 3)), "N", TICK_AXIS, names=("load",))
    assert [(name, unit) for name, unit, _ in columns("f", series)] == [
        ("f[load,0]", "N"),
        ("f[load,1]", "N"),
        ("f[load,2]", "N"),
    ]


@pytest.mark.req("REC-016", "REC-018")
def test_json_series_carry_column_names_and_units(tmp_path: Path) -> None:
    path = _labelled(tmp_path, {JOINT_POSITIONS: LABELS})
    out = tmp_path / "joints.json"
    with RunArchive(path) as archive:
        export_json(archive, out, [JOINT_POSITIONS])
    document = json.loads(out.read_text(encoding="utf-8"))
    entry = document["series"][JOINT_POSITIONS]
    assert (entry["names"], entry["units"]) == (["trolley", "hoist"], ["m", "rad"])
    assert "names" not in document["series"]["time/seconds"]


@pytest.mark.req("REC-017", "REC-018")
def test_mat_carries_column_names_and_units(tmp_path: Path) -> None:
    path = _labelled(tmp_path, {JOINT_POSITIONS: LABELS})
    out = tmp_path / "joints.mat"
    with RunArchive(path) as archive:
        export_mat(archive, out, [JOINT_POSITIONS])
    data = scipy.io.loadmat(out)
    names = [str(item[0]) for item in data["series_names"][0]]
    position = names.index(JOINT_POSITIONS)
    assert [str(item[0]) for item in data["series_columns"][0, position][0]] == ["trolley", "hoist"]
    assert [str(item[0]) for item in data["series_column_units"][0, position][0]] == ["m", "rad"]
    assert data["series_columns"][0, names.index("time/seconds")].size == 0
