"""Structural series, integrity events and structural run metadata in the recorder and archive (REC-004, REC-010,
REC-026; brief R). Synthetic rows: two lumped members, two ropes, one integrity crossing."""

from __future__ import annotations

import csv
import hashlib
import json
from fractions import Fraction
from pathlib import Path

import h5py
import numpy as np
import pytest

from mechsim.record import Recorder, RunArchive
from mechsim.record.archive import write_archive
from mechsim.record.export import export_csv, export_json
from mechsim.record.layout import MEMBER_SERIES, ROPE_MAX_TENSION, ROPE_SERIES, ROPE_TENSION, Columns
from mechsim.record.reader import Kinematics, RecordArchiveReader, RecordingView
from mechsim.record.tick import (
    MemberEndLoads,
    PhysicsOutputs,
    RecordedEvent,
    RecordError,
    RopeTension,
    TickRecord,
    Wrench,
)
from mechsim.testing.core_driver import RecordArchiver
from mechsim.testing.driver import (
    INTEGRITY_KIND,
    INTEGRITY_SOURCE,
    IntegrityCrossing,
    IntegrityResults,
    MaterialOverridesRecord,
    MemberIntegrity,
    MemberSection,
    RunMetadata,
    RunReport,
    StructuralError,
)
from tests.unit.test_record_support import finished, identity

MEMBERS = ("rail_x", "rail_y")
ROPES = ("hoist",)
TICKS = 6
CROSSING_TICK = 4
CROSSING = IntegrityCrossing("rail_x", "stress", 3.5e7, 3.0e7)


def _row(tick: int) -> TickRecord:
    members = tuple(
        MemberEndLoads(
            Wrench((tick + 0.1 * k, 2.0 * tick, -1.0 - k), (10.0 * k, tick, 0.5)),
            Wrench((-tick, 3.0 + k, 4.0 * tick), (1.0, -2.0, tick + k)),
        )
        for k in range(len(MEMBERS))
    )
    ropes = (RopeTension(100.0 + tick, tick == 2, 120.0 + 2.0 * tick),)
    events: tuple[RecordedEvent, ...] = ()
    if tick == CROSSING_TICK:
        events = (RecordedEvent(tick, INTEGRITY_SOURCE, INTEGRITY_KIND, CROSSING.detail()),)
    return TickRecord(time=tick, physics=PhysicsOutputs(steps=tick + 1, members=members, ropes=ropes), events=events)


def _columns() -> dict[str, Columns]:
    columns = {path: Columns(MEMBERS) for path in MEMBER_SERIES}
    columns.update({path: Columns(ROPES) for path in ROPE_SERIES})
    return columns


@pytest.fixture
def rows() -> list[TickRecord]:
    return [_row(tick) for tick in range(TICKS)]


@pytest.fixture
def archive_path(rows: list[TickRecord], tmp_path: Path) -> Path:
    recorder = Recorder(Fraction(1000), _columns())
    for row in rows:
        recorder.write_row(row)
    path = tmp_path / "structural.h5"
    write_archive(path, identity(), finished(TICKS), recorder.recording())
    return path


@pytest.mark.req("REC-004")
def test_member_end_loads_carry_member_names_and_units_every_tick(rows: list[TickRecord], archive_path: Path) -> None:
    """Start and end wrenches, six values each, per member per physics tick, columns named by member."""
    with RunArchive(archive_path) as archive:
        for path, unit, end, part in (
            (MEMBER_SERIES[0], "N m", "start", "moment"),
            (MEMBER_SERIES[1], "N", "start", "force"),
            (MEMBER_SERIES[2], "N m", "end", "moment"),
            (MEMBER_SERIES[3], "N", "end", "force"),
        ):
            series = archive.series(path)
            assert series.unit == unit
            assert series.names == MEMBERS
            assert series.values.shape == (TICKS, len(MEMBERS), 3)
            expected = [[getattr(getattr(m, end), part) for m in row.physics.members] for row in rows]
            np.testing.assert_array_equal(series.values, np.array(expected))
        replayed = list(archive.records())
    assert [row.physics.members for row in replayed] == [row.physics.members for row in rows]


@pytest.mark.req("REC-004")
def test_member_names_must_match_the_member_count(rows: list[TickRecord]) -> None:
    recorder = Recorder(Fraction(1000), {MEMBER_SERIES[0]: Columns(("only_one",))})
    with pytest.raises(RecordError, match="physics/members/start_moment has 2 columns, 1 names given"):
        recorder.write_row(rows[0])


@pytest.mark.req("REC-026")
def test_rope_load_end_and_max_tension_every_tick(rows: list[TickRecord], archive_path: Path) -> None:
    """Load-end tension (`tension`) and maximum segment tension per rope per physics tick, N, named by rope."""
    with RunArchive(archive_path) as archive:
        load_end = archive.series(ROPE_TENSION)
        maximum = archive.series(ROPE_MAX_TENSION)
        assert (load_end.unit, maximum.unit) == ("N", "N")
        assert load_end.names == ROPES
        assert maximum.names == ROPES
        assert load_end.values[:, 0].tolist() == [100.0 + tick for tick in range(TICKS)]
        assert maximum.values[:, 0].tolist() == [120.0 + 2.0 * tick for tick in range(TICKS)]
        assert archive.series("physics/ropes/slack").values[:, 0].tolist() == [tick == 2 for tick in range(TICKS)]
        replayed = list(archive.records())
    assert [row.physics.ropes for row in replayed] == [row.physics.ropes for row in rows]


@pytest.mark.req("REC-026")
def test_an_archive_without_the_max_tension_series_replays_the_load_end_tension(archive_path: Path) -> None:
    """A format 2 archive written before the series existed: the maximum reads as the one tension recorded."""
    with h5py.File(archive_path, "r+") as h5:
        del h5[ROPE_MAX_TENSION]
        names = [name.decode() if isinstance(name, bytes) else name for name in h5.attrs["series"]]
        h5.attrs["series"] = np.array([name for name in names if name != ROPE_MAX_TENSION], dtype=h5py.string_dtype())
    with RunArchive(archive_path) as archive:
        ropes = [row.physics.ropes[0] for row in archive.records()]
    assert [rope.max_tension for rope in ropes] == [rope.tension for rope in ropes]


@pytest.mark.req("REC-026")
def test_exports_carry_the_member_and_rope_series(archive_path: Path, tmp_path: Path) -> None:
    names = [MEMBER_SERIES[0], ROPE_TENSION, ROPE_MAX_TENSION]
    with RunArchive(archive_path) as archive:
        export_csv(archive, tmp_path / "out.csv", names)
        export_json(archive, tmp_path / "out.json", names)
    with (tmp_path / "out.csv").open(encoding="utf-8") as handle:
        header = next(csv.reader(handle))
    assert "physics/ropes/max_tension[hoist] [N]" in header
    assert "physics/members/start_moment[rail_y,2] [N m]" in header
    exported = json.loads((tmp_path / "out.json").read_text(encoding="utf-8"))
    assert json.dumps(exported).count("max_tension") >= 1


@pytest.mark.req("REC-010")
def test_integrity_events_are_recorded_with_their_simulated_time(archive_path: Path) -> None:
    """Kind and source `integrity`, detail the crossing (member, quantity, value, limit, tier), at its tick."""
    with RunArchive(archive_path) as archive:
        table = archive.events_table()
        events = archive.events()
    crossings = [(row, event) for row, event in events if event.kind == INTEGRITY_KIND]
    assert len(crossings) == 1
    row, event = crossings[0]
    assert (row, event.time, event.source) == (CROSSING_TICK, CROSSING_TICK, INTEGRITY_SOURCE)
    assert IntegrityCrossing.parse(event.detail) == CROSSING
    seconds = [float(item["seconds"]) for item in table if item["kind"] == INTEGRITY_KIND.encode()]
    assert seconds == [CROSSING_TICK / 1000.0]
    assert json.loads(event.detail) == {
        "limit": 3.0e7,
        "member": "rail_x",
        "quantity": "stress",
        "tier": 1,
        "value": 3.5e7,
    }


@pytest.mark.req("REC-010")
@pytest.mark.parametrize(
    "detail",
    ["", "{}", '{"member": "a"}', "[1]", '{"member": "a", "quantity": "q", "value": "x", "limit": 1, "tier": 1}'],
)
def test_an_unreadable_integrity_detail_is_an_error(detail: str) -> None:
    with pytest.raises(StructuralError, match="integrity event detail"):
        IntegrityCrossing.parse(detail)


def _metadata(**fields: object) -> RunMetadata:
    values: dict[str, object] = {
        "run_id": "S1-seed42",
        "case_id": "S1",
        "design_name": "synthetic",
        "design_version": "1.2.3",
        "design_hash": "0" * 40,
        "case_hash": "1" * 40,
        "seed": 42,
        "controller_mode": None,
        "sweep": None,
        "report": RunReport("end_time", TICKS - 1, Fraction(1000)),
    }
    values.update(fields)
    return RunMetadata(**values)  # type: ignore[arg-type]


@pytest.mark.req("REC-004")
def test_run_metadata_records_the_member_list_overrides_variation_and_integrity(
    rows: list[TickRecord], tmp_path: Path
) -> None:
    """The archive keeps the structural member list (BLD-008), the material overrides text and sha256 (brief M's
    key), a variation's drawn values and the integrity table, and the reader returns them unchanged."""
    recorder = Recorder(Fraction(1000), _columns())
    for row in rows:
        recorder.write_row(row)
    view = RecordingView(recorder.recording(), Kinematics((), (), ()))
    text = "material_overrides:\n  petg_printed: {}\n"
    overrides = MaterialOverridesRecord(text, hashlib.sha256(text.encode()).hexdigest())
    members = (
        MemberSection("rail_x", "petg_printed", 4e-4, 1.3e-8, 0.5),
        MemberSection("rail_y", "pla", 1e-4, 2e-9, 0.4, 0.01),
    )
    integrity = IntegrityResults((MemberIntegrity("rail_x", 3.5e7, 0.004, 0.002, 0.003, 1.2, 0.004, ("tier1",)),))
    metadata = _metadata(
        members=members, material_overrides=overrides, variation=(("${damping}", "0.05 kg"),), integrity=integrity
    )
    path = tmp_path / "meta.h5"
    RecordArchiver(tmp_path).write(path, view, metadata)
    read, _ = RecordArchiveReader().open(path)
    assert read.members == members
    assert read.material_overrides == overrides
    assert read.variation == (("${damping}", "0.05 kg"),)
    assert read.integrity == integrity
    with RunArchive(path) as archive:
        assert archive.extra["material_overrides"] == text
        assert archive.extra["material_overrides_sha256"] == overrides.sha256


@pytest.mark.req("REC-004")
def test_run_metadata_without_structural_fields_reads_back_empty(rows: list[TickRecord], tmp_path: Path) -> None:
    recorder = Recorder(Fraction(1000))
    for row in rows:
        recorder.write_row(row)
    path = tmp_path / "plain.h5"
    RecordArchiver(tmp_path).write(path, RecordingView(recorder.recording(), Kinematics((), (), ())), _metadata())
    read, _ = RecordArchiveReader().open(path)
    assert (read.members, read.material_overrides, read.variation, read.integrity) == ((), None, None, None)


@pytest.mark.req("VIE-010")
def test_member_section_fibre_is_stated_or_that_of_the_equivalent_rectangle() -> None:
    # Rectangle b = 0.02 m, h = 0.04 m: A = 8e-4 m^2, I = b h^3 / 12 = 1.0667e-7 m^4, c = h / 2 = 0.02 m exactly.
    area = 0.02 * 0.04
    second = 0.02 * 0.04**3 / 12.0
    assert MemberSection("m", "pla", area, second, 1.0).fibre() == pytest.approx(0.02, rel=1e-12)
    assert MemberSection("m", "pla", area, second, 1.0, 0.03).fibre() == 0.03
