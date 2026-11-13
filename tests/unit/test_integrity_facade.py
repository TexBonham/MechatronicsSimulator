"""Integrity through the facade and the runner (FAC-013, INT-022)."""

from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path
from typing import Any

import pytest

from mechsim.facade.protocol import parse_message
from mechsim.facade.service import Facade, FacadeError, FacadeServices
from mechsim.integrity.plugin import IntegrityError
from mechsim.testing.driver import IntegrityResults, MemberIntegrity, RunMetadata
from mechsim.testing.runner import run_case
from tests.unit.test_facade_service import SECTIONS, _OverlayReader
from tests.unit.test_runner import BIND, fake, machine, services
from tests.unit.test_runner_fakes import FakeArchiver

TABLE = IntegrityResults(
    (
        MemberIntegrity("rail_x", 2.0e6, 0.003, 0.004, 0.003, 12.5, 0.003, ()),
        MemberIntegrity(
            "rail_y", 3.0e7, 0.005, 0.02, 0.005, 1.2, 0.005, ("deflection_over_target", "safety_factor_below_minimum")
        ),
    )
)


class _IntegrityReader(_OverlayReader):
    def __init__(self, integrity: IntegrityResults | None) -> None:
        super().__init__(SECTIONS)
        self.integrity = integrity

    def open(self, path: Path) -> tuple[RunMetadata, Any]:
        metadata, view = super().open(path)
        return replace(metadata, integrity=self.integrity), view


def _facade(tmp_path: Path, integrity: IntegrityResults | None = TABLE) -> Facade:
    archive = tmp_path / "S1-seed0.h5"
    archive.write_bytes(b"not an hdf5 file")
    facade = Facade(FacadeServices(reader=_IntegrityReader(integrity)))
    facade.replay_open(archive)
    return facade


@pytest.mark.req("FAC-013")
def test_the_facade_serves_the_integrity_table(tmp_path: Path) -> None:
    result = _facade(tmp_path).integrity_results("S1-seed0")
    assert result.run_id == "S1-seed0"
    assert [entry.member for entry in result.members] == ["rail_x", "rail_y"]
    assert result.members[1].flags == ["deflection_over_target", "safety_factor_below_minimum"]
    assert (result.members[0].peak_stress, result.members[0].safety_factor) == (2.0e6, 12.5)
    assert (result.members[0].deflection, result.members[0].deflection_evaluated) == (0.004, True)
    bare = tmp_path / "bare"
    bare.mkdir()
    with pytest.raises(FacadeError) as raised:
        _facade(bare, None).integrity_results("S1-seed0")
    assert (raised.value.code, raised.value.requirement_id) == ("not_found", "FAC-013")


@pytest.mark.req("FAC-013", "INT-006")
def test_a_quantity_integrity_did_not_evaluate_is_an_explicit_state_not_a_nan(tmp_path: Path) -> None:
    # A member built rigid has no node poses: its deflection is NaN in the table (INT-006); through the facade it is
    # None with its flag false, so no client reads a NaN as a number (invariant 7). An evaluated zero stays a number.
    rigid = MemberIntegrity("bridge", 0.0, 0.0, math.nan, 0.0, math.nan, 0.0, ("deflection_not_evaluated",))
    result = _facade(tmp_path, IntegrityResults((rigid,))).integrity_results("S1-seed0")
    (row,) = result.members
    assert (row.deflection, row.deflection_evaluated) == (None, False)
    assert (row.safety_factor, row.safety_factor_evaluated) == (None, False)
    assert (row.peak_stress, row.peak_stress_evaluated) == (0.0, True)
    assert row.flags == ["deflection_not_evaluated"]
    assert "NaN" not in row.model_dump_json()


@pytest.mark.req("FAC-013")
def test_the_facade_serves_the_load_case_export(tmp_path: Path) -> None:
    facade = _facade(tmp_path)
    result = facade.load_case("S1-seed0", "json")
    data = json.loads(result.text)
    assert (result.format, result.design_version, data["design"]) == (
        "json",
        "0.1.0",
        {"name": "machine", "version": "0.1.0"},
    )
    loads = {(entry["member"], entry["end"], entry["quantity"]): entry for entry in data["loads"]}
    # Start force of member k at tick t is (10 k, t, 0.5): member 1 peaks at t = 5, |(10, 5, 0.5)|.
    peak = loads[("rail_y", "start", "force")]
    assert peak["magnitude"] == pytest.approx(math.sqrt(100.0 + 25.0 + 0.25))
    assert (peak["time"], peak["x"]) == (0.005, 0.0)
    assert loads[("rail_y", "end", "force")]["x"] == 0.5
    text = facade.load_case("S1-seed0", "csv").text
    assert text.splitlines()[1].startswith("member,end,x,quantity")
    with pytest.raises(FacadeError) as raised:
        facade.load_case("S1-seed0", "xml")
    assert (raised.value.code, raised.value.field) == ("invalid_message", "format")


@pytest.mark.req("FAC-013")
def test_the_load_case_of_an_archive_missing_a_member_is_not_found(tmp_path: Path) -> None:
    class _Ghost(_IntegrityReader):
        def open(self, path: Path) -> tuple[RunMetadata, Any]:
            metadata, view = super().open(path)
            return replace(metadata, members=(replace(SECTIONS[0], name="ghost"),)), view

    archive = tmp_path / "S1-seed0.h5"
    archive.write_bytes(b"x")
    facade = Facade(FacadeServices(reader=_Ghost(TABLE)))
    facade.replay_open(archive)
    with pytest.raises(FacadeError) as raised:
        facade.load_case("S1-seed0", "json")
    assert (raised.value.code, raised.value.requirement_id) == ("not_found", "FAC-013")


@pytest.mark.req("FAC-013")
def test_the_protocol_carries_the_integrity_requests() -> None:
    table = parse_message(
        json.dumps({"version": 1, "message_id": "a", "kind": "integrity_results", "payload": {"run_id": "S1-seed0"}})
    )
    case = parse_message(
        json.dumps(
            {"version": 1, "message_id": "b", "kind": "load_case", "payload": {"run_id": "S1-seed0", "format": "csv"}}
        )
    )
    assert (table.kind, case.kind) == ("integrity_results", "load_case")
    with pytest.raises(Exception, match="format"):
        parse_message(
            json.dumps(
                {
                    "version": 1,
                    "message_id": "c",
                    "kind": "load_case",
                    "payload": {"run_id": "S1-seed0", "format": "xml"},
                }
            )
        )


@pytest.mark.req("INT-022")
def test_the_runner_attaches_the_table_and_writes_it_beside_the_archive(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)
    archiver = FakeArchiver()
    seen: list[Any] = []

    def integrity(build: Any, case: Any, recording: Any) -> IntegrityResults:
        seen.append(build)
        return TABLE

    result = run_case(case, services(designs, fake(), archiver=archiver, integrity=integrity), BIND, tmp_path / "runs")
    assert result.metadata is not None
    assert result.metadata.integrity == TABLE
    assert result.archive is not None
    beside = result.archive.with_name(f"{result.archive.stem}.integrity.json")
    assert json.loads(beside.read_text(encoding="utf-8")) == json.loads(TABLE.to_json())
    assert len(seen) == 1


@pytest.mark.req("INT-022")
def test_an_integrity_error_fails_the_run_with_a_report(tmp_path: Path) -> None:
    designs, case = machine(tmp_path)

    def integrity(build: Any, case: Any, recording: Any) -> IntegrityResults:
        raise IntegrityError("member 'rail': no recorded end-loads")

    result = run_case(case, services(designs, fake(), integrity=integrity), BIND, tmp_path / "runs")
    assert result.score.status == "error"
    assert result.score.error == "run error: member 'rail': no recorded end-loads"
