"""Not valid, not fail (rulings 69, 72; brief M3-I5 item 5): a member the engine flags (the one-way carried-load
result, ENG-052) has its post-run rows marked `not_valid` from the flag time (values before it stand), its tier 1
crossings after it recorded as `integrity_not_valid` events, and a criterion reading it inside the interval, or a case
whose only crossings are not valid, unevaluable with the reason; the validity record goes to the archive metadata.

End to end: the data-only carried bar (`tests/unit/test_carried_members.py`) through `CoreDriver`, the runner and the
record archiver. The bar alone, loaded by its resting carriage, is never flagged by the engine (its ratio stays below
the limit), so the flagged run substitutes the engine's record with a flag at a stated time (`carried_validity`, the
one function reading `CarriedRecord.values()`); everything downstream of the record is the real path."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from mechsim.integrity.checks.strength import Strength
from mechsim.integrity.config import parse_config
from mechsim.integrity.plugin import MemberInputs, NoParameters, RunInputs, SectionProperties, Wrenches
from mechsim.integrity.service import IntegrityService
from mechsim.integrity.table import evaluate
from mechsim.record.archive import RunArchive
from mechsim.record.validity import (
    NOT_VALID_FLAG,
    NOT_VALID_KIND,
    VALIDITY_KEY,
    CarriedValidity,
    validity_from_json,
    validity_json,
)
from mechsim.testing import core_driver
from mechsim.testing.core_driver import CoreDriver, RecordArchiver, RecordArchiveReader, core_builder
from mechsim.testing.runner import Services, run_case
from tests.unit.test_carried_members import _design_data

CASE = """id: NV1
title: Carried bar, not-valid results
design:
  name: carried_bar
  version: 1.0.0
seed: 0
initial_state:
  joints: []
rates:
  physics_step: 1 ms
physics:
  adapter: native
end:
  at: 0.04 s
  max_duration: 1 s
criteria:
  - name: bar_safety_factor
    check: value
    quantity: safety_factor:bar
    statistic: minimum
    comparison: at_least
    limit: "0 1"
  - name: bar_deflection_early
    check: value
    quantity: deflection:bar
    statistic: maximum
    comparison: at_most
    limit: 1 m
    window:
      end: 0.01 s
  - name: bar_deflection_late
    check: value
    quantity: deflection:bar
    statistic: maximum
    comparison: at_most
    limit: 1 m
"""
SINCE = 0.02


def _setup(tmp_path: Path, accuracy: str) -> tuple[Path, Services, CoreDriver]:
    data = _design_data(["slide"])
    data["structural"] = {
        "design_load": {"rated_load": "1 kg", "allowance": "0 percent"},
        "accuracy_target": accuracy,
        "deflection_target": "static",
    }
    designs = tmp_path / "designs"
    folder = designs / "carried_bar"
    folder.mkdir(parents=True)
    (folder / "design.yaml").write_text(json.dumps(data, indent=1), encoding="utf-8")
    (folder / "integrity.yaml").write_text(
        "schema_version: 1\nchecks:\n  - check: strength\n  - check: deflection\n", encoding="utf-8"
    )
    case = tmp_path / "NV1.yaml"
    case.write_text(CASE, encoding="utf-8")
    driver = CoreDriver(designs_root=designs)
    services = Services(
        lambda: driver, core_builder, RecordArchiver(designs), designs, integrity=IntegrityService(designs)
    )
    return case, services, driver


def _flag(monkeypatch: pytest.MonkeyPatch, since: float) -> None:
    real = core_driver.carried_validity

    def flagged(model: Any, values: Any) -> tuple[CarriedValidity, ...]:
        return tuple(CarriedValidity(record.member, True, since, 3.5, record.work) for record in real(model, values))

    monkeypatch.setattr(core_driver, "carried_validity", flagged)


@pytest.mark.req("INT-022", "REC-004", "SCO-029")
def test_an_unflagged_run_stores_the_record_and_is_unchanged_bit_for_bit(tmp_path: Path) -> None:
    case, services, driver = _setup(tmp_path, "1 m")
    result: Any = run_case(case, services, None, tmp_path / "runs")
    assert result.report is not None, result.score.error
    assert not result.report.errored, result.report.error
    assert result.score.status == "pass", result.score
    view = driver.recording()
    (record,) = view.validity
    assert record.member == "bar"
    assert not record.flagged
    assert record.since == 0.0
    assert 0.0 < record.ratio < 2.04
    assert record.work > 0.0
    with RunArchive(result.archive) as archive:
        assert validity_from_json(archive.extra[VALIDITY_KEY]) == view.validity
    _, replayed = RecordArchiveReader().open(result.archive)
    assert replayed.validity == view.validity
    build = core_builder(_design(tmp_path), "native")
    stripped = copy.copy(view)
    stripped.validity = ()
    service = IntegrityService(tmp_path / "designs")
    with_record = service(build, None, view)  # type: ignore[arg-type]
    without = service(build, None, stripped)  # type: ignore[arg-type]
    assert with_record is not None
    assert without is not None
    assert with_record.report_json() == without.report_json()
    assert all(event.kind != NOT_VALID_KIND for event in view.events())


def _design(tmp_path: Path) -> Any:
    from mechsim.design.schema import Design

    text = (tmp_path / "designs" / "carried_bar" / "design.yaml").read_text(encoding="utf-8")
    return Design.model_validate(json.loads(text))


@pytest.mark.req("INT-022", "INT-024", "SCO-015", "SCO-029", "REC-004")
def test_a_flagged_member_is_not_valid_from_its_flag_time_end_to_end(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # accuracy 1 nm: the bar's deflection under its weight and the carriage (about 1e-4 m) crosses the tier 1 limit
    # on the first rows, so the crossing lands before or after the flag time as the flag is set.
    _flag(monkeypatch, 0.0)
    case, services, driver = _setup(tmp_path, "1 nm")
    result: Any = run_case(case, services, None, tmp_path / "runs")
    view = driver.recording()
    kinds = [event.kind for event in view.events() if event.source == "integrity"]
    assert kinds == [NOT_VALID_KIND]
    score = result.score
    assert score.status == "error"
    reason = "one-way carried-load result not valid for bar from 0 s (dynamic ratio 3.5)"
    assert score.error == reason
    assert any(line.endswith(f"not valid, {reason}") for line in score.crossings)
    results = {item.name: item for item in score.criteria}
    assert results["bar_safety_factor"].status == "error"
    assert results["bar_safety_factor"].note == f"safety_factor:bar: {reason}"
    with RunArchive(result.archive) as archive:
        stored = validity_from_json(archive.extra[VALIDITY_KEY])
    assert stored[0].flagged
    assert stored[0].since == 0.0
    table = json.loads(result.metadata.integrity.to_json())
    (row,) = [item for item in table["members"] if item["member"] == "bar"]
    assert NOT_VALID_FLAG in row["flags"]


@pytest.mark.req("INT-022", "SCO-029")
def test_a_criterion_before_the_flag_time_is_scored_and_one_inside_it_is_unevaluable(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _flag(monkeypatch, SINCE)
    case, services, driver = _setup(tmp_path, "1 m")
    result: Any = run_case(case, services, None, tmp_path / "runs")
    reason = f"one-way carried-load result not valid for bar from {SINCE:.12g} s (dynamic ratio 3.5)"
    results = {item.name: item for item in result.score.criteria}
    assert results["bar_deflection_early"].status == "pass"
    assert results["bar_deflection_late"].status == "error"
    assert results["bar_deflection_late"].note == f"deflection:bar: {reason}"
    assert results["bar_safety_factor"].status == "error"
    assert result.score.status == "error"
    assert not any(event.kind == NOT_VALID_KIND for event in driver.recording().events())
    report = result.metadata.integrity
    assert any(note.startswith(f"integrity: {reason}") for note in report.notes)
    for check in report.checks:
        for row in check.rows:
            if row.subject == "bar":
                assert NOT_VALID_FLAG in row.flags
                assert dict(row.detail)["not_valid_from"] == SINCE
                assert row.time <= SINCE
    times, values = report.deflection_series("bar")
    for time, value in zip(times, values, strict=True):
        assert math.isnan(value) == (time > SINCE)


# ===== Two members, one flagged (synthetic inputs) ==========================


def _two_members(rows: int) -> RunInputs:
    section = SectionProperties(1e-4, 70e9, 2700.0, 1.0, 8e-10, 8e-10, 0.005, 0.005)
    times = np.arange(rows, dtype=np.float64) * 1e-3
    out = []
    for name, scale in (("carrier", 1.0), ("post", 0.5)):
        moment = np.zeros((rows, 3))
        moment[:, 2] = scale * (1.0 + times * 100.0)
        zero = np.zeros((rows, 3))
        out.append(MemberInputs(name, "fake", section, ((0.0, "clamped"),), Wrenches(zero, moment, zero, zero.copy())))

    class Material:
        def strength_spread(self, kind: str, temperature: float, direction: Any = None, process: Any = None) -> Any:
            from mechsim.integrity.plugin import Spread

            return Spread(1e8, 1e8, 1e8)

        def sn_curve(self) -> tuple[tuple[float, float], ...]:
            return ((1e3, 1e8), (1e6, 4e7))

    return RunInputs(times, tuple(out), {"fake": Material()}, 293.15)


@pytest.mark.req("INT-022")
def test_the_flagged_members_rows_stand_before_the_flag_and_the_other_member_is_unaffected() -> None:
    config = parse_config({"schema_version": 1, "checks": [{"check": "strength"}, {"check": "fatigue"}]}, "test")
    run = _two_members(50)
    plain = evaluate(config, run)
    record = CarriedValidity("carrier", True, 0.0205, 4.0, 2.0)
    marked = evaluate(config, run, (), (record,))
    before = Strength().evaluate(run.until(0.0205), NoParameters(), 1)
    for kept, flagged in zip(plain.checks, marked.checks, strict=True):
        post = [row for row in kept.rows if row.subject == "post"]
        assert [row for row in flagged.rows if row.subject == "post"] == post
        for row in flagged.rows:
            if row.subject == "carrier":
                assert NOT_VALID_FLAG in row.flags
                assert dict(row.detail)["not_valid_from"] == 0.0205
    (strength,) = [result for result in marked.checks if result.check == "strength"]
    (strength_row,) = [row for row in strength.rows if row.subject == "carrier"]
    assert strength_row.value == before.rows[0].value
    assert strength_row.time == 0.020
    assert plain.member("post") == marked.member("post")
    assert NOT_VALID_FLAG in marked.member("carrier").flags
    unflagged = evaluate(config, run, (), (CarriedValidity("carrier", False, 0.0, 1.0, 2.0),))
    assert unflagged.report_json() == plain.report_json()


@pytest.mark.req("INT-022")
def test_a_flag_before_the_first_sample_leaves_nan_rows_never_zero() -> None:
    config = parse_config({"schema_version": 1, "checks": [{"check": "strength"}]}, "test")
    run = _two_members(5)
    marked = evaluate(config, run, (), (CarriedValidity("carrier", True, -1.0, 4.0, 2.0),))
    (strength,) = [result for result in marked.checks if result.check == "strength"]
    (row,) = [row for row in strength.rows if row.subject == "carrier"]
    assert math.isnan(row.value)
    assert NOT_VALID_FLAG in row.flags
    assert math.isnan(marked.member("carrier").peak_stress)


@pytest.mark.req("REC-004")
def test_the_validity_record_round_trips_and_refuses_non_finite_values() -> None:
    records = (CarriedValidity("a", True, 1.5, 2.5, 3.5), CarriedValidity("b", False, 0.0, 1.0, 0.25))
    assert validity_from_json(validity_json(records)) == records
    assert records[0].covers(1.6)
    assert not records[0].covers(1.5)
    assert not records[1].covers(9.0)
    with pytest.raises(ValueError, match="not finite"):
        validity_from_json('[{"flagged": true, "member": "a", "ratio": NaN, "since": 0, "work": 0}]')
