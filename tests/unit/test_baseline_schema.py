"""Unit tier: regression baseline file schema, deterministic writer and diff (SCO, ADR 0012 D4)."""

from __future__ import annotations

import copy
import functools
import io
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError
from ruamel.yaml import YAML

from mechsim.design.schema import yaml_loader
from mechsim.testing.baseline import (
    Baseline,
    BaselineError,
    Change,
    baseline_text,
    check_against_cases,
    diff_baselines,
    load_baseline,
    validate_baseline_file,
)
from mechsim.testing.schema import load_test_case

ROOT = Path(__file__).resolve().parents[2]
EXAMPLE = ROOT / "test_cases" / "gantry" / "baseline.yaml"
G3 = ROOT / "test_cases" / "gantry" / "G3.yaml"
RUN = {"run_id": "run-1", "archive_sha256": "a" * 64, "design_hash": "b" * 40, "case_hash": "c" * 40}


@functools.cache
def _raw() -> Any:
    return YAML(typ="safe").load(EXAMPLE.read_text(encoding="utf-8"))


def example() -> dict[str, Any]:
    return copy.deepcopy(_raw())


def entry(case: str, criterion: str, status: str = "not_run", **fields: Any) -> dict[str, Any]:
    item: dict[str, Any] = {"case": case, "criterion": criterion, "status": status, "limit": "peak < 1 m"}
    if status in ("pass", "fail"):
        item.update(measured="0.5 m", worst_time="1.5 s", run=copy.deepcopy(RUN))
    if status == "error":
        item["run"] = copy.deepcopy(RUN)
    item.update(fields)
    return item


def baseline(*entries: dict[str, Any], suite: str = "gantry") -> dict[str, Any]:
    return {"design": {"name": "gantry", "version": "0.1.0"}, "suite": suite, "entries": list(entries)}


def rejected(data: Any, *fragments: str) -> None:
    with pytest.raises(ValidationError) as caught:
        Baseline.model_validate(data)
    text = str(caught.value)
    for fragment in fragments:
        assert fragment in text, text


def accepted(data: Any) -> Baseline:
    return Baseline.model_validate(data)


# ===== Shipped example =================================================


@pytest.mark.req("SCO-009")
def test_example_baseline_validates_and_matches_g3() -> None:
    assert validate_baseline_file(EXAMPLE) == []
    loaded = load_baseline(EXAMPLE)
    assert {item.status for item in loaded.entries} == {"not_run"}
    assert check_against_cases(loaded, [load_test_case(G3)]) == []
    assert len(loaded.entries) == len(load_test_case(G3).criteria)


@pytest.mark.req("SCO-009")
def test_example_round_trips_and_writer_is_byte_identical() -> None:
    text = EXAMPLE.read_text(encoding="utf-8")
    loader = yaml_loader()
    stream = io.StringIO()
    loader.dump(loader.load(text), stream)
    assert stream.getvalue() == text
    assert baseline_text(load_baseline(EXAMPLE)) == text


@pytest.mark.req("SCO-011")
@pytest.mark.parametrize("worst_time", ["1.5 s", "250 ms", "0.1 s", "0 s", None])
def test_baseline_text_round_trips_through_load_baseline(tmp_path: Path, worst_time: str | None) -> None:
    item = entry("G3", "final_x", "pass", worst_time=worst_time)
    if worst_time is None:
        item.pop("worst_time")
    original = accepted(baseline(item, entry("G3", "final_y", "error"), entry("G4", "a")))
    path = tmp_path / "baseline.yaml"
    path.write_text(baseline_text(original), encoding="utf-8")
    assert load_baseline(path) == original


@pytest.mark.req("SCO-009")
def test_writer_sorts_entries_naturally_and_is_deterministic() -> None:
    data = baseline(entry("G3", "b"), entry("G10", "a"), entry("G3", "a"), entry("G2", "z"))
    rejected(data, "must be sorted")
    ordered = Baseline.model_construct(
        design=accepted(baseline(entry("G3", "a"))).design,
        suite="gantry",
        entries=[accepted(baseline(item)).entries[0] for item in data["entries"]],
    )
    text = baseline_text(ordered)
    reloaded = accepted(YAML(typ="safe").load(text))
    assert [(item.case, item.criterion) for item in reloaded.entries] == [
        ("G2", "z"),
        ("G3", "a"),
        ("G3", "b"),
        ("G10", "a"),
    ]
    assert baseline_text(reloaded) == text


# ===== Fields ==========================================================


@pytest.mark.req("SCO-009")
@pytest.mark.parametrize("field", ["design", "suite", "entries"])
def test_required_root_field_missing_rejected(field: str) -> None:
    data = example()
    del data[field]
    rejected(data, field, "Field required")


@pytest.mark.req("SCO-009")
@pytest.mark.parametrize("field", ["case", "criterion", "status", "limit"])
def test_required_entry_field_missing_rejected(field: str) -> None:
    data = example()
    del data["entries"][0][field]
    rejected(data, field, "Field required")


@pytest.mark.req("SCO-009")
@pytest.mark.parametrize("field", ["run_id", "archive_sha256", "design_hash", "case_hash"])
def test_required_run_field_missing_rejected(field: str) -> None:
    item = entry("G3", "a", "pass")
    del item["run"][field]
    rejected(baseline(item), field, "Field required")


@pytest.mark.req("SCO-009")
def test_empty_entries_rejected() -> None:
    rejected(baseline(), "entries", "at least 1")


@pytest.mark.req("SCO-009")
@pytest.mark.parametrize("where", ["root", "design", "entry", "run"])
def test_unknown_key_rejected(where: str) -> None:
    data = baseline(entry("G3", "a", "pass"))
    targets = {"root": data, "design": data["design"], "entry": data["entries"][0], "run": data["entries"][0]["run"]}
    targets[where]["archive"] = "run.h5"
    rejected(data, "archive", "Extra inputs are not permitted")


@pytest.mark.req("SCO-009")
@pytest.mark.parametrize("status", ["pass", "fail", "not_run", "error"])
def test_each_status_accepted(status: str) -> None:
    assert accepted(baseline(entry("G3", "a", status))).entries[0].status == status


@pytest.mark.req("SCO-009")
@pytest.mark.parametrize(
    ("item", "fragment"),
    [
        (entry("G3", "a", "passed"), "status"),
        (entry("G3", "a", "pass", measured=None), "needs measured and run"),
        (entry("G3", "a", "fail", run=None), "needs measured and run"),
        (entry("G3", "a", "not_run", measured="1 m"), "takes no measured or worst_time"),
        (entry("G3", "a", "not_run", worst_time="1 s"), "takes no measured or worst_time"),
        (entry("G3", "a", "not_run", run=RUN), "takes no run"),
        (entry("G3", "a", "error", measured="1 m"), "takes no measured or worst_time"),
        (entry("G3", "a", "error", run=None), "needs the run that errored"),
        (entry("G3", "a", "pass", worst_time="-1 s"), "must not be negative"),
        (entry("G3", "a", "pass", worst_time="1 m"), "expected [time]"),
        (entry("G3", "a", "pass", worst_time=1.0), "worst_time"),
        (entry("G3", "a", "pass", measured=0.5), "must be a string with a unit"),
        (entry("G3", "a", "pass", measured="m"), "must start with a number"),
        (entry("g3", "a"), "case"),
        (entry("G3", "A"), "criterion"),
        (entry("G3", "a", limit=""), "at least 1 character"),
        (entry("G3", "a", "pass", run={**RUN, "archive_sha256": "a" * 63}), "archive_sha256"),
        (entry("G3", "a", "pass", run={**RUN, "design_hash": "B" * 40}), "design_hash"),
        (entry("G3", "a", "pass", run={**RUN, "case_hash": "c" * 41}), "case_hash"),
        (entry("G3", "a", "pass", run={**RUN, "run_id": ""}), "run_id"),
    ],
)
def test_bad_entry_rejected(item: dict[str, Any], fragment: str) -> None:
    rejected(baseline(item), fragment)


@pytest.mark.req("SCO-009")
def test_pass_entry_values_parsed_to_si() -> None:
    parsed = accepted(baseline(entry("G3", "a", "pass", worst_time="1500 ms"))).entries[0]
    assert parsed.measured is not None
    assert parsed.measured.value == pytest.approx(0.5)
    assert parsed.worst_time == pytest.approx(1.5)
    assert parsed.run is not None
    assert parsed.run.run_id == "run-1"


@pytest.mark.req("SCO-009")
def test_pass_entry_without_worst_time_accepted() -> None:
    assert accepted(baseline(entry("G3", "a", "pass", worst_time=None))).entries[0].worst_time is None


@pytest.mark.req("SCO-009")
def test_duplicate_entry_rejected() -> None:
    rejected(baseline(entry("G3", "a"), entry("G3", "a", "fail")), "duplicate entry G3/a")


@pytest.mark.req("SCO-009")
def test_check_against_cases_reports_unknown_entries() -> None:
    case = load_test_case(G3)
    data = example()
    data["entries"].append(entry("G4", "a"))
    errors = check_against_cases(accepted(data), [case])
    assert errors == ["entries[8].case: no case 'G4' in the suite"]
    data = example()
    data["entries"][0]["criterion"] = "final_w"
    data["entries"].sort(key=lambda item: item["criterion"])
    assert any("has no criterion 'final_w'" in error for error in check_against_cases(accepted(data), [case]))
    data = example()
    data["entries"][0]["limit"] = "final within 2 cm of ${target_x}"
    assert any("differs from the case's" in error for error in check_against_cases(accepted(data), [case]))
    data = example()
    data["design"]["version"] = "0.2.0"
    assert any("another design version" in error for error in check_against_cases(accepted(data), [case]))


@pytest.mark.req("SCO-009")
def test_file_errors_name_the_file(tmp_path: Path) -> None:
    path = tmp_path / "baseline.yaml"
    path.write_text("suite: gantry\n", encoding="utf-8")
    errors = validate_baseline_file(path)
    assert any(error.startswith(f"{path}: design:") for error in errors)
    with pytest.raises(BaselineError, match="entries"):
        load_baseline(path)
    path.write_text("suite: [", encoding="utf-8")
    assert "YAML parse error" in validate_baseline_file(path)[0]


# ===== Diff ============================================================


def diff_of(old: list[dict[str, Any]], new: list[dict[str, Any]]) -> Any:
    return diff_baselines(accepted(baseline(*old)), accepted(baseline(*new)))


@pytest.mark.req("SCO-009", "SCO-010")
@pytest.mark.parametrize("before", ["fail", "error", "not_run"])
def test_diff_newly_passing(before: str) -> None:
    result = diff_of([entry("G3", "a", before)], [entry("G3", "a", "pass")])
    assert result.newly_passing == (Change("G3", "a", before, "pass"),)
    assert result.newly_failing == result.unchanged == result.missing == result.added == ()


@pytest.mark.req("SCO-009", "SCO-010")
@pytest.mark.parametrize("after", ["fail", "error", "not_run"])
def test_diff_newly_failing(after: str) -> None:
    result = diff_of([entry("G3", "a", "pass")], [entry("G3", "a", after)])
    assert result.newly_failing == (Change("G3", "a", "pass", after),)
    assert result.newly_passing == result.unchanged == result.missing == result.added == ()


@pytest.mark.req("SCO-009", "SCO-010")
@pytest.mark.parametrize(
    ("before", "after"),
    [("pass", "pass"), ("fail", "fail"), ("fail", "error"), ("not_run", "fail"), ("error", "not_run")],
)
def test_diff_unchanged(before: str, after: str) -> None:
    result = diff_of([entry("G3", "a", before)], [entry("G3", "a", after)])
    assert result.unchanged == (Change("G3", "a", before, after),)
    assert result.newly_passing == result.newly_failing == result.missing == result.added == ()


@pytest.mark.req("SCO-009", "SCO-010")
def test_diff_missing_and_added() -> None:
    result = diff_of([entry("G3", "a", "pass"), entry("G3", "b", "fail")], [entry("G3", "b", "fail"), entry("G4", "a")])
    assert result.missing == (Change("G3", "a", "pass", None),)
    assert result.added == (Change("G4", "a", None, "not_run"),)
    assert result.unchanged == (Change("G3", "b", "fail", "fail"),)


@pytest.mark.req("SCO-009", "SCO-010")
def test_diff_is_sorted_naturally_and_identical_for_identical_input() -> None:
    old = [entry("G2", "a", "fail"), entry("G3", "a", "fail"), entry("G10", "a", "fail")]
    new = [entry("G2", "a", "pass"), entry("G3", "a", "pass"), entry("G10", "a", "pass")]
    result = diff_of(old, new)
    assert [change.case for change in result.newly_passing] == ["G2", "G3", "G10"]
    assert diff_of(old, new) == result


@pytest.mark.req("SCO-009")
def test_diff_of_different_suites_rejected() -> None:
    with pytest.raises(ValueError, match="cannot diff suite 'other'"):
        diff_baselines(accepted(baseline(entry("G3", "a"))), accepted(baseline(entry("G3", "a"), suite="other")))
