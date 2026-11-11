"""Scoring (SCO): statistics on synthetic series, criteria, case and suite aggregation, baselines, the text report."""

from __future__ import annotations

import math
from fractions import Fraction
from typing import Any

import pytest

from mechsim.scoring.score import (
    CaseScore,
    ScoringError,
    SuiteScore,
    diff_sections,
    not_run,
    regression_diff,
    report_markdown,
    report_text,
    score_run,
    suite_baseline,
)
from mechsim.scoring.statistics import (
    StatisticError,
    compare,
    statistic,
    upward_crossings,
    window_indices,
    within,
)
from mechsim.testing.baseline import RunIdentity, baseline_text, diff_baselines, load_baseline
from mechsim.testing.driver import MESSAGE_KIND, RecordedEvent, RunReport
from mechsim.testing.schema import Case, DesignRef
from tests.unit.test_runner_fakes import FakeRecording, Row, recording_from

ENDED = RunReport("end_time", 3, Fraction(1))
IDENTITY = RunIdentity(run_id="X1-seed0", archive_sha256="a" * 64, design_hash="b" * 40, case_hash="c" * 40)
TIMES = [0.0, 1.0, 2.0, 3.0]
SAMPLES = [0.0, 3.0, -5.0, 2.0]


def case_with(*criteria: dict[str, Any], **fields: Any) -> Case:
    data: dict[str, Any] = {
        "id": "X1",
        "title": "x",
        "design": {"name": "machine", "version": "0.1.0"},
        "controller_mode": "native",
        "seed": 11,
        "initial_state": {},
        "end": {"at": "3 s", "max_duration": "3 s"},
        "criteria": list(criteria),
    }
    data.update(fields)
    return Case.model_validate(data)


def value(statistic_name: str, comparison: str, limit: str, **extra: Any) -> dict[str, Any]:
    return {
        "name": extra.pop("name", "c"),
        "check": "value",
        "quantity": extra.pop("quantity", "signal:s.v"),
        "statistic": statistic_name,
        "comparison": comparison,
        "limit": limit,
        **extra,
    }


def score_one(criterion: dict[str, Any], recording: Any, report: RunReport = ENDED) -> Any:
    return score_run(case_with(criterion), recording, report).criteria[0]


# ===== Statistics =======================================================


@pytest.mark.req("SCO-020")
@pytest.mark.parametrize(
    ("name", "expected"),
    [("final", (2.0, 3.0)), ("minimum", (-5.0, 2.0)), ("maximum", (3.0, 1.0)), ("peak_absolute", (5.0, 2.0))],
)
def test_each_statistic_matches_the_hand_computed_value_and_worst_time(
    name: Any, expected: tuple[float, float]
) -> None:
    assert statistic(name, TIMES, SAMPLES) == expected


@pytest.mark.req("SCO-020")
def test_ties_report_the_first_worst_sample() -> None:
    assert statistic("maximum", [0.0, 1.0, 2.0], [1.0, 4.0, 4.0]) == (4.0, 1.0)
    assert statistic("minimum", [0.0, 1.0, 2.0], [-1.0, 0.0, -1.0]) == (-1.0, 0.0)
    assert statistic("peak_absolute", [0.0, 1.0], [-2.0, 2.0]) == (2.0, 0.0)


@pytest.mark.req("SCO-020")
def test_a_statistic_of_no_samples_is_an_error() -> None:
    with pytest.raises(StatisticError, match="no samples"):
        statistic("final", [], [])


@pytest.mark.req("SCO-020")
def test_the_window_includes_both_bounds() -> None:
    assert window_indices(TIMES, 1.0, 2.0) == [1, 2]
    assert window_indices(TIMES, None, 1.0) == [0, 1]
    assert window_indices(TIMES, 2.5, None) == [3]
    assert window_indices(TIMES, 3.5, None) == []


@pytest.mark.req("SCO-020")
def test_the_period_of_a_sampled_sine_by_zero_crossings() -> None:
    # x(t) = sin(w (t - t0)) sampled at h; the root of the chord through samples a before and b after a crossing
    # (a + b = h) misses the true crossing by e = w^2 a b (b - a) / 6 + O(h^5) (Taylor of sin to third order; the
    # second-order term vanishes at an inflection). |a b (b - a)| is largest at b - a = h / sqrt(3), giving
    # |e| <= w^2 h^3 / (36 sqrt(3)). The period estimate (t_last - t_first) / (N - 1) is off by at most 2 |e| / (N - 1);
    # float rounding of times near 25 s adds under 1e-12 s.
    period = 1.1596536
    omega = 2.0 * math.pi / period
    step = 0.001
    t0 = 0.0003
    times = [n * step for n in range(24401)]
    values = [math.sin(omega * (t - t0)) for t in times]
    crossings = upward_crossings(times, values)
    count = len(crossings)
    bound = 2.0 * omega**2 * step**3 / (36.0 * math.sqrt(3.0)) / (count - 1) + 1e-12
    measured, worst = statistic("period", times, values)
    print(f"closed form {period} s, measured {measured!r} s, {count} crossings, bound {bound:.3e} s")
    # Crossings at t0 + k T for t0 + k T <= 24.4 s: k = 0 .. floor((24.4 - t0) / T) = 21, so 22 of them.
    assert count == math.floor((times[-1] - t0) / period) + 1 == 22
    assert abs(measured - period) <= bound
    assert worst == crossings[-1]


@pytest.mark.req("SCO-020")
def test_a_sample_exactly_at_zero_counts_once() -> None:
    assert upward_crossings([0.0, 1.0, 2.0, 3.0], [-1.0, 0.0, 1.0, -1.0]) == [1.0]
    assert upward_crossings([0.0, 1.0], [-1.0, 3.0]) == [0.25]
    assert upward_crossings([0.0, 1.0], [1.0, -1.0]) == []


@pytest.mark.req("SCO-020")
@pytest.mark.parametrize("values", [[1.0, 2.0, 3.0], [-1.0, 1.0, 2.0]])
def test_a_period_needs_two_upward_crossings(values: list[float]) -> None:
    with pytest.raises(StatisticError, match="at least 2 upward zero crossings"):
        statistic("period", [0.0, 1.0, 2.0], values)


@pytest.mark.parametrize(
    ("comparison", "below", "equal", "above"),
    [
        ("below", True, False, False),
        ("at_most", True, True, False),
        ("above", False, False, True),
        ("at_least", False, True, True),
    ],
)
def test_comparisons_name_their_inclusive_and_strict_forms(
    comparison: str, below: bool, equal: bool, above: bool
) -> None:
    assert (compare(0.5, comparison, 1.0), compare(1.0, comparison, 1.0), compare(1.5, comparison, 1.0)) == (
        below,
        equal,
        above,
    )
    with pytest.raises(ValueError, match="unknown comparison"):
        compare(1.0, "near", 1.0)


def test_within_passes_an_error_equal_to_the_tolerance() -> None:
    assert within(1.25, 1.0, 0.25)
    assert within(0.75, 1.0, 0.25)
    assert not within(1.2500001, 1.0, 0.25)


# ===== Criteria =========================================================


@pytest.mark.req("SCO-001")
@pytest.mark.req("SCO-002")
@pytest.mark.req("SCO-003")
@pytest.mark.req("SCO-004")
def test_a_value_criterion_reports_measured_beside_limit_with_the_worst_time() -> None:
    recording = recording_from(TIMES, {"signal:s.v": SAMPLES}, "V")
    passed = score_one(value("maximum", "at_most", "3 V"), recording)
    failed = score_one(value("maximum", "below", "3 V"), recording)
    assert (passed.status, passed.measured, passed.unit, passed.worst_time) == ("pass", 3.0, "V", 1.0)
    assert passed.limit == "maximum at_most 3 V"
    assert passed.measured_text() == "3 V"
    assert failed.status == "fail"


@pytest.mark.req("SCO-020")
def test_a_value_criterion_uses_its_window() -> None:
    recording = recording_from(TIMES, {"signal:s.v": SAMPLES}, "V")
    result = score_one(value("minimum", "at_least", "0 V", window={"end": "1 s"}), recording)
    assert (result.status, result.measured) == ("pass", 0.0)
    empty = score_one(value("minimum", "at_least", "0 V", window={"start": "5 s"}), recording)
    assert (empty.status, empty.note) == ("error", "no samples in the window")


@pytest.mark.req("SCO-020")
def test_relative_and_absolute_within_limits() -> None:
    recording = recording_from(TIMES, {"signal:s.v": SAMPLES}, "V")
    absolute = {"target": "2.5 V", "limit": "0.5 V", "comparison": "within"}
    assert score_one(value("final", **absolute), recording).status == "pass"
    criterion = value("final", "within", "x", target="2.5 V", relative_limit="10 percent")
    criterion.pop("limit")
    assert score_one(criterion, recording).status == "fail"
    criterion["relative_limit"] = "20 percent"
    assert score_one(criterion, recording).status == "pass"


@pytest.mark.req("SCO-020")
def test_a_period_criterion_on_a_recorded_series() -> None:
    times = [n * 0.01 for n in range(401)]
    values = [math.sin(2.0 * math.pi * (t - 0.003)) for t in times]
    criterion = value("period", "within", "x", quantity="joint:j.position", target="1 s", relative_limit="0.1 percent")
    criterion.pop("limit")
    result = score_one(criterion, recording_from(times, {"joint:j.position": values}, "rad"))
    assert (result.status, result.unit) == ("pass", "s")
    flat = score_one(criterion, recording_from(times, {"joint:j.position": [1.0] * len(times)}, "rad"))
    assert flat.status == "error"


def test_per_axis_criteria_pass_a_planar_error_each_axis_admits() -> None:
    # Owner ruling 2026-11-04: x and y each within 1 cm; 0.99 cm on both axes (1.40 cm planar) passes.
    recording = recording_from([0.0, 1.0], {"pose:hook.x": [0.0, 0.4099], "pose:hook.y": [0.0, 0.2599]}, "m")
    criteria = [
        value("final", "within", "1 cm", name="final_x", quantity="pose:hook.x", target="40 cm"),
        value("final", "within", "1 cm", name="final_y", quantity="pose:hook.y", target="25 cm"),
    ]
    score = score_run(case_with(*criteria), recording, ENDED)
    assert [result.status for result in score.criteria] == ["pass", "pass"]
    off = recording_from([0.0, 1.0], {"pose:hook.x": [0.0, 0.411], "pose:hook.y": [0.0, 0.25]}, "m")
    score = score_run(case_with(*criteria), off, ENDED)
    assert [result.status for result in score.criteria] == ["fail", "pass"]
    assert score.status == "fail"


def _events_recording() -> FakeRecording:
    rows = [
        Row({}, "idle"),
        Row({}, "homing", [RecordedEvent(1.0, "operator", MESSAGE_KIND, "home")]),
        Row({}, "ready", [RecordedEvent(2.0, "controller", "safety", "home")]),
        Row({}, "homing", [RecordedEvent(3.0, "operator", MESSAGE_KIND, "home")]),
    ]
    return FakeRecording(TIMES, rows, {})


def test_count_references_count_messages_and_state_entries() -> None:
    messages = score_one(value("final", "at_least", "2", quantity="count:message.home"), _events_recording())
    entries = score_one(value("final", "at_most", "1", quantity="count:state.homing"), _events_recording())
    assert (messages.status, messages.measured) == ("pass", 2.0)
    assert (entries.status, entries.measured) == ("fail", 2.0)


@pytest.mark.req("SCO-022")
@pytest.mark.parametrize(
    ("event", "expect", "window", "status", "count"),
    [
        ("message:home", "present", None, "pass", 1.0 + 1.0),
        ("message:home", "present", {"start": "1.5 s", "end": "2.5 s"}, "fail", 0.0),
        ("message:home", "absent", {"end": "0.5 s"}, "pass", 0.0),
        ("message:home", "absent", None, "fail", 2.0),
        ("state:ready", "present", {"start": "2 s"}, "pass", 1.0),
        ("state:idle", "present", None, "pass", 1.0),
        ("state:fault", "absent", None, "pass", 0.0),
    ],
)
def test_event_presence_and_absence_within_a_window(
    event: str, expect: str, window: Any, status: str, count: float
) -> None:
    criterion: dict[str, Any] = {"name": "e", "check": "event", "event": event, "expect": expect}
    if window is not None:
        criterion["window"] = window
    result = score_one(criterion, _events_recording())
    assert (result.status, result.measured) == (status, count)


@pytest.mark.req("SCO-021")
def test_the_interval_between_two_events() -> None:
    criterion = {
        "name": "i",
        "check": "interval",
        "start_event": "message:home",
        "end_event": "state:ready",
        "comparison": "at_most",
        "limit": "1 s",
    }
    result = score_one(criterion, _events_recording())
    assert (result.status, result.measured, result.unit, result.worst_time) == ("pass", 1.0, "s", 2.0)
    criterion["comparison"] = "below"
    assert score_one(criterion, _events_recording()).status == "fail"


@pytest.mark.req("SCO-021")
@pytest.mark.parametrize(
    ("start", "end", "note"),
    [
        ("message:target", "state:ready", "start event message:target never recorded"),
        ("message:home", "state:fault", "end event state:fault never recorded"),
        (
            "state:ready",
            "event:safety_early",
            "end event event:safety_early recorded only before the first start event state:ready at 2 s",
        ),
    ],
    ids=["missing_first", "missing_second", "second_before_first"],
)
def test_an_interval_that_cannot_be_measured_is_an_error_naming_the_ref(start: str, end: str, note: str) -> None:
    rows = [
        Row({}, "idle", [RecordedEvent(0.0, "controller", "safety_early", "x")]),
        Row({}, "homing", [RecordedEvent(1.0, "operator", MESSAGE_KIND, "home")]),
        Row({}, "ready"),
        Row({}, "ready"),
    ]
    criterion = {
        "name": "i",
        "check": "interval",
        "start_event": start,
        "end_event": end,
        "comparison": "at_most",
        "limit": "1 s",
    }
    result = score_one(criterion, FakeRecording(TIMES, rows, {}))
    assert (result.status, result.note) == ("error", note)


@pytest.mark.req("SCO-021")
def test_an_interval_between_firmware_events() -> None:
    # motion_start at 1 s, motion_end at 1 s and 3 s: the first end at or after the first start is at 1 s, so 0 s.
    rows = [
        Row({}),
        Row({}, "", [RecordedEvent(1.0, "controller", "motion_start", "x_motor")]),
        Row({}, "", [RecordedEvent(1.0, "controller", "motion_end", "x_motor")]),
        Row({}, "", [RecordedEvent(3.0, "controller", "motion_end", "x_motor")]),
    ]
    criterion = {
        "name": "i",
        "check": "interval",
        "start_event": "event:motion_start",
        "end_event": "event:motion_end",
        "comparison": "at_most",
        "limit": "0 s",
    }
    result = score_one(criterion, FakeRecording(TIMES, rows, {}))
    assert (result.status, result.measured, result.worst_time) == ("pass", 0.0, 1.0)


def _settle(values: list[float], comparison: str = "at_most", limit: str = "2 s", **extra: Any) -> Any:
    criterion = {
        "name": "s",
        "check": "settle_time",
        "quantity": "signal:s.v",
        "band": "0.1 V",
        "comparison": comparison,
        "limit": limit,
        **extra,
    }
    times = [float(n) for n in range(len(values))]
    return score_one(criterion, recording_from(times, {"signal:s.v": values}, "V"))


@pytest.mark.req("SCO-024")
def test_settle_time_into_a_band_about_the_final_value_or_a_target() -> None:
    # Final value 1.0, band 0.1: the last sample out of band is 1.2 at 2 s, so it settles at 3 s.
    result = _settle([0.0, 0.5, 1.2, 0.95, 1.0, 1.0], "at_most", "3 s")
    assert (result.status, result.measured, result.worst_time) == ("pass", 3.0, 2.0)
    assert _settle([0.0, 0.5, 1.2, 0.95, 1.0, 1.0], "below", "3 s").status == "fail"
    already = _settle([1.0, 1.0, 1.0])
    assert (already.measured, already.worst_time) == (0.0, 0.0)
    never = _settle([1.0, 1.0, 2.0], target="1 V")
    assert (never.status, never.note) == ("fail", "did not settle in the window")


@pytest.mark.req("SCO-023")
def test_no_motion_against_the_threshold() -> None:
    criterion = {
        "name": "m",
        "check": "no_motion",
        "actuators": ["a", "b"],
        "threshold": "0.01 rad",
        "window": {"start": "1 s"},
    }
    recording = recording_from(
        TIMES, {"actuator:a.position": [5.0, 1.0, 1.005, 1.009], "actuator:b.position": [0.0, 0.0, 0.0, -0.002]}, "rad"
    )
    result = score_one(criterion, recording)
    assert result.status == "pass"
    assert result.measured == pytest.approx(0.009)
    assert result.worst_time == 3.0
    moved = recording_from(
        TIMES, {"actuator:a.position": [0.0, 0.0, 0.0, 0.0], "actuator:b.position": [0.0, 0.0, 0.02, 0.0]}, "rad"
    )
    result = score_one(criterion, moved)
    assert (result.status, result.measured, result.worst_time) == ("fail", 0.02, 2.0)


@pytest.mark.req("SCO-026")
def test_hold_band_measures_each_joints_excursion_from_its_first_sample() -> None:
    criterion = {
        "name": "h",
        "check": "hold_band",
        "joints": ["x", "lift"],
        "band": "1 mm",
        "window": {"start": "1 s"},
    }
    # In the window (t >= 1 s) x starts at 0.2 and sways 0.4 mm; lift sags 0.9 mm from 0.1: worst 0.9 mm at 3 s.
    held = recording_from(
        TIMES, {"joint:x.position": [5.0, 0.2, 0.2004, 0.1999], "joint:lift.position": [0.0, 0.1, 0.1, 0.0991]}, "m"
    )
    result = score_one(criterion, held)
    assert result.status == "pass"
    assert result.measured == pytest.approx(0.0009)
    assert result.worst_time == 3.0
    assert result.limit == "every joint within 1 mm of its first sample"
    moved = recording_from(
        TIMES, {"joint:x.position": [0.0, 0.0, 0.0011, 0.0], "joint:lift.position": [0.0, 0.0, 0.0, 0.0]}, "m"
    )
    result = score_one(criterion, moved)
    assert (result.status, result.worst_time) == ("fail", 2.0)
    assert result.measured == pytest.approx(0.0011)


def _motion(*events: tuple[float, str, str]) -> FakeRecording:
    """Rows at TIMES; each (time, kind, actuator) event lands in its row in the order given."""
    rows = [Row({}) for _ in TIMES]
    for time, kind, actuator in events:
        rows[TIMES.index(time)].events.append(RecordedEvent(time, "controller", kind, actuator))
    return FakeRecording(TIMES, rows, {})


def _no_overlap(recording: FakeRecording, closed: bool = False) -> Any:
    criterion = {
        "name": "o",
        "check": "no_overlap",
        "actuator": "a",
        "start_event": "event:motion_start",
        "end_event": "event:motion_end",
        "closed": closed,
    }
    return score_one(criterion, recording)


@pytest.mark.req("SCO-025")
def test_no_overlap_passes_when_every_start_follows_the_previous_end() -> None:
    # Pairs (0, 1) and (1, 2) on a: the second start is recorded after the end at the same time, so not before it.
    # b's start at 1 s lies inside a's pair but is another actuator.
    recording = _motion(
        (0.0, "motion_start", "a"),
        (1.0, "motion_end", "a"),
        (1.0, "motion_start", "b"),
        (1.0, "motion_start", "a"),
        (2.0, "motion_end", "a"),
    )
    result = _no_overlap(recording, closed=True)
    assert (result.status, result.measured, result.worst_time) == ("pass", 0.0, None)
    assert result.limit == "no overlap of event:motion_start to event:motion_end on a, closed at the end"


@pytest.mark.req("SCO-025")
@pytest.mark.parametrize(
    ("events", "worst"),
    [
        (
            [
                (0.0, "motion_start", "a"),
                (1.0, "motion_start", "a"),
                (2.0, "motion_end", "a"),
                (3.0, "motion_end", "a"),
            ],
            1.0,
        ),
        (
            [
                (0.0, "motion_start", "a"),
                (2.0, "motion_start", "a"),
                (2.0, "motion_end", "a"),
                (3.0, "motion_end", "a"),
            ],
            2.0,
        ),
    ],
    ids=["start_before_previous_end", "start_recorded_before_the_end_at_the_same_time"],
)
def test_a_start_before_the_previous_end_on_the_same_actuator_fails(
    events: list[tuple[float, str, str]], worst: float
) -> None:
    result = _no_overlap(_motion(*events))
    assert (result.status, result.measured, result.worst_time) == ("fail", 1.0, worst)


@pytest.mark.req("SCO-025")
def test_an_unpaired_start_at_the_end_fails_only_when_the_case_says_closed() -> None:
    recording = _motion((0.0, "motion_start", "a"), (1.0, "motion_end", "a"), (2.0, "motion_start", "a"))
    assert _no_overlap(recording).status == "pass"
    closed = _no_overlap(recording, closed=True)
    assert (closed.status, closed.measured, closed.worst_time) == ("fail", 1.0, 3.0)


@pytest.mark.req("SCO-025")
@pytest.mark.parametrize(
    ("events", "note"),
    [
        ([(0.0, "motion_start", "b"), (1.0, "motion_end", "b")], "start event event:motion_start never recorded on a"),
        (
            [(0.0, "motion_end", "a"), (1.0, "motion_start", "a")],
            "end event event:motion_end on a at 0 s has no open start event event:motion_start",
        ),
    ],
    ids=["no_start", "end_without_start"],
)
def test_no_overlap_without_its_pairing_is_an_error_naming_the_ref(
    events: list[tuple[float, str, str]], note: str
) -> None:
    result = _no_overlap(_motion(*events))
    assert (result.status, result.note) == ("error", note)


@pytest.mark.req("SCO-025")
def test_the_event_criterion_counts_firmware_events_by_kind() -> None:
    recording = _motion((0.0, "motion_start", "a"), (2.0, "motion_start", "b"))
    result = score_one({"name": "e", "check": "event", "event": "event:motion_start", "expect": "present"}, recording)
    assert (result.status, result.measured, result.worst_time) == ("pass", 2.0, 0.0)


def test_a_series_the_recording_lacks_is_an_error() -> None:
    result = score_one(value("final", "at_most", "1 V"), recording_from(TIMES, {"signal:other.v": SAMPLES}))
    assert (result.status, result.note) == ("error", "not recorded: signal:s.v")


def test_an_unresolved_limit_is_an_error() -> None:
    criterion = value("final", "at_most", "${limit_v}")
    case = case_with(
        criterion, open_parameters=[{"name": "limit_v", "dimension": "[length]", "owed_by": "t", "note": "n"}]
    )
    result = score_run(case, recording_from(TIMES, {"signal:s.v": SAMPLES}), ENDED).criteria[0]
    assert (result.status, result.note) == ("error", "limit ${limit_v} is unresolved")


# ===== Cases and errored runs ===========================================


@pytest.mark.req("SCO-005")
def test_a_case_passes_only_when_every_criterion_passes() -> None:
    recording = recording_from(TIMES, {"signal:s.v": SAMPLES}, "V")
    good = value("maximum", "at_most", "3 V", name="a")
    bad = value("minimum", "above", "0 V", name="b")
    broken = value("final", "at_most", "1 V", name="d", quantity="signal:x.v")
    assert score_run(case_with(good), recording, ENDED).status == "pass"
    assert score_run(case_with(good, bad), recording, ENDED).status == "fail"
    assert score_run(case_with(good, bad, broken), recording, ENDED).status == "error"


@pytest.mark.req("SCO-007")
def test_a_run_error_is_errored_never_passed() -> None:
    recording = recording_from([0.0], {"signal:s.v": [0.0]}, "V")
    report = RunReport("run_error", 2, Fraction(1000), "controller: emulator fault")
    score = score_run(case_with(value("final", "at_most", "1 V")), recording, report)
    assert score.status == "error"
    assert (score.end_reason, score.end_time, score.error) == ("run_error", 0.002, "controller: emulator fault")
    assert score.criteria[0].status == "error"
    assert score.criteria[0].note == "run error: controller: emulator fault"


@pytest.mark.req("SCO-017")
def test_criteria_come_from_the_case_with_any_name_and_measure() -> None:
    recording = recording_from(TIMES, {"signal:widget.level": SAMPLES, "pose:arm.yaw": [0.1] * 4}, "")
    case = case_with(
        value("peak_absolute", "at_most", "5", name="anything_at_all", quantity="signal:widget.level"),
        value("final", "within", "0.01 rad", name="yaw", quantity="pose:arm.yaw", target="0.1 rad"),
    )
    score = score_run(case, recording, ENDED)
    assert [(result.name, result.status) for result in score.criteria] == [("anything_at_all", "pass"), ("yaw", "pass")]


# ===== Suites, report, baselines ========================================


def _suite(*cases: CaseScore) -> SuiteScore:
    return SuiteScore("demo", DesignRef(name="machine", version="0.1.0"), cases)


def _scored(name: str, maximum: str, run: RunIdentity | None = IDENTITY, case_id: str = "X1") -> CaseScore:
    recording = recording_from(TIMES, {"signal:s.v": SAMPLES}, "V")
    case = case_with(value("maximum", "at_most", maximum, name=name), id=case_id)
    return score_run(case, recording, ENDED, run=run)


@pytest.mark.req("SCO-006")
@pytest.mark.req("SCO-008")
def test_a_suite_passes_only_when_every_case_passes() -> None:
    passing = _scored("a", "3 V")
    failing = _scored("a", "2 V", case_id="X2")
    errored = not_run(None, "X3", DesignRef(name="machine", version="0.1.0"), 0, "invalid")
    assert _suite(passing).status == "pass"
    assert _suite(passing, failing).status == "fail"
    assert _suite(passing, failing, errored).status == "error"
    assert _suite().status == "fail"
    text = report_text(_suite(passing, failing, errored))
    assert text.startswith("Suite demo (machine 0.1.0): ERROR\n3 runs: 1 passed, 1 failed, 1 errored\n")


@pytest.mark.req("SCO-013")
def test_the_report_states_seed_design_version_and_mode_per_case() -> None:
    text = report_text(_suite(_scored("a", "3 V")))
    assert "Case X1: PASS\n  seed 11, design machine 0.1.0, controller mode native\n  ended: end_time at 3 s\n" in text
    assert "  pass  a: measured 3 V; limit maximum at_most 3 V; worst at 1 s\n" in text


@pytest.mark.req("SCO-016")
def test_scoring_the_same_recording_twice_gives_an_identical_report() -> None:
    first = report_text(_suite(_scored("a", "3 V"), _scored("b", "1 V", case_id="X2")))
    second = report_text(_suite(_scored("a", "3 V"), _scored("b", "1 V", case_id="X2")))
    assert first == second


@pytest.mark.req("SCO-018")
def test_sweep_runs_are_reported_side_by_side() -> None:
    first = _scored("a", "3 V")
    second = _scored("a", "2 V")
    runs = [
        CaseScore(**{**first.__dict__, "sweep": ("${gain}", "1")}),
        CaseScore(**{**second.__dict__, "sweep": ("${gain}", "2")}),
    ]
    text = report_text(_suite(*runs))
    assert "Sweep X1 over ${gain}\n| criterion | 1 | 2 |\n|---|---|---|\n| a | 3 V (pass) | 3 V (fail) |\n" in text
    assert "Case X1 [${gain} = 2]: FAIL" in text


@pytest.mark.req("SCO-011")
def test_a_suite_summary_is_written_and_read_back_as_a_baseline(tmp_path: Any) -> None:
    suite = _suite(_scored("b", "3 V"), _scored("a", "2 V", case_id="X10"), _scored("c", "3 V", case_id="X2"))
    baseline = suite_baseline(suite)
    assert [(entry.case, entry.criterion, entry.status) for entry in baseline.entries] == [
        ("X1", "b", "pass"),
        ("X2", "c", "pass"),
        ("X10", "a", "fail"),
    ]
    path = tmp_path / "baseline.yaml"
    path.write_text(baseline_text(baseline), encoding="utf-8")
    loaded = load_baseline(path)
    assert loaded == baseline
    assert str(loaded.entries[0].measured) == "3 V"
    assert loaded.entries[0].worst_time == 1.0
    diff = diff_baselines(loaded, suite_baseline(_suite(_scored("b", "2 V"), _scored("a", "3 V", case_id="X10"))))
    assert [(change.case, change.criterion) for change in diff.newly_failing] == [("X1", "b")]
    assert [(change.case, change.criterion) for change in diff.newly_passing] == [("X10", "a")]
    assert [(change.case, change.criterion) for change in diff.missing] == [("X2", "c")]


@pytest.mark.req("SCO-011")
def test_baseline_entries_for_unrun_errored_and_sweep_runs() -> None:
    unrun = not_run(
        case_with(value("final", "at_most", "1 V", name="z")),
        "X5",
        DesignRef(name="machine", version="0.1.0"),
        0,
        "bad",
    )
    errored = score_run(
        case_with(value("final", "at_most", "1 V", name="e"), id="X3"),
        recording_from([0.0], {}),
        RunReport("run_error", 0, Fraction(1000), "boom"),
        run=IDENTITY,
    )
    swept = CaseScore(**{**_scored("s", "3 V").__dict__, "sweep": ("${gain}", "1")})
    baseline = suite_baseline(_suite(errored, unrun, swept))
    assert [(entry.case, entry.status, entry.run is None) for entry in baseline.entries] == [
        ("X3", "error", False),
        ("X5", "not_run", True),
    ]
    with pytest.raises(ScoringError, match="archive identity"):
        suite_baseline(_suite(_scored("a", "3 V", run=None)))
    with pytest.raises(ScoringError, match="no criteria"):
        suite_baseline(_suite(swept))


@pytest.mark.req("SCO-009", "SCO-010")
def test_the_regression_diff_lists_what_changed_first() -> None:
    old = suite_baseline(
        _suite(_scored("b", "2 V"), _scored("a", "3 V", case_id="X10"), _scored("c", "3 V", case_id="X2"))
    )
    new = _suite(_scored("b", "3 V"), _scored("a", "2 V", case_id="X10"), _scored("d", "3 V", case_id="X3"))
    diff = regression_diff(new, old)
    assert diff == diff_baselines(old, suite_baseline(new))
    sections = [
        (title, [(change.case, change.criterion) for change in changes]) for title, changes in diff_sections(diff)
    ]
    assert sections == [
        ("newly passing", [("X1", "b")]),
        ("newly failing", [("X10", "a")]),
        ("unchanged", []),
        ("missing", [("X2", "c")]),
        ("added", [("X3", "d")]),
    ]
    with pytest.raises(ValueError, match="cannot diff suite"):
        regression_diff(SuiteScore("other", new.design, new.cases), old)


@pytest.mark.req("SCO-012")
def test_the_shareable_report_is_a_self_contained_markdown_document() -> None:
    passing = CaseScore(**{**_scored("a", "3 V").__dict__, "approximations": ("friction | lumped",)})
    unrun = not_run(None, "X3", DesignRef(name="machine", version="0.1.0"), 0, "invalid")
    suite = _suite(passing, unrun)
    diff = regression_diff(_suite(_scored("a", "3 V")), suite_baseline(_suite(_scored("a", "2 V"))))
    text = report_markdown(suite, diff)
    assert text == report_markdown(suite, diff)
    assert text.startswith("# Suite demo: ERROR\n\n- Design: machine 0.1.0\n- Runs: 2: 1 passed, 0 failed, 1 errored\n")
    assert (
        "## Case X1: PASS\n\n- Seed 11, design machine 0.1.0, controller mode native\n"
        f"- Run X1-seed0: design blob `{'b' * 40}`, case blob `{'c' * 40}`, archive sha256 `{'a' * 64}`\n"
        "- Ended: end_time at 3 s\n- Approximation: friction | lumped\n\n"
        "| Criterion | Status | Measured | Limit | Worst at | Note |\n|---|---|---|---|---|---|\n"
        "| a | pass | 3 V | maximum at_most 3 V | 1 s |  |\n"
    ) in text
    assert "## Case X3: ERROR\n\n- Seed 0, design machine 0.1.0, controller mode none\n- Run: no archive\n" in text
    assert "- Not run\n- Error: invalid\n" in text
    assert text.endswith(
        "## Regression diff\n\n### Newly passing (1)\n\n- X1 / a: fail -> pass\n\n### Newly failing (0)\n\nNone.\n\n"
        "### Unchanged (0)\n\nNone.\n\n### Missing (0)\n\nNone.\n\n### Added (0)\n\nNone.\n"
    )
    assert report_markdown(suite, None).endswith("## Regression diff\n\nNo baseline diff for this run.\n")


@pytest.mark.req("SCO-012")
def test_a_table_cell_escapes_pipes_and_a_run_error_is_stated() -> None:
    errored = score_run(
        case_with(value("final", "at_most", "1 V", name="e"), id="X4"),
        recording_from([0.0], {}),
        RunReport("run_error", 0, Fraction(1000), "a | b"),
        run=IDENTITY,
    )
    text = report_markdown(_suite(errored), None)
    assert "| e | error | n/a | final at_most 1 V |  | run error: a \\| b |\n" in text
    failed = CaseScore(**{**errored.__dict__, "ran": False, "end_reason": "run_error", "run": None})
    assert "- Ended: run_error, no recording\n" in report_markdown(_suite(failed), None)


@pytest.mark.req("SCO-020")
def test_rail_references_read_the_rail_state_and_voltage() -> None:
    # Ruling 27: rail:<name> is the recorded on/off series of the rail, rail:<name>.voltage its voltage.
    recording = recording_from(
        TIMES, {"signal:motor_power.on": [1.0, 1.0, 0.0, 0.0], "signal:motor_power.voltage": [24.0, 24.0, 0.0, 0.0]}
    )
    off = score_one(
        value("maximum", "at_most", "0", quantity="rail:motor_power", window={"start": "2 s"}),
        recording,
    )
    on = score_one(value("maximum", "at_most", "0", quantity="rail:motor_power"), recording)
    volts = score_one(value("minimum", "at_most", "0 V", quantity="rail:motor_power.voltage"), recording)
    assert (off.status, off.measured) == ("pass", 0.0)
    assert (on.status, on.measured) == ("fail", 1.0)
    assert (volts.status, volts.measured) == ("pass", 0.0)


@pytest.mark.req("SCO-013")
def test_layer_approximation_events_reach_the_score_once_each_after_the_bridge_lines() -> None:
    note = RecordedEvent(1.0, "load_cell", "approximation", "load past its overload limit, held at the limit")
    rows = [
        Row({"signal:a": 0.0}),
        Row({"signal:a": 1.0}, events=[note, RecordedEvent(1.0, "controller", "refused", "10")]),
        Row({"signal:a": 2.0}, events=[note]),
        Row({"signal:a": 3.0}),
    ]
    report = RunReport("end_time", 3, Fraction(1), None, (), (), ("bridge line",))
    scored = score_run(case_with(value("maximum", "at_most", "3")), FakeRecording(TIMES, rows, {}), report)
    assert scored.approximations == (
        "bridge line",
        "load_cell: load past its overload limit, held at the limit",
    )
