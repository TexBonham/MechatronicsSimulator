"""Oscillation statistics (SCO-027), the deflection history read both ways (SCO-028) and not-evaluated quantities as
unevaluable errors (SCO-029)."""

from __future__ import annotations

import math
from typing import Any

import pytest

from mechsim.integrity.plugin import CheckResult
from mechsim.integrity.table import IntegrityReport
from mechsim.scoring.score import score_run
from mechsim.scoring.statistics import StatisticError, statistic, upward_crossings
from mechsim.testing.driver import IntegrityResults, MemberIntegrity
from tests.unit.test_runner_fakes import recording_from
from tests.unit.test_scoring import ENDED, case_with, score_one, value

# x(t) = c + A sin(w t) sampled at h = T / 100 over exactly three periods (300 samples, t = 0 .. 2.99 s, T = 1 s).
# Over whole periods of equally spaced samples sum(sin(w t_n)) = 0 exactly (a geometric sum of roots of unity), so
# the window mean is c to float rounding (300 terms of size 1: under 1e-13). The samples at t = T/4 + nT and
# 3T/4 + nT land on the crests, so (max - min) / 2 = A to rounding.
OFFSET = -0.25
AMPLITUDE = 0.02
PERIOD = 1.0
STEP = 0.01
TIMES = [n * STEP for n in range(300)]
SWING = [OFFSET + AMPLITUDE * math.sin(2.0 * math.pi * t / PERIOD) for t in TIMES]


def _crossing_bound(count: int) -> float:
    # The chord root through samples a before and b after a crossing misses it by w^2 a b (b - a) / 6 (third-order
    # Taylor of sin; see the SCO-020 period test), at most w^2 h^3 / (36 sqrt(3)); the period (t_last - t_first) /
    # (N - 1) is off by at most twice that over N - 1. A constant error in the subtracted mean shifts every upward
    # crossing by the same time, so it cancels.
    omega = 2.0 * math.pi / PERIOD
    return 2.0 * omega**2 * STEP**3 / (36.0 * math.sqrt(3.0)) / (count - 1) + 1e-12


@pytest.mark.req("SCO-027")
def test_mean_and_amplitude_of_an_offset_sine() -> None:
    mean, mean_time = statistic("mean", TIMES, SWING)
    amplitude, amplitude_time = statistic("amplitude", TIMES, SWING)
    print(f"mean closed form {OFFSET}, measured {mean!r}; amplitude closed form {AMPLITUDE}, measured {amplitude!r}")
    assert abs(mean - OFFSET) <= 1e-13
    assert mean_time == TIMES[-1]
    assert abs(amplitude - AMPLITUDE) <= 1e-15
    # The later of the two extreme samples, each the first on a tie: crest 0.25 s, trough 0.75 s.
    assert amplitude_time == pytest.approx(0.75, abs=1e-12)


@pytest.mark.req("SCO-027")
def test_period_and_frequency_about_the_mean_of_an_offset_series() -> None:
    # The raw series never crosses zero (it swings about -0.25 by 0.02): the SCO-020 period is unevaluable there.
    with pytest.raises(StatisticError, match="found 0"):
        statistic("period", TIMES, SWING)
    count = len(upward_crossings(TIMES, [item - OFFSET for item in SWING]))
    bound = _crossing_bound(count)
    period, last = statistic("period_about_mean", TIMES, SWING)
    frequency, frequency_last = statistic("frequency_about_mean", TIMES, SWING)
    print(f"period closed form {PERIOD} s, measured {period!r} s over {count} crossings, bound {bound:.3e} s")
    # The sample at t = 0 sits on the mean (before < 0 fails), so the crossings are those at 1 s and 2 s.
    assert count == 2
    assert abs(period - PERIOD) <= bound
    assert frequency == 1.0 / period
    assert last == frequency_last
    assert last == pytest.approx(2.0, abs=bound)


@pytest.mark.req("SCO-027")
def test_an_about_the_mean_period_needs_two_crossings() -> None:
    flat = [OFFSET] * 10
    with pytest.raises(StatisticError, match="the window's mean, found 0"):
        statistic("period_about_mean", TIMES[:10], flat)


@pytest.mark.req("SCO-027")
def test_oscillation_criteria_on_a_recorded_series_report_their_units() -> None:
    recording = recording_from(TIMES, {"pose:bob.x": SWING}, "m")
    period = score_one(value("period_about_mean", "within", "1 ms", quantity="pose:bob.x", target="1 s"), recording)
    frequency = score_one(
        value("frequency_about_mean", "within", "1 mHz", quantity="pose:bob.x", target="1 Hz"), recording
    )
    amplitude = score_one(
        {**value("amplitude", "within", "0.1 mm", quantity="pose:bob.x", target="2 cm"), "window": {"start": "2 s"}},
        recording,
    )
    mean = score_one(
        {**value("mean", "within", "1 um", quantity="pose:bob.x", target="-25 cm"), "window": {"start": "1 s"}},
        recording,
    )
    assert (period.status, period.unit) == ("pass", "s")
    assert (frequency.status, frequency.unit) == ("pass", "Hz")
    assert (amplitude.status, amplitude.unit) == ("pass", "m")
    assert (mean.status, mean.unit) == ("pass", "m")
    wrong = score_one(value("period_about_mean", "within", "1 ms", quantity="pose:bob.x", target="1.05 s"), recording)
    assert wrong.status == "fail"


@pytest.mark.req("SCO-027")
@pytest.mark.parametrize(
    ("criterion", "message"),
    [
        (value("mean", "at_most", "1 m", quantity="pose:bob.x"), "needs a window with a start"),
        (
            {**value("mean", "at_most", "1 m", quantity="pose:bob.x"), "window": {"end": "1 s"}},
            "needs a window with a start",
        ),
        (value("frequency_about_mean", "at_most", "1 s", quantity="pose:bob.x"), "expected 1 / [time]"),
        (value("period_about_mean", "at_most", "1 Hz", quantity="pose:bob.x"), "expected [time]"),
        (value("amplitude", "at_most", "1", quantity="count:message.go"), "needs a series"),
        (value("mean", "at_most", "1", quantity="state:idle"), "needs a series"),
    ],
)
def test_the_case_rejects_an_oscillation_statistic_it_cannot_score(criterion: dict[str, Any], message: str) -> None:
    with pytest.raises(ValueError, match=message.replace("[", r"\[").replace("]", r"\]")):
        case_with(criterion)


# Deflection history: the release overshoot peaks at 2 d, the run settles at d (a straight release under a static load
# swings between 0 and twice the static deflection). The table's peak is the history's maximum.
HISTORY_TIMES = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0)
HISTORY = (0.0, 0.010, 0.002, 0.006, 0.005, 0.005, 0.005)


def _report(history: tuple[float, ...] | None) -> IntegrityResults:
    row = MemberIntegrity("bar", 1.0e6, 0.5, max(HISTORY), 0.5, 4.0, 0.5)
    series: tuple[tuple[str, tuple[float, ...]], ...] = ()
    if history is not None:
        series = (("bar", history),)
    check = CheckResult("deflection", 1, (), series=series)
    return IntegrityReport((row,), (), (check,), 1.0, (), HISTORY_TIMES)


@pytest.mark.req("SCO-028", "SCO-014")
def test_deflection_reads_both_the_dynamic_peak_and_the_settled_value() -> None:
    recording = recording_from(list(HISTORY_TIMES), {"signal:s.v": [0.0] * 7}, "")
    peak = value("maximum", "at_most", "1 cm", name="peak", quantity="deflection:bar")
    settled = {
        **value("mean", "within", "0.1 mm", name="settled", quantity="deflection:bar", target="5 mm"),
        "window": {"start": "2 s"},
    }
    scored = score_run(case_with(peak, settled), recording, ENDED, integrity=_report(HISTORY))
    first, second = scored.criteria
    assert (first.status, first.measured, first.worst_time) == ("pass", 0.010, 0.5)
    assert (second.status, second.measured, second.unit) == ("pass", 0.005, "m")
    # Without the history (an archived table) the peak stays readable; a final window holds no sample: an error.
    table_only = score_run(case_with(peak, settled), recording, ENDED, integrity=_report(None))
    assert table_only.criteria[0].measured == 0.010
    assert (table_only.criteria[1].status, table_only.criteria[1].note) == ("error", "no samples in the window")


@pytest.mark.req("SCO-029", "INT-006")
def test_a_member_integrity_did_not_evaluate_is_unevaluable_never_pass_or_fail() -> None:
    rigid = IntegrityResults(
        (MemberIntegrity("bar", 0.0, 0.0, math.nan, 0.0, math.nan, 0.0, ("deflection_not_evaluated",)),)
    )
    recording = recording_from(TIMES, {"signal:s.v": SWING}, "")
    for quantity, comparison in (("deflection:bar", "at_most"), ("safety_factor:bar", "at_least")):
        limit = "1 cm"
        if quantity.startswith("safety"):
            limit = "1.5"
        criterion = value("maximum", comparison, limit, quantity=quantity)
        scored = score_run(case_with(criterion), recording, ENDED, integrity=rigid)
        (result,) = scored.criteria
        assert (result.status, result.measured) == ("error", None)
        assert result.note == f"{quantity}: not evaluated by integrity (flags: deflection_not_evaluated)"
        assert scored.status == "error"


@pytest.mark.req("SCO-029")
@pytest.mark.parametrize("statistic_name", ["final", "minimum", "maximum", "peak_absolute", "mean", "amplitude"])
def test_a_nan_sample_read_by_a_criterion_is_unevaluable(statistic_name: str) -> None:
    samples = [1.0, math.nan, 2.0, 3.0]
    recording = recording_from([0.0, 1.0, 2.0, 3.0], {"signal:s.v": samples}, "V")
    criterion = {**value(statistic_name, "at_most", "100 V"), "window": {"start": "0 s"}}
    result = score_one(criterion, recording)
    assert (result.status, result.note) == ("error", "not evaluated: the sample at 1 s is not a number")
    settle = {
        "name": "s",
        "check": "settle_time",
        "quantity": "signal:s.v",
        "band": "0.1 V",
        "target": "3 V",
        "comparison": "at_most",
        "limit": "5 s",
    }
    assert score_one(settle, recording).status == "error"
    with pytest.raises(StatisticError, match="not a number"):
        statistic(statistic_name, [0.0, 1.0, 2.0, 3.0], samples)
