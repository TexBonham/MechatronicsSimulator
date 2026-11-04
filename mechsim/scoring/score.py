"""Scoring a run (SCO-001 to SCO-008, SCO-013, SCO-016, SCO-017, SCO-020 to SCO-024) and a suite.

Every criterion is evaluated from the recording alone, with the measure and limit the case states. A run that ended in
a run error is scored as errored, every criterion with it (SCO-007, TST-023). A criterion that cannot be evaluated
(series not recorded, empty window, too few zero crossings, unresolved limit) is an error, never a pass.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Literal

from mechsim.design.schema import OpenValue
from mechsim.scoring.statistics import StatisticError, compare, statistic, window_indices, within
from mechsim.testing.baseline import Baseline, Entry, RunIdentity, entry_key
from mechsim.testing.driver import MESSAGE_KIND, RecordView, RunReport, Series
from mechsim.testing.schema import (
    Case,
    DesignRef,
    EventCriterion,
    IntervalCriterion,
    NoMotionCriterion,
    NoOverlapCriterion,
    Quantity,
    SettleCriterion,
    ValueCriterion,
    Window,
)

CriterionStatus = Literal["pass", "fail", "error"]
CaseStatus = Literal["pass", "fail", "error"]


class ScoringError(ValueError):
    """Scoring could not produce what was asked (for example a baseline from a run without an archive)."""


class _UnevaluableError(Exception):
    pass


@dataclass(frozen=True)
class CriterionResult:
    """Measured value beside the limit, pass or fail, time of the worst case (SCO-002 to SCO-004)."""

    name: str
    status: CriterionStatus
    limit: str
    measured: float | None = None
    unit: str = ""
    worst_time: float | None = None
    note: str | None = None

    def measured_text(self) -> str | None:
        if self.measured is None:
            return None
        return f"{self.measured:.12g} {self.unit}".strip()


@dataclass(frozen=True)
class CaseScore:
    """One scored run with the identity the report states (SCO-013); `ran` is False for a run stopped before t = 0."""

    case_id: str
    status: CaseStatus
    criteria: tuple[CriterionResult, ...]
    design: DesignRef
    seed: int
    controller_mode: str | None
    ran: bool
    end_reason: str | None = None
    end_time: float | None = None
    error: str | None = None
    sweep: tuple[str, str] | None = None
    run: RunIdentity | None = None


@dataclass(frozen=True)
class SuiteScore:
    name: str
    design: DesignRef
    cases: tuple[CaseScore, ...]

    @property
    def status(self) -> CaseStatus:
        """Passed only when every run passes (SCO-006); any error makes the suite errored."""
        statuses = [case.status for case in self.cases]
        if "error" in statuses:
            return "error"
        if "fail" in statuses or len(statuses) == 0:
            return "fail"
        return "pass"


# ===== Criteria =========================================================


def score_run(
    case: Case,
    recording: RecordView,
    report: RunReport,
    limits: dict[str, str] | None = None,
    sweep: tuple[str, str] | None = None,
    run: RunIdentity | None = None,
) -> CaseScore:
    """Score one run of a resolved case. `limits` maps criterion name to the limit text of the case as written
    (its `${}` forms), which the baseline keeps; by default the resolved criterion's own text."""
    results: list[CriterionResult] = []
    for criterion in case.criteria:
        limit = _limit_text(criterion, limits)
        if report.errored:
            results.append(CriterionResult(criterion.name, "error", limit, note=f"run error: {report.error}"))
            continue
        try:
            results.append(_evaluate(criterion, recording, limit))
        except (_UnevaluableError, StatisticError, KeyError) as error:
            results.append(CriterionResult(criterion.name, "error", limit, note=_reason(error)))
    status = _case_status(results)
    if report.errored:
        status = "error"
    return CaseScore(
        case_id=case.id,
        status=status,
        criteria=tuple(results),
        design=case.design,
        seed=case.seed,
        controller_mode=case.controller_mode,
        ran=True,
        end_reason=report.reason,
        end_time=report.end_time,
        error=report.error,
        sweep=sweep,
        run=run,
    )


def not_run(
    case: Case | None, case_id: str, design: DesignRef, seed: int, error: str, sweep: tuple[str, str] | None = None
) -> CaseScore:
    """A case the runner stopped before t = 0 (invalid, unresolved, precondition mismatch): errored, never passed."""
    criteria: tuple[CriterionResult, ...] = ()
    mode: str | None = None
    if case is not None:
        criteria = tuple(CriterionResult(item.name, "error", item.limit_text(), note=error) for item in case.criteria)
        mode = case.controller_mode
    return CaseScore(case_id, "error", criteria, design, seed, mode, ran=False, error=error, sweep=sweep)


def _limit_text(criterion: Any, limits: dict[str, str] | None) -> str:
    if limits is not None and criterion.name in limits:
        return limits[criterion.name]
    text: str = criterion.limit_text()
    return text


def _reason(error: Exception) -> str:
    if isinstance(error, KeyError):
        return f"not recorded: {error.args[0]}"
    return str(error)


def _case_status(results: Sequence[CriterionResult]) -> CaseStatus:
    """Passed only when every criterion passes (SCO-005)."""
    statuses = [result.status for result in results]
    if "error" in statuses:
        return "error"
    if "fail" in statuses:
        return "fail"
    return "pass"


def _evaluate(criterion: Any, recording: RecordView, limit: str) -> CriterionResult:
    if isinstance(criterion, ValueCriterion):
        return _value(criterion, recording, limit)
    if isinstance(criterion, EventCriterion):
        return _event(criterion, recording, limit)
    if isinstance(criterion, IntervalCriterion):
        return _interval(criterion, recording, limit)
    if isinstance(criterion, SettleCriterion):
        return _settle(criterion, recording, limit)
    if isinstance(criterion, NoMotionCriterion):
        return _no_motion(criterion, recording, limit)
    if isinstance(criterion, NoOverlapCriterion):
        raise _UnevaluableError("no_overlap needs actuator command records, not evaluated in milestone 1 (SCO-025)")
    raise _UnevaluableError(f"unknown criterion {criterion!r}")


def _number(value: Any, what: str) -> float:
    if isinstance(value, OpenValue):
        raise _UnevaluableError(f"{what} {value} is unresolved")
    if isinstance(value, Quantity):
        return value.value
    if isinstance(value, float):
        return value
    raise _UnevaluableError(f"{what} {value!r} is not a number")


def _bounds(window: Window | None) -> tuple[float | None, float | None]:
    if window is None:
        return None, None
    start: float | None = None
    end: float | None = None
    if window.start is not None:
        start = _number(window.start, "window start")
    if window.end is not None:
        end = _number(window.end, "window end")
    return start, end


def _windowed(series: Series, window: Window | None) -> tuple[list[float], list[float]]:
    start, end = _bounds(window)
    indices = window_indices(series.times, start, end)
    if len(indices) == 0:
        raise _UnevaluableError("no samples in the window")
    return [series.times[index] for index in indices], [series.values[index] for index in indices]


def _series(reference: str, recording: RecordView) -> Series:
    kind, _, rest = reference.partition(":")
    if kind == "count":
        source, _, name = rest.partition(".")
        times = list(recording.times())
        occurrences = _occurrences(f"{source}:{name}", recording)
        values: list[float] = []
        seen = 0
        for time in times:
            while seen < len(occurrences) and occurrences[seen] <= time:
                seen += 1
            values.append(float(seen))
        return Series(tuple(times), tuple(values), "")
    return recording.series(reference)


def _occurrences(event: str, recording: RecordView) -> list[float]:
    """Times of `message:<name>` (an operator message event) or `state:<name>` (each entry into the state)."""
    kind, _, name = event.partition(":")
    if kind == "message":
        return sorted(item.time for item in recording.events() if item.kind == MESSAGE_KIND and item.detail == name)
    times = recording.times()
    states = recording.controller_states()
    entries: list[float] = []
    for index, state in enumerate(states):
        if state == name and (index == 0 or states[index - 1] != name):
            entries.append(times[index])
    return entries


def _value(criterion: ValueCriterion, recording: RecordView, limit: str) -> CriterionResult:
    series = _series(criterion.quantity, recording)
    times, values = _windowed(series, criterion.window)
    measured, worst = statistic(criterion.statistic, times, values)
    unit = series.unit
    if criterion.statistic == "period":
        unit = "s"
    if criterion.comparison == "within":
        target = _number(criterion.target, "target")
        if criterion.relative_limit is not None:
            tolerance = _number(criterion.relative_limit, "relative_limit") * abs(target)
        else:
            tolerance = _number(criterion.limit, "limit")
        passed = within(measured, target, tolerance)
    else:
        passed = compare(measured, criterion.comparison, _number(criterion.limit, "limit"))
    return CriterionResult(criterion.name, _status(passed), limit, measured, unit, worst)


def _status(passed: bool) -> CriterionStatus:
    if passed:
        return "pass"
    return "fail"


def _in_window(times: Sequence[float], window: Window | None) -> list[float]:
    start, end = _bounds(window)
    return [times[index] for index in window_indices(times, start, end)]


def _event(criterion: EventCriterion, recording: RecordView, limit: str) -> CriterionResult:
    found = _in_window(_occurrences(criterion.event, recording), criterion.window)
    worst: float | None = None
    if len(found) > 0:
        worst = found[0]
    passed = len(found) == 0
    if criterion.expect == "present":
        passed = len(found) > 0
    return CriterionResult(criterion.name, _status(passed), limit, float(len(found)), "", worst)


def _interval(criterion: IntervalCriterion, recording: RecordView, limit: str) -> CriterionResult:
    starts = _occurrences(criterion.start_event, recording)
    if len(starts) == 0:
        raise _UnevaluableError(f"start event {criterion.start_event} never recorded")
    ends = [time for time in _occurrences(criterion.end_event, recording) if time >= starts[0]]
    if len(ends) == 0:
        raise _UnevaluableError(f"end event {criterion.end_event} never recorded after {criterion.start_event}")
    measured = ends[0] - starts[0]
    passed = compare(measured, criterion.comparison, _number(criterion.limit, "limit"))
    return CriterionResult(criterion.name, _status(passed), limit, measured, "s", ends[0])


def _settle(criterion: SettleCriterion, recording: RecordView, limit: str) -> CriterionResult:
    """Settle time: from the window's first sample to the first sample after which every sample stays within
    `band` of the target (the window's final value when the case gives none)."""
    times, values = _windowed(_series(criterion.quantity, recording), criterion.window)
    target = values[-1]
    if criterion.target is not None:
        target = _number(criterion.target, "target")
    band = _number(criterion.band, "band")
    last_out = -1
    for index, value in enumerate(values):
        if not within(value, target, band):
            last_out = index
    if last_out == len(values) - 1:
        measured = times[-1] - times[0]
        return CriterionResult(criterion.name, "fail", limit, measured, "s", times[-1], "did not settle in the window")
    worst = times[0]
    if last_out >= 0:
        worst = times[last_out]
    measured = times[last_out + 1] - times[0]
    passed = compare(measured, criterion.comparison, _number(criterion.limit, "limit"))
    return CriterionResult(criterion.name, _status(passed), limit, measured, "s", worst)


def _no_motion(criterion: NoMotionCriterion, recording: RecordView, limit: str) -> CriterionResult:
    """Motion: the largest excursion of any listed actuator's position from its first sample in the window."""
    threshold = _number(criterion.threshold, "threshold")
    measured = 0.0
    worst: float | None = None
    unit = ""
    for actuator in criterion.actuators:
        series = recording.series(f"actuator:{actuator}.position")
        times, values = _windowed(series, criterion.window)
        for time, value in zip(times, values, strict=True):
            excursion = abs(value - values[0])
            if worst is None or excursion > measured:
                measured = excursion
                worst = time
                unit = series.unit
    return CriterionResult(criterion.name, _status(measured <= threshold), limit, measured, unit, worst)


# ===== Suite, baseline and report =======================================


def suite_baseline(suite: SuiteScore) -> Baseline:
    """The suite's scoring summary in the baseline format (SCO-011), from its nominal runs (sweep runs are reported
    side by side, not baselined)."""
    entries: list[Entry] = []
    for case in suite.cases:
        if case.sweep is not None:
            continue
        for result in case.criteria:
            entries.append(_entry(case, result))
    if len(entries) == 0:
        raise ScoringError(f"suite {suite.name!r} has no criteria to baseline")
    ordered = sorted(entries, key=entry_key)
    return Baseline.model_validate({"design": suite.design.model_dump(), "suite": suite.name, "entries": ordered})


def _entry(case: CaseScore, result: CriterionResult) -> Entry:
    data: dict[str, Any] = {"case": case.case_id, "criterion": result.name, "limit": result.limit}
    if not case.ran:
        data["status"] = "not_run"
        return Entry.model_validate(data)
    if case.run is None:
        raise ScoringError(f"{case.case_id}: a baseline needs the run's archive identity")
    data["run"] = case.run
    data["status"] = result.status
    if result.status == "error" or result.measured_text() is None:
        data["status"] = "error"
        return Entry.model_validate(data)
    data["measured"] = result.measured_text()
    if result.worst_time is not None:
        data["worst_time"] = f"{result.worst_time:.12g} s"
    return Entry.model_validate(data)


def report_text(suite: SuiteScore) -> str:
    """The plain-text pass/fail report: suite verdict, then per run the identity and every criterion (SCO-008,
    SCO-013); sweep runs of a case side by side per sweep value (SCO-018). Deterministic (SCO-016)."""
    counts = {"pass": 0, "fail": 0, "error": 0}
    for case in suite.cases:
        counts[case.status] += 1
    lines = [
        f"Suite {suite.name} ({suite.design.name} {suite.design.version}): {suite.status.upper()}",
        f"{len(suite.cases)} runs: {counts['pass']} passed, {counts['fail']} failed, {counts['error']} errored",
    ]
    for case in suite.cases:
        lines.append("")
        lines.extend(_case_lines(case))
    lines.extend(_sweep_tables(suite.cases))
    return "\n".join(lines) + "\n"


def _case_lines(case: CaseScore) -> list[str]:
    mode = "none"
    if case.controller_mode is not None:
        mode = case.controller_mode
    title = f"Case {case.case_id}"
    if case.sweep is not None:
        title = f"{title} [{case.sweep[0]} = {case.sweep[1]}]"
    lines = [
        f"{title}: {case.status.upper()}",
        f"  seed {case.seed}, design {case.design.name} {case.design.version}, controller mode {mode}",
    ]
    if case.ran and case.end_time is not None:
        lines.append(f"  ended: {case.end_reason} at {case.end_time:.12g} s")
    else:
        lines.append("  not run")
    if case.error is not None:
        lines.append(f"  error: {case.error}")
    for result in case.criteria:
        measured = result.measured_text()
        if measured is None:
            measured = "n/a"
        line = f"  {result.status:<5} {result.name}: measured {measured}; limit {result.limit}"
        if result.worst_time is not None:
            line = f"{line}; worst at {result.worst_time:.12g} s"
        if result.note is not None:
            line = f"{line} ({result.note})"
        lines.append(line)
    return lines


def _sweep_tables(cases: Sequence[CaseScore]) -> list[str]:
    groups: dict[tuple[str, str], list[CaseScore]] = {}
    for case in cases:
        if case.sweep is not None:
            groups.setdefault((case.case_id, case.sweep[0]), []).append(case)
    lines: list[str] = []
    for (case_id, parameter), runs in groups.items():
        lines.append("")
        lines.append(f"Sweep {case_id} over {parameter}")
        lines.append("| criterion | " + " | ".join(run.sweep[1] for run in runs if run.sweep is not None) + " |")
        lines.append("|---" * (len(runs) + 1) + "|")
        names = [result.name for result in runs[0].criteria]
        for name in names:
            cells: list[str] = []
            for run in runs:
                cells.append(_cell(run, name))
            lines.append(f"| {name} | " + " | ".join(cells) + " |")
    return lines


def _cell(run: CaseScore, name: str) -> str:
    for result in run.criteria:
        if result.name == name:
            measured = result.measured_text()
            if measured is None:
                return result.status
            return f"{measured} ({result.status})"
    return "n/a"
