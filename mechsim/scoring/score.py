"""Scoring a run (SCO-001 to SCO-008, SCO-013, SCO-016, SCO-017, SCO-020 to SCO-025) and a suite (SCO-009 to
SCO-012).

Every criterion is evaluated from the recording alone, with the measure and limit the case states. A run that ended in
a run error is scored as errored, every criterion with it (SCO-007, TST-023). A criterion that cannot be evaluated
(series not recorded, empty window, too few zero crossings, unresolved limit) is an error, never a pass.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, replace
from typing import Any, Literal, Protocol, runtime_checkable

from mechsim.design.schema import OpenValue
from mechsim.scoring.statistics import StatisticError, compare, statistic, window_indices, within
from mechsim.testing.baseline import Baseline, BaselineDiff, Change, Entry, RunIdentity, diff_baselines, entry_key
from mechsim.testing.driver import (
    FAULT_KIND,
    INTEGRITY_KIND,
    MESSAGE_KIND,
    IntegrityCrossing,
    IntegrityResults,
    RecordedEvent,
    RecordView,
    RunReport,
    Series,
    StructuralError,
)
from mechsim.testing.schema import (
    Case,
    DesignRef,
    EventCriterion,
    HoldBandCriterion,
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
    approximations: tuple[str, ...] = ()
    variation: tuple[tuple[str, str], ...] | None = None
    crossings: tuple[str, ...] = ()
    notes: tuple[str, ...] = ()


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
    integrity: IntegrityResults | None = None,
) -> CaseScore:
    """Score one run of a resolved case. `limits` maps criterion name to the limit text of the case as written
    (its `${}` forms), which the baseline keeps; by default the resolved criterion's own text. Structural criteria
    read `integrity` (SCO-014); a recorded tier 1 crossing fails the case (SCO-015)."""
    recording = _StructuralView(recording, integrity)
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
    crossings = _crossings(recording)
    if (len(crossings) > 0 or report.reason == "integrity_halt") and status == "pass":
        status = "fail"
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
        approximations=report.approximations + _layer_approximations(recording),
        crossings=crossings,
        notes=tuple(report.notes),
    )


STRUCTURAL_KINDS = ("safety_factor", "deflection")


@runtime_checkable
class _DeflectionHistory(Protocol):
    """Integrity results that also carry each member's deflection per recorded sample (the integrity report)."""

    def deflection_series(self, member: str) -> tuple[Sequence[float], Sequence[float]] | None: ...


class _StructuralView:
    """The run's record view plus the integrity results it carries (SCO-014): `safety_factor:<member>` (the minimum
    over the run, dimensionless) read from the per-member table as one sample at the time integrity reports for it;
    `deflection:<member>` (m) the member's deflection at every recorded sample when the results carry it (SCO-027: its
    maximum is the table's peak, its mean over a final window the settled value), else the table's peak as one sample;
    never recomputed from the recording. A table value integrity did not evaluate (NaN) is unevaluable, never a number
    (invariant 7). Other refs go to the recording."""

    def __init__(self, recording: RecordView, integrity: IntegrityResults | None) -> None:
        self._recording = recording
        self._integrity = integrity

    def times(self) -> Sequence[float]:
        return self._recording.times()

    def controller_states(self) -> Sequence[str]:
        return self._recording.controller_states()

    def events(self) -> Sequence[RecordedEvent]:
        return self._recording.events()

    def series(self, reference: str) -> Series:
        kind, _, member = reference.partition(":")
        if kind not in STRUCTURAL_KINDS:
            return self._recording.series(reference)
        if self._integrity is None:
            raise _UnevaluableError(f"{reference}: the run has no integrity results")
        try:
            entry = self._integrity.member(member)
        except KeyError as error:
            raise _UnevaluableError(f"{reference}: integrity reports no member {member!r}") from error
        if kind == "safety_factor":
            value = entry.safety_factor
            time = entry.safety_factor_time
            unit = ""
        else:
            value = entry.deflection
            time = entry.deflection_time
            unit = "m"
        if math.isnan(value):
            flags = ", ".join(entry.flags)
            raise _UnevaluableError(f"{reference}: not evaluated by integrity (flags: {flags})")
        if kind == "deflection" and isinstance(self._integrity, _DeflectionHistory):
            history = self._integrity.deflection_series(member)
            if history is not None:
                return Series(tuple(history[0]), tuple(history[1]), "m")
        return Series((time,), (value,), unit)


def _crossings(recording: RecordView) -> tuple[str, ...]:
    """Each recorded tier 1 integrity crossing (an `integrity` event whose detail says tier 1; INT-024, ruling 26)
    as one report line, in record order; an unreadable detail is a line too (it may be tier 1)."""
    lines: list[str] = []
    for event in recording.events():
        if event.kind != INTEGRITY_KIND:
            continue
        try:
            crossing = IntegrityCrossing.parse(event.detail)
            if crossing.tier != 1:
                continue
            lines.append(
                f"{crossing.member} {crossing.quantity} {crossing.value:.12g} crossed limit {crossing.limit:.12g} "
                f"at {event.time:.12g} s (tier {crossing.tier})"
            )
        except StructuralError as error:
            lines.append(f"unreadable crossing at {event.time:.12g} s: {error}")
    return tuple(lines)


def _layer_approximations(recording: RecordView) -> tuple[str, ...]:
    """Recorded `approximation` events (the signal layer's component notes, such as a load cell saturating; ruling
    20) as `<source>: <detail>` lines, each once, in record order (SCO-013)."""
    lines: list[str] = []
    for event in recording.events():
        if event.kind == "approximation":
            line = f"{event.source}: {event.detail}"
            if line not in lines:
                lines.append(line)
    return tuple(lines)


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


def run_failed(case: Case, error: str, sweep: tuple[str, str] | None = None) -> CaseScore:
    """A run the driver could not carry out once asked (a typed driver error from `run`, invariant 7): errored with
    end reason run_error and the message; no recording, so nothing is scored and no end time is stated."""
    score = not_run(case, case.id, case.design, case.seed, f"run error: {error}", sweep)
    return replace(score, end_reason="run_error")


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
    if isinstance(criterion, HoldBandCriterion):
        return _hold_band(criterion, recording, limit)
    if isinstance(criterion, NoOverlapCriterion):
        return _no_overlap(criterion, recording, limit)
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
    times = [series.times[index] for index in indices]
    values = [series.values[index] for index in indices]
    for time, value in zip(times, values, strict=True):
        if math.isnan(value):
            raise _UnevaluableError(f"not evaluated: the sample at {time:.12g} s is not a number")
    return times, values


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
    if kind == "fault":
        return _fault_series(rest, recording)
    if kind == "rail":
        name, _, field = rest.partition(".")
        if field == "":
            field = "on"
        return recording.series(f"signal:{name}.{field}")
    return recording.series(reference)


def _fault_changes(recording: RecordView) -> list[tuple[float, str]]:
    """The controller's reported fault each time it changed (ruling 30): `controller_fault` events, time order."""
    changes = [(item.time, item.detail) for item in recording.events() if item.kind == FAULT_KIND]
    return sorted(changes, key=lambda change: change[0])


def _fault_series(name: str, recording: RecordView) -> Series:
    """`fault:<name>` as a quantity: 1 at each sample while the controller's reported fault is `name`, else 0; the
    reported fault at a sample is the last change at or before it (none recorded before the first change)."""
    changes = _fault_changes(recording)
    times = list(recording.times())
    values: list[float] = []
    seen = 0
    current = ""
    for time in times:
        while seen < len(changes) and changes[seen][0] <= time:
            current = changes[seen][1]
            seen += 1
        if current == name:
            values.append(1.0)
        else:
            values.append(0.0)
    return Series(tuple(times), tuple(values), "")


def _occurrences(event: str, recording: RecordView) -> list[float]:
    """Times of `message:<name>` (an operator message event), `event:<name>` (a recorded event of kind `name`, such
    as a firmware log event), `fault:<name>` (each change of the controller's reported fault to `name`, ruling 30)
    or `state:<name>` (each entry into the state)."""
    kind, _, name = event.partition(":")
    if kind == "message":
        return sorted(item.time for item in recording.events() if item.kind == MESSAGE_KIND and item.detail == name)
    if kind == "event":
        return sorted(item.time for item in recording.events() if item.kind == name)
    if kind == "fault":
        return [time for time, detail in _fault_changes(recording) if detail == name]
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
    if criterion.statistic in ("period", "period_about_mean"):
        unit = "s"
    if criterion.statistic == "frequency_about_mean":
        unit = "Hz"
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
    every_end = _occurrences(criterion.end_event, recording)
    if len(every_end) == 0:
        raise _UnevaluableError(f"end event {criterion.end_event} never recorded")
    ends = [time for time in every_end if time >= starts[0]]
    if len(ends) == 0:
        raise _UnevaluableError(
            f"end event {criterion.end_event} recorded only before the first start event {criterion.start_event} "
            f"at {starts[0]:.12g} s"
        )
    measured = ends[0] - starts[0]
    passed = compare(measured, criterion.comparison, _number(criterion.limit, "limit"))
    return CriterionResult(criterion.name, _status(passed), limit, measured, "s", ends[0])


def _no_overlap(criterion: NoOverlapCriterion, recording: RecordView, limit: str) -> CriterionResult:
    """Start and end events whose detail is the actuator, in time order (record order within one time): a start while
    another is open is an overlap; with `closed`, each start still open when the run ends is one too. Measured is the
    count of violations, worst time the first of them. No start at all, or an end with no open start, is
    unevaluable: the pairing the criterion rests on is not there."""
    start = criterion.start_event.partition(":")[2]
    end = criterion.end_event.partition(":")[2]
    ordered = sorted(
        (item.time, index, item.kind)
        for index, item in enumerate(recording.events())
        if item.detail == criterion.actuator and item.kind in (start, end)
    )
    if not any(kind == start for _, _, kind in ordered):
        raise _UnevaluableError(f"start event {criterion.start_event} never recorded on {criterion.actuator}")
    open_starts = 0
    violations = 0
    worst: float | None = None
    for time, _, kind in ordered:
        if kind == start:
            if open_starts > 0:
                violations += 1
                if worst is None:
                    worst = time
            open_starts += 1
        else:
            if open_starts == 0:
                raise _UnevaluableError(
                    f"end event {criterion.end_event} on {criterion.actuator} at {time:.12g} s has no open start "
                    f"event {criterion.start_event}"
                )
            open_starts -= 1
    if criterion.closed and open_starts > 0:
        violations += open_starts
        if worst is None:
            times = recording.times()
            if len(times) > 0:
                worst = times[-1]
    return CriterionResult(criterion.name, _status(violations == 0), limit, float(violations), "", worst)


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


def _hold_band(criterion: HoldBandCriterion, recording: RecordView, limit: str) -> CriterionResult:
    """The largest excursion of any listed joint's position from its first sample in the window, against the band."""
    band = _number(criterion.band, "band")
    measured = 0.0
    worst: float | None = None
    unit = ""
    for joint in criterion.joints:
        series = recording.series(f"joint:{joint}.position")
        times, values = _windowed(series, criterion.window)
        for time, value in zip(times, values, strict=True):
            excursion = abs(value - values[0])
            if worst is None or excursion > measured:
                measured = excursion
                worst = time
                unit = series.unit
    return CriterionResult(criterion.name, _status(measured <= band), limit, measured, unit, worst)


# ===== Suite, baseline and report =======================================


def suite_baseline(suite: SuiteScore) -> Baseline:
    """The suite's scoring summary in the baseline format (SCO-011), from its nominal runs (sweep runs are reported
    side by side, not baselined)."""
    entries: list[Entry] = []
    for case in suite.cases:
        if case.sweep is not None or case.variation is not None:
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


def regression_diff(suite: SuiteScore, baseline: Baseline) -> BaselineDiff:
    """The suite's results against its regression baseline (SCO-009, SCO-010): newly passing, newly failing,
    unchanged, missing, added, per case and criterion."""
    return diff_baselines(baseline, suite_baseline(suite))


_DIFF_SECTIONS = ("newly passing", "newly failing", "unchanged", "missing", "added")


def diff_sections(diff: BaselineDiff) -> tuple[tuple[str, tuple[Change, ...]], ...]:
    """The diff's groups in report order: what changed first (SCO-010)."""
    groups = (diff.newly_passing, diff.newly_failing, diff.unchanged, diff.missing, diff.added)
    return tuple(zip(_DIFF_SECTIONS, groups, strict=True))


def _status_word(status: str | None) -> str:
    if status is None:
        return "absent"
    return status


def report_markdown(suite: SuiteScore, diff: BaselineDiff | None) -> str:
    """The pass/fail report as a self-contained Markdown document (SCO-012): verdict, design and version, per run its
    identity with the archive's git blob hashes, a table of criteria, the approximation lines and the end reason,
    then the regression diff. Deterministic text (SCO-016)."""
    counts = {"pass": 0, "fail": 0, "error": 0}
    for case in suite.cases:
        counts[case.status] += 1
    lines = [
        f"# Suite {suite.name}: {suite.status.upper()}",
        "",
        f"- Design: {suite.design.name} {suite.design.version}",
        f"- Runs: {len(suite.cases)}: {counts['pass']} passed, {counts['fail']} failed, {counts['error']} errored",
    ]
    for case in suite.cases:
        lines.append("")
        lines.extend(_case_markdown(case))
    lines.append("")
    lines.append("## Regression diff")
    lines.append("")
    if diff is None:
        lines.append("No baseline diff for this run.")
    else:
        for title, changes in diff_sections(diff):
            lines.append(f"### {title.capitalize()} ({len(changes)})")
            lines.append("")
            if len(changes) == 0:
                lines.append("None.")
            for change in changes:
                lines.append(
                    f"- {change.case} / {change.criterion}: {_status_word(change.old)} -> {_status_word(change.new)}"
                )
            lines.append("")
        lines.pop()
    return "\n".join(lines) + "\n"


def _cell_text(text: str) -> str:
    return text.replace("|", "\\|").replace("\n", " ")


def _case_markdown(case: CaseScore) -> list[str]:
    mode = "none"
    if case.controller_mode is not None:
        mode = case.controller_mode
    title = _title(f"## Case {case.case_id}", case)
    lines = [
        f"{title}: {case.status.upper()}",
        "",
        f"- Seed {case.seed}, design {case.design.name} {case.design.version}, controller mode {mode}",
    ]
    if case.run is not None:
        lines.append(
            f"- Run {case.run.run_id}: design blob `{case.run.design_hash}`, case blob `{case.run.case_hash}`, "
            f"archive sha256 `{case.run.archive_sha256}`"
        )
    else:
        lines.append("- Run: no archive")
    if case.ran and case.end_time is not None:
        lines.append(f"- Ended: {case.end_reason} at {case.end_time:.12g} s")
    elif case.end_reason == "run_error":
        lines.append("- Ended: run_error, no recording")
    else:
        lines.append("- Not run")
    if case.error is not None:
        lines.append(f"- Error: {_cell_text(case.error)}")
    for line in case.notes:
        lines.append(f"- Note: {_cell_text(line)}")
    for line in case.approximations:
        lines.append(f"- Approximation: {line}")
    for line in case.crossings:
        lines.append(f"- Integrity crossing: {line}")
    if len(case.criteria) == 0:
        return lines
    lines.extend(["", "| Criterion | Status | Measured | Limit | Worst at | Note |", "|---|---|---|---|---|---|"])
    for result in case.criteria:
        measured = result.measured_text()
        if measured is None:
            measured = "n/a"
        worst = ""
        if result.worst_time is not None:
            worst = f"{result.worst_time:.12g} s"
        note = ""
        if result.note is not None:
            note = result.note
        cells = [result.name, result.status, measured, result.limit, worst, note]
        lines.append("| " + " | ".join(_cell_text(cell) for cell in cells) + " |")
    return lines


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
    lines.extend(_variation_tables(suite.cases))
    return "\n".join(lines) + "\n"


def _case_lines(case: CaseScore) -> list[str]:
    mode = "none"
    if case.controller_mode is not None:
        mode = case.controller_mode
    title = _title(f"Case {case.case_id}", case)
    lines = [
        f"{title}: {case.status.upper()}",
        f"  seed {case.seed}, design {case.design.name} {case.design.version}, controller mode {mode}",
    ]
    if case.ran and case.end_time is not None:
        lines.append(f"  ended: {case.end_reason} at {case.end_time:.12g} s")
    elif case.end_reason == "run_error":
        lines.append("  ended: run_error, no recording")
    else:
        lines.append("  not run")
    if case.error is not None:
        lines.append(f"  error: {case.error}")
    for line in case.notes:
        lines.append(f"  note: {line}")
    for line in case.approximations:
        lines.append(f"  approximation: {line}")
    for line in case.crossings:
        lines.append(f"  integrity crossing: {line}")
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


def _title(title: str, case: CaseScore) -> str:
    if case.sweep is not None:
        return f"{title} [{case.sweep[0]} = {case.sweep[1]}]"
    if case.variation is not None:
        drawn = ", ".join(f"{parameter} = {value}" for parameter, value in case.variation)
        return f"{title} [variation seed {case.seed}: {drawn}]"
    return title


def _variation_tables(cases: Sequence[CaseScore]) -> list[str]:
    """Seeded variation runs of a case side by side, one column per variation seed (SCO-019)."""
    groups: dict[str, list[CaseScore]] = {}
    for case in cases:
        if case.variation is not None:
            groups.setdefault(case.case_id, []).append(case)
    lines: list[str] = []
    for case_id, runs in groups.items():
        lines.append("")
        lines.append(f"Variations of {case_id}")
        lines.append("| criterion | " + " | ".join(f"seed {run.seed}" for run in runs) + " |")
        lines.append("|---" * (len(runs) + 1) + "|")
        parameters: list[str] = []
        for run in runs:
            for parameter, _ in run.variation or ():
                if parameter not in parameters:
                    parameters.append(parameter)
        for parameter in parameters:
            cells = [dict(run.variation or ()).get(parameter, "n/a") for run in runs]
            lines.append(f"| {parameter} | " + " | ".join(cells) + " |")
        for name in [result.name for result in runs[0].criteria]:
            lines.append(f"| {name} | " + " | ".join(_cell(run, name) for run in runs) + " |")
    return lines
