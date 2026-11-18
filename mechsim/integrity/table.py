"""Evaluate a configuration over one run and build the per-member table (INT-020 to INT-022) in R's
`IntegrityResults` shape. Table values are at the design load (the recorded loads times the design-load rule's
factor, INT-019): peak stress is the strength check's tier 1 normal (or mount) stress, the safety factor the smallest
minimum-strength safety factor over the strength rows (any tier), deflection the deflection check's peak (0 for a
mount); flags are every row's flags on that subject, sorted."""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from dataclasses import dataclass, replace
from typing import Any

from mechsim.integrity.beam import peak
from mechsim.integrity.checks.deflection import NOT_EVALUATED as STATIC
from mechsim.integrity.config import IntegrityConfig
from mechsim.integrity.plugin import LOADS_NOT_EVALUATED, CheckResult, CheckRow, RunInputs, rigid_note
from mechsim.record.validity import NOT_VALID_FLAG, NOT_VALID_KIND, CarriedValidity
from mechsim.testing.driver import IntegrityResults, MemberIntegrity, RopeIntegrity

THERMAL_NOTE = "thermal check slot: accepted, not evaluated in v1 (INT-036)"
SINGLE_AXIS = "approximation: single-axis section"
SNAP = "snap"
DESIGN_LOAD = "design_load"
_STRESS = ("normal_stress", "mount_stress")
CARRIED_NOTE = (
    "design load: the case carries it (integrity.design_load_carried), so the table applies factor 1, not the "
    "design's rule (TST-043, INT-019, ruling 52)"
)


def window_note(start: float) -> str:
    """The line of an evaluation start (TST-044)."""
    return (
        f"integrity evaluation from {start:.12g} s (integrity.evaluate_from): peaks, safety factors and tier 1 "
        "crossings before it do not count; a tier 1 crossing before it is recorded as an "
        "integrity_before_evaluation event and does not fail the case (TST-044)"
    )


@dataclass(frozen=True)
class IntegrityReport(IntegrityResults):
    """The table plus every check's rows, the cycle counts for the wear seam (INT-037), notes and the recorded sample
    times the checks' `series` are taken at."""

    checks: tuple[CheckResult, ...] = ()
    load_factor: float = 1.0
    notes: tuple[str, ...] = ()
    times: tuple[float, ...] = ()

    def deflection_series(self, member: str) -> tuple[tuple[float, ...], tuple[float, ...]] | None:
        """The member's deflection at each recorded sample (m, design load; SCO-027), with the sample times; None
        when the deflection check did not evaluate it."""
        for result in self.checks:
            if result.check != "deflection":
                continue
            for subject, values in result.series:
                if subject == member:
                    return self.times, values
        return None

    def cycle_counts(self) -> dict[str, float]:
        out: dict[str, float] = {}
        for result in self.checks:
            for subject, cycles in result.cycle_counts:
                out[subject] = cycles
        return out

    def report_json(self) -> str:
        data: dict[str, Any] = {
            "checks": [
                {"check": result.check, "rows": [row.to_dict() for row in result.rows], "tier": result.tier}
                for result in self.checks
            ],
            "cycle_counts": self.cycle_counts(),
            "load_factor": self.load_factor,
            "table": json.loads(self.to_json()),
            "notes": list(self.notes),
        }
        return json.dumps(data, sort_keys=True, indent=1)


def not_valid_note(record: CarriedValidity) -> str:
    """The line of a member the engine flagged (rulings 69, 72)."""
    return (
        f"integrity: {record.reason()}; its rows count the samples up to then and are flagged {NOT_VALID_FLAG}, its "
        f"tier 1 crossings after then are recorded as {NOT_VALID_KIND} events, never counted (rulings 69, 72)"
    )


def evaluate(
    config: IntegrityConfig,
    inputs: RunInputs,
    lines: Sequence[str] = (),
    validity: Sequence[CarriedValidity] = (),
) -> IntegrityReport:
    """Every configured check over `inputs`; `lines` (the case's design-load and evaluation-start lines) lead the
    notes. `validity` (the engine's carried-load record, ruling 69): a flagged member's rows (and its mounts') are its
    checks over the samples up to the flag time (values before it stand), flagged `not_valid` with the detail
    `not_valid_from`; with no sample before it they are NaN; its deflection history is NaN after it; one line says
    so. Without a flag the result is the same, bit for bit."""
    results: list[CheckResult] = []
    notes: list[str] = list(lines)
    for check, tier, parameters in config.entries():
        result = check.evaluate(inputs, parameters, tier)
        results.append(result)
        notes += result.notes
    names = {member.name for member in inputs.members}
    for record in validity:
        if record.flagged and record.member in names:
            results = _not_valid(config, inputs, results, record)
            notes.append(not_valid_note(record))
    for member in inputs.members:
        if member.loads is None:
            unevaluated = [result.check for result in results if _rigid(result, member.name)]
            notes.append(rigid_note(member.name, unevaluated))
    if config.thermal is not None:
        notes.append(THERMAL_NOTE)
    single = {member.name for member in inputs.members if member.section.single_axis}
    members = tuple(_row(subject, results, subject in single) for subject in _subjects(inputs))
    times = tuple(float(time) for time in inputs.times)
    return IntegrityReport(members, _ropes(inputs), tuple(results), inputs.load_factor, tuple(notes), times)


def _not_valid(
    config: IntegrityConfig, inputs: RunInputs, results: list[CheckResult], record: CarriedValidity
) -> list[CheckResult]:
    subjects = {record.member}
    subjects.update(joint.name for joint in inputs.joints if joint.member == record.member and joint.kind == "mount")
    before = inputs.until(record.since)
    marks = (("not_valid_from", record.since),)
    out: list[CheckResult] = []
    for (check, tier, parameters), result in zip(config.entries(), results, strict=True):
        kept: tuple[CheckRow, ...] = ()
        counts = result.cycle_counts
        if before.times.size > 0:
            cut = check.evaluate(before, parameters, tier)
            kept = tuple(row for row in cut.rows if row.subject in subjects)
            counts = tuple(item for item in result.cycle_counts if item[0] not in subjects) + tuple(
                item for item in cut.cycle_counts if item[0] in subjects
            )
        else:
            kept = tuple(replace(row, value=math.nan, margin=None) for row in result.rows if row.subject in subjects)
        marked = tuple(
            replace(row, flags=tuple(sorted({*row.flags, NOT_VALID_FLAG})), detail=row.detail + marks) for row in kept
        )
        rows = tuple(row for row in result.rows if row.subject not in subjects) + marked
        series_out: list[tuple[str, tuple[float, ...]]] = []
        for subject, values in result.series:
            if subject not in subjects:
                series_out.append((subject, values))
                continue
            masked: list[float] = []
            for time, value in zip(inputs.times, values, strict=True):
                if record.covers(float(time)):
                    masked.append(math.nan)
                else:
                    masked.append(value)
            series_out.append((subject, tuple(masked)))
        series = tuple(series_out)
        out.append(replace(result, rows=rows, cycle_counts=counts, series=series))
    return out


def _rigid(result: CheckResult, subject: str) -> bool:
    for row in result.rows:
        if row.subject == subject and (LOADS_NOT_EVALUATED in row.flags or STATIC in row.flags):
            return True
    return False


def _ropes(inputs: RunInputs) -> tuple[RopeIntegrity, ...]:
    """Per rope: peak of the largest segment tension; `snap` above the breaking tension (none set: never, ruling 19);
    `design_load` above the design-load weight rated_load (1 + allowance) |g| when the design states the rule."""
    out: list[RopeIntegrity] = []
    for rope in inputs.ropes:
        value, time = peak(rope.tension, inputs.times)
        flags: list[str] = []
        if inputs.design_tension is not None and value > inputs.design_tension:
            flags.append(DESIGN_LOAD)
        if rope.breaking_tension is not None and value > rope.breaking_tension:
            flags.append(SNAP)
        out.append(RopeIntegrity(rope.name, value, time, tuple(sorted(flags))))
    return tuple(out)


def _subjects(inputs: RunInputs) -> list[str]:
    names = [member.name for member in inputs.members]
    names += [joint.name for joint in inputs.joints if joint.kind == "mount" and joint.member is not None]
    return names


def _rows(results: list[CheckResult], check: str, subject: str) -> list[CheckRow]:
    out: list[CheckRow] = []
    for result in results:
        if result.check == check:
            out += [row for row in result.rows if row.subject == subject]
    return out


def _row(subject: str, results: list[CheckResult], single_axis: bool) -> MemberIntegrity:
    stress = 0.0
    stress_time = 0.0
    factor = math.inf
    factor_time = 0.0
    unevaluated = False
    for row in _rows(results, "strength", subject):
        if LOADS_NOT_EVALUATED in row.flags:
            unevaluated = True
        if row.quantity in _STRESS and row.tier == 1:
            stress = row.value
            stress_time = row.time
        if row.margin is not None and row.margin.min < factor:
            factor = row.margin.min
            factor_time = row.time
    if unevaluated:
        stress = math.nan
        factor = math.nan
    deflection = 0.0
    deflection_time = 0.0
    for row in _rows(results, "deflection", subject):
        deflection = row.value
        deflection_time = row.time
    flags: set[str] = set()
    if single_axis:
        flags.add(SINGLE_AXIS)
    for result in results:
        for row in result.rows:
            if row.subject == subject:
                flags.update(row.flags)
    return MemberIntegrity(
        subject, stress, stress_time, deflection, deflection_time, factor, factor_time, tuple(sorted(flags))
    )
