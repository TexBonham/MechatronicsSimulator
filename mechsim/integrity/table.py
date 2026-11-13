"""Evaluate a configuration over one run and build the per-member table (INT-020 to INT-022) in R's
`IntegrityResults` shape. Table values are at the design load (the recorded loads times the design-load rule's
factor, INT-019): peak stress is the strength check's tier 1 normal (or mount) stress, the safety factor the smallest
minimum-strength safety factor over the strength rows (any tier), deflection the deflection check's peak (0 for a
mount); flags are every row's flags on that subject, sorted."""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

from mechsim.integrity.beam import peak
from mechsim.integrity.checks.deflection import NOT_EVALUATED as STATIC
from mechsim.integrity.config import IntegrityConfig
from mechsim.integrity.plugin import LOADS_NOT_EVALUATED, CheckResult, CheckRow, RunInputs, rigid_note
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


def evaluate(config: IntegrityConfig, inputs: RunInputs, lines: Sequence[str] = ()) -> IntegrityReport:
    """Every configured check over `inputs`; `lines` (the case's design-load and evaluation-start lines) lead the
    notes."""
    results: list[CheckResult] = []
    notes: list[str] = list(lines)
    for check, tier, parameters in config.entries():
        result = check.evaluate(inputs, parameters, tier)
        results.append(result)
        notes += result.notes
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
