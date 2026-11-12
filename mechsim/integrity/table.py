"""Evaluate a configuration over one run and build the per-member table (INT-020 to INT-022) in R's
`IntegrityResults` shape. Table values are at the design load (the recorded loads times the design-load rule's
factor, INT-019): peak stress is the strength check's tier 1 normal (or mount) stress, the safety factor the smallest
minimum-strength safety factor over the strength rows (any tier), deflection the deflection check's peak (0 for a
mount); flags are every row's flags on that subject, sorted."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

from mechsim.integrity.config import IntegrityConfig
from mechsim.integrity.plugin import CheckResult, CheckRow, RunInputs
from mechsim.testing.driver import IntegrityResults, MemberIntegrity

THERMAL_NOTE = "thermal check slot: accepted, not evaluated in v1 (INT-036)"
_STRESS = ("normal_stress", "mount_stress")


@dataclass(frozen=True)
class IntegrityReport(IntegrityResults):
    """The table plus every check's rows, the cycle counts for the wear seam (INT-037) and notes."""

    checks: tuple[CheckResult, ...] = ()
    load_factor: float = 1.0
    notes: tuple[str, ...] = ()

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
            "members": json.loads(self.to_json()),
            "notes": list(self.notes),
        }
        return json.dumps(data, sort_keys=True, indent=1)


def evaluate(config: IntegrityConfig, inputs: RunInputs) -> IntegrityReport:
    results: list[CheckResult] = []
    notes: list[str] = []
    for check, tier, parameters in config.entries():
        result = check.evaluate(inputs, parameters, tier)
        results.append(result)
        notes += result.notes
    if config.thermal is not None:
        notes.append(THERMAL_NOTE)
    members = tuple(_row(subject, results) for subject in _subjects(inputs))
    return IntegrityReport(members, tuple(results), inputs.load_factor, tuple(notes))


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


def _row(subject: str, results: list[CheckResult]) -> MemberIntegrity:
    stress = 0.0
    stress_time = 0.0
    factor = math.inf
    factor_time = 0.0
    for row in _rows(results, "strength", subject):
        if row.quantity in _STRESS and row.tier == 1:
            stress = row.value
            stress_time = row.time
        if row.margin is not None and row.margin.min < factor:
            factor = row.margin.min
            factor_time = row.time
    deflection = 0.0
    deflection_time = 0.0
    for row in _rows(results, "deflection", subject):
        deflection = row.value
        deflection_time = row.time
    flags: set[str] = set()
    for result in results:
        for row in result.rows:
            if row.subject == subject:
                flags.update(row.flags)
    return MemberIntegrity(
        subject, stress, stress_time, deflection, deflection_time, factor, factor_time, tuple(sorted(flags))
    )
