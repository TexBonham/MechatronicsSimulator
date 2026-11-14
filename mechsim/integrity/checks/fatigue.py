"""Fatigue (INT-008, INT-037): rainflow cycle counting (ASTM E1049-85, three-point method; residual ranges count
as half cycles) on each member's signed normal stress at the four corners of its governing section along the member
(`beam.member_sections`: the strength check's ends, node springs and moment under a carried load; the eight
end-section corners for a member without node poses, ruling 72), Miner damage sum
D = sum n_i / N(S_a,i) against the material's S-N curve with S_a = range / 2 and no mean-stress correction. N(S) is
log-log linear between the S-N points; below the curve's last amplitude the life is infinite (no damage); above its
first amplitude the first sloped segment is extended. The governing section is the one with the largest damage over
its corners (ruling 75; the larger cycle count on a tie, then the first), reported in the row's detail `section` (and
`position` where the section is fixed); a read under a carried load that lacks a value in some row is not evaluated,
with a line, never read as zero. The governing history's cycle count is recorded for the wear seam (INT-037)."""

from __future__ import annotations

import itertools
import math
from collections.abc import Sequence

import numpy as np
from pydantic import Field

from mechsim.integrity.beam import member_sections
from mechsim.integrity.plugin import (
    CheckResult,
    CheckRow,
    FloatArray,
    IntegrityCheck,
    IntegrityError,
    RunInputs,
    Spread,
    not_evaluated,
)
from mechsim.schema_base import StrictModel

FLAG = "fatigue_damage_over_limit"


class FatigueParameters(StrictModel):
    damage_limit: float = Field(default=1.0, gt=0.0, description="Miner sum at which the member is flagged.")


def reversals(values: Sequence[float] | FloatArray) -> list[float]:
    """Turning points of a history: the first and last samples and every slope sign change; repeats dropped."""
    points: list[float] = []
    for value in values:
        number = float(value)
        if points and number == points[-1]:
            continue
        if len(points) >= 2 and (points[-1] - points[-2]) * (number - points[-1]) > 0.0:
            points[-1] = number
            continue
        points.append(number)
    return points


def rainflow(values: Sequence[float] | FloatArray) -> list[tuple[float, float]]:
    """(range, count) pairs, count 1.0 for a full cycle and 0.5 for a half cycle."""
    out: list[tuple[float, float]] = []
    stack: list[float] = []
    for point in reversals(values):
        stack.append(point)
        while len(stack) >= 3:
            latest = abs(stack[-1] - stack[-2])
            previous = abs(stack[-2] - stack[-3])
            if latest < previous:
                break
            if len(stack) == 3:
                out.append((previous, 0.5))
                stack.pop(0)
            else:
                out.append((previous, 1.0))
                del stack[-3:-1]
    for index in range(len(stack) - 1):
        out.append((abs(stack[index + 1] - stack[index]), 0.5))
    return out


def cycles_to_failure(curve: Sequence[tuple[float, float]], amplitude: float) -> float:
    """N at stress amplitude S from (cycles, amplitude) points, cycles increasing and amplitude non-increasing."""
    if len(curve) < 2:
        raise IntegrityError("S-N curve needs at least two points")
    if amplitude <= 0.0 or amplitude < curve[-1][1]:
        return math.inf
    if amplitude > curve[0][1]:
        for (n0, s0), (n1, s1) in itertools.pairwise(curve):
            if s0 > s1:
                exponent = math.log(n1 / n0) / math.log(s0 / s1)
                first_n, first_s = curve[0]
                return float(first_n * (first_s / amplitude) ** exponent)
        return curve[0][0]
    found = 0.0
    for (n0, s0), (n1, s1) in itertools.pairwise(curve):
        if s0 > s1 and s1 <= amplitude <= s0:
            weight = math.log(s0 / amplitude) / math.log(s0 / s1)
            found = max(found, math.exp(math.log(n0) + weight * math.log(n1 / n0)))
        elif s0 == s1 == amplitude:
            found = max(found, n1)
    return found


def damage(curve: Sequence[tuple[float, float]], history: Sequence[float] | FloatArray) -> tuple[float, float]:
    """(Miner sum, cycle count) of one stress history."""
    total = 0.0
    count = 0.0
    for stress_range, cycles in rainflow(history):
        count += cycles
        life = cycles_to_failure(curve, stress_range / 2.0)
        if math.isfinite(life):
            total += cycles / life
    return total, count


def missing_note(member: str, index: int, missing: int, rows: int) -> str:
    return (
        f"fatigue of {member!r}: section {index} (a read under a carried load) has no value in {missing} of {rows} "
        "rows, not evaluated (a missing row is never read as zero, INT-008)"
    )


class Fatigue(IntegrityCheck):
    name = "fatigue"
    parameters = FatigueParameters

    def evaluate(self, inputs: RunInputs, parameters: StrictModel, tier: int) -> CheckResult:
        self.check_tier(tier)
        assert isinstance(parameters, FatigueParameters)
        rows: list[CheckRow] = []
        counts: list[tuple[str, float]] = []
        notes: list[str] = []
        end_time = 0.0
        if inputs.times.size:
            end_time = float(inputs.times[-1])
        for member in inputs.members:
            if member.loads is None:
                rows.append(not_evaluated(member.name, "fatigue_damage", 1, parameters.damage_limit))
                continue
            curve = inputs.material(member.material).sn_curve()
            governing = (0.0, 0.0)
            chosen = None
            sections = member_sections(
                member.loads.scaled(inputs.load_factor),
                member.section,
                member,
                inputs.bodies,
                inputs.load_factor,
                member.spring_offsets,
            )
            for candidate in sections:
                missing = int(candidate.present.size - np.count_nonzero(candidate.present))
                if missing > 0:
                    notes.append(missing_note(member.name, candidate.index, missing, candidate.present.size))
                    continue
                for history in candidate.corners:
                    found = damage(curve, history)
                    if (
                        chosen is None
                        or found[0] > governing[0]
                        or (found[0] == governing[0] and found[1] > governing[1])
                    ):
                        governing = found
                        chosen = candidate
            flags: tuple[str, ...] = ()
            if governing[0] >= parameters.damage_limit:
                flags = (FLAG,)
            margin: Spread | None = None
            if governing[0] > 0.0:
                ratio = parameters.damage_limit / governing[0]
                margin = Spread(ratio, ratio, ratio)
            detail: tuple[tuple[str, float], ...] = (("cycles", governing[1]),)
            if chosen is not None and chosen.index >= 0:
                detail = (*detail, ("section", float(chosen.index)))
                if math.isfinite(chosen.position):
                    detail = (*detail, ("position", chosen.position))
            rows.append(
                CheckRow(
                    member.name,
                    "fatigue_damage",
                    governing[0],
                    end_time,
                    1,
                    parameters.damage_limit,
                    margin,
                    flags,
                    detail,
                )
            )
            counts.append((member.name, governing[1]))
        return CheckResult(self.name, tier, tuple(rows), tuple(counts), tuple(notes))


CHECK = Fatigue
