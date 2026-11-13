"""Deflection (INT-006, INT-021): each member's peak deflection over the run from the recorded node poses of its lumped
chain (`beam.node_deflection`, round 5 ruling 28), times the design-load factor (linear elasticity, INT-019), against
the design's accuracy target (DES-032). A member built rigid (the bridge) has no node poses: its row is not evaluated
(value NaN, never zero) with a note line saying so."""

from __future__ import annotations

import math

from mechsim.integrity.beam import node_deflection, peak
from mechsim.integrity.plugin import CheckResult, CheckRow, IntegrityCheck, RunInputs
from mechsim.schema_base import StrictModel

FLAG = "deflection_over_target"
NOT_EVALUATED = "deflection_not_evaluated"


def not_evaluated_note(member: str) -> str:
    return f"deflection of {member!r}: not evaluated, the member is built rigid and has no node poses (INT-006)"


class Deflection(IntegrityCheck):
    name = "deflection"

    def evaluate(self, inputs: RunInputs, parameters: StrictModel, tier: int) -> CheckResult:
        self.check_tier(tier)
        rows: list[CheckRow] = []
        notes: list[str] = []
        for member in inputs.members:
            if member.nodes is None or inputs.bodies is None:
                rows.append(
                    CheckRow(
                        member.name, "deflection", math.nan, 0.0, 1, inputs.deflection_limit, None, (NOT_EVALUATED,)
                    )
                )
                notes.append(not_evaluated_note(member.name))
                continue
            value, time = peak(node_deflection(member, inputs.bodies) * inputs.load_factor, inputs.times)
            flags: tuple[str, ...] = ()
            if inputs.deflection_limit is not None and value > inputs.deflection_limit:
                flags = (FLAG,)
            rows.append(CheckRow(member.name, "deflection", value, time, 1, inputs.deflection_limit, None, flags))
        return CheckResult(self.name, tier, tuple(rows), notes=tuple(notes))


CHECK = Deflection
