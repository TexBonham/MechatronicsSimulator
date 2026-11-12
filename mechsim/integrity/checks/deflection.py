"""Deflection (INT-006, INT-021): each member's peak deflection over the run from its recorded end-loads and the
section's E I (`beam.deflection`), against the design's accuracy target (DES-032). End-loads rather than chain node
poses: they exist for every member the recorder lists, the closed forms reference them directly, and the node poses
carry the rigid motion of the carrying body that would have to be subtracted first."""

from __future__ import annotations

from mechsim.integrity.beam import deflection, peak
from mechsim.integrity.plugin import CheckResult, CheckRow, IntegrityCheck, RunInputs
from mechsim.schema_base import StrictModel

FLAG = "deflection_over_target"
APPROXIMATION = "approximation: static deflection from end loads"


class Deflection(IntegrityCheck):
    name = "deflection"

    def evaluate(self, inputs: RunInputs, parameters: StrictModel, tier: int) -> CheckResult:
        self.check_tier(tier)
        rows: list[CheckRow] = []
        for member in inputs.members:
            value, time = peak(deflection(member, member.loads.scaled(inputs.load_factor)), inputs.times)
            flags: tuple[str, ...] = (APPROXIMATION,)
            if inputs.deflection_limit is not None and value > inputs.deflection_limit:
                flags = (APPROXIMATION, FLAG)
            rows.append(CheckRow(member.name, "deflection", value, time, 1, inputs.deflection_limit, None, flags))
        return CheckResult(self.name, tier, tuple(rows))


CHECK = Deflection
