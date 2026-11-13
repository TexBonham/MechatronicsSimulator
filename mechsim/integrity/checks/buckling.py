"""Buckling (INT-007): each compression member's margin P_cr / P over the run, Euler's load
P_cr = pi^2 E I_min / (K L)^2 with K from the member's end conditions or the configuration's override."""

from __future__ import annotations

import math

import numpy as np
from pydantic import Field

from mechsim.integrity.beam import axial, effective_length_factor, peak
from mechsim.integrity.plugin import CheckResult, CheckRow, IntegrityCheck, RunInputs, Spread, not_evaluated
from mechsim.schema_base import StrictModel

FLAG = "buckling_margin_below_minimum"


class BucklingParameters(StrictModel):
    effective_length_factors: dict[str, float] = Field(
        default_factory=dict, description="Per member K overriding the one its end conditions give."
    )


class Buckling(IntegrityCheck):
    name = "buckling"
    parameters = BucklingParameters

    def evaluate(self, inputs: RunInputs, parameters: StrictModel, tier: int) -> CheckResult:
        self.check_tier(tier)
        assert isinstance(parameters, BucklingParameters)
        rows: list[CheckRow] = []
        notes: list[str] = []
        for member in inputs.members:
            if member.loads is None:
                rows.append(not_evaluated(member.name, "compression", 1))
                continue
            compression = np.maximum(-axial(member.loads.scaled(inputs.load_factor)), 0.0)
            value, time = peak(compression, inputs.times)
            if value <= 0.0:
                notes.append(f"buckling: member {member.name!r} is never in compression")
                continue
            factor = parameters.effective_length_factors.get(member.name)
            if factor is None:
                factor = effective_length_factor(member)
            section = member.section
            critical = math.pi**2 * section.elastic_modulus * section.i_min() / (factor * section.length) ** 2
            ratio = critical / value
            flags: tuple[str, ...] = ()
            if inputs.minimum_safety_factor is not None and ratio < inputs.minimum_safety_factor:
                flags = (FLAG,)
            detail = (("critical_load", critical), ("effective_length_factor", factor))
            margin = Spread(ratio, ratio, ratio)
            rows.append(CheckRow(member.name, "compression", value, time, 1, critical, margin, flags, detail))
        return CheckResult(self.name, tier, tuple(rows), notes=tuple(notes))


CHECK = Buckling
