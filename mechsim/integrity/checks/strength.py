"""Strength (INT-005): peak stress per member and mount from the recorded end-loads and, for a lumped member, the
moment at every node spring from the recorded node poses (`beam.member_normal_stress`), safety factor against the
material's direction-dependent strength at the run's ambient temperature. Tier 1: normal stress (axial plus bending
about both axes) against yield; tier 2 adds average shear against shear strength and von Mises against yield, a node
taking the larger end shear (a single-signed transverse load keeps the shear between its end values)."""

from __future__ import annotations

import numpy as np

from mechsim.integrity.beam import member_normal_stress, mount_stress, peak, shear_stress
from mechsim.integrity.plugin import CheckResult, CheckRow, IntegrityCheck, RunInputs, Spread, not_evaluated
from mechsim.schema_base import StrictModel

FLAG = "safety_factor_below_minimum"
ENDS_ONLY = "strength of {member!r}: end moments only, the member has no node poses (INT-005)"


def judged(inputs: RunInputs, margin: Spread) -> tuple[str, ...]:
    """The minimum-safety-factor flag (INT-020) on the conservative (minimum-strength) safety factor."""
    if inputs.minimum_safety_factor is not None and margin.min < inputs.minimum_safety_factor:
        return (FLAG,)
    return ()


class Strength(IntegrityCheck):
    name = "strength"
    tiers = (1, 2)

    def evaluate(self, inputs: RunInputs, parameters: StrictModel, tier: int) -> CheckResult:
        self.check_tier(tier)
        rows: list[CheckRow] = []
        notes: list[str] = []
        for member in inputs.members:
            view = inputs.material(member.material)
            yield_strength = Spread.of(view.strength_spread("yield", inputs.temperature, member.axis_body))
            if member.loads is None:
                rows.append(not_evaluated(member.name, "normal_stress", 1, yield_strength.min))
                if tier >= 2:
                    rows.append(not_evaluated(member.name, "shear_stress", 2))
                    rows.append(not_evaluated(member.name, "von_mises_stress", 2, yield_strength.min))
                continue
            loads = member.loads.scaled(inputs.load_factor)
            if member.nodes is None:
                notes.append(ENDS_ONLY.format(member=member.name))
            normal = member_normal_stress(loads, member.section, member, inputs.bodies, inputs.load_factor)
            value, time = peak(normal, inputs.times)
            margin = yield_strength.over(value)
            rows.append(
                CheckRow(
                    member.name, "normal_stress", value, time, 1, yield_strength.min, margin, judged(inputs, margin)
                )
            )
            if tier >= 2:
                shear = shear_stress(loads, member.section)
                shear_strength = Spread.of(view.strength_spread("shear", inputs.temperature, member.axis_body))
                value, time = peak(shear, inputs.times)
                margin = shear_strength.over(value)
                rows.append(
                    CheckRow(
                        member.name, "shear_stress", value, time, 2, shear_strength.min, margin, judged(inputs, margin)
                    )
                )
                interior = np.repeat(shear.max(axis=1, keepdims=True), normal.shape[1] - 2, axis=1)
                along = np.concatenate((shear, interior), axis=1)
                value, time = peak(np.sqrt(normal**2 + 3.0 * along**2), inputs.times)
                margin = yield_strength.over(value)
                rows.append(
                    CheckRow(
                        member.name,
                        "von_mises_stress",
                        value,
                        time,
                        2,
                        yield_strength.min,
                        margin,
                        judged(inputs, margin),
                    )
                )
        for joint in inputs.joints:
            if joint.kind != "mount" or joint.member is None:
                continue
            member = inputs.member(joint.member)
            view = inputs.material(member.material)
            yield_strength = Spread.of(view.strength_spread("yield", inputs.temperature))
            stress = mount_stress(joint.force * inputs.load_factor, joint.moment * inputs.load_factor, member.section)
            value, time = peak(stress, inputs.times)
            margin = yield_strength.over(value)
            rows.append(
                CheckRow(joint.name, "mount_stress", value, time, 1, yield_strength.min, margin, judged(inputs, margin))
            )
        return CheckResult(self.name, tier, tuple(rows), notes=tuple(notes))


CHECK = Strength
