"""Impact (INT-011): impact events are load steps in the recording, a rise of a member's largest end force or a
mount's reaction by more than `step_threshold` between consecutive samples; a new event starts only after the
previous one's window. Over each event's window the peak stress along the member (the strength check's
`member_normal_stress`: ends, node springs, the moment under a carried load; ruling 72) times
`impact_factor` is judged against the ultimate strength."""

from __future__ import annotations

import numpy as np
from pydantic import Field

from mechsim.integrity.beam import member_normal_stress, mount_stress, peak
from mechsim.integrity.plugin import (
    CheckResult,
    CheckRow,
    FloatArray,
    ForceQuantity,
    IntegrityCheck,
    RunInputs,
    SectionProperties,
    Spread,
    TimeQuantity,
    not_evaluated,
)
from mechsim.schema_base import StrictModel

FLAG = "impact_margin_below_minimum"


def judged(inputs: RunInputs, margin: Spread) -> tuple[str, ...]:
    if inputs.minimum_safety_factor is not None and margin.min < inputs.minimum_safety_factor:
        return (FLAG,)
    return ()


class ImpactParameters(StrictModel):
    impact_factor: float = Field(ge=1.0, description="Multiplier on the event's peak stress.")
    step_threshold: ForceQuantity = Field(description="Rise between consecutive samples that marks an impact event.")
    window: TimeQuantity = Field(description="Time after the event's first sample over which its peak is taken.")


def events(load: FloatArray, times: FloatArray, threshold: float, window: float) -> list[int]:
    """Row indices where `load` rises by more than `threshold` from the previous sample, one per window."""
    found: list[int] = []
    for index in range(1, load.shape[0]):
        if load[index] - load[index - 1] <= threshold:
            continue
        if found and times[index] <= times[found[-1]] + window:
            continue
        found.append(index)
    return found


class Impact(IntegrityCheck):
    name = "impact"
    parameters = ImpactParameters

    def _rows(
        self,
        subject: str,
        load: FloatArray,
        stress: FloatArray,
        ultimate: Spread,
        inputs: RunInputs,
        parameters: ImpactParameters,
    ) -> list[CheckRow]:
        rows: list[CheckRow] = []
        times = inputs.times
        for index in events(load, times, parameters.step_threshold, parameters.window):
            inside = (times >= times[index]) & (times <= times[index] + parameters.window)
            value, time = peak(stress[inside] * parameters.impact_factor, times[inside])
            margin = ultimate.over(value)
            detail = (("event_time", float(times[index])), ("impact_factor", parameters.impact_factor))
            rows.append(
                CheckRow(subject, "impact_stress", value, time, 1, ultimate.min, margin, judged(inputs, margin), detail)
            )
        return rows

    def evaluate(self, inputs: RunInputs, parameters: StrictModel, tier: int) -> CheckResult:
        self.check_tier(tier)
        assert isinstance(parameters, ImpactParameters)
        rows: list[CheckRow] = []
        for member in inputs.members:
            if member.loads is None:
                rows.append(not_evaluated(member.name, "impact_stress", 1))
                continue
            loads = member.loads.scaled(inputs.load_factor)
            load = np.maximum(np.linalg.norm(loads.start_force, axis=1), np.linalg.norm(loads.end_force, axis=1))
            stress = member_normal_stress(
                loads, member.section, member, inputs.bodies, inputs.load_factor, member.spring_offsets
            ).max(axis=1)
            view = inputs.material(member.material)
            ultimate = Spread.of(view.strength_spread("ultimate", inputs.temperature, member.axis_body))
            rows += self._rows(member.name, load, stress, ultimate, inputs, parameters)
        for joint in inputs.joints:
            if joint.kind != "mount" or joint.member is None:
                continue
            held = inputs.member(joint.member)
            section: SectionProperties = held.section
            force = joint.force * inputs.load_factor
            stress = mount_stress(force, joint.moment * inputs.load_factor, section)
            ultimate = Spread.of(inputs.material(held.material).strength_spread("ultimate", inputs.temperature))
            rows += self._rows(joint.name, np.linalg.norm(force, axis=1), stress, ultimate, inputs, parameters)
        return CheckResult(self.name, tier, tuple(rows))


CHECK = Impact
