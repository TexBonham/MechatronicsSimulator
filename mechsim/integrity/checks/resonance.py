"""Resonance screening (INT-012): each member's first natural frequency (the design's calibrated value, else the
section's closed form for its end conditions) against the excitation frequencies the run recorded, f = |value| times
`hz_per_unit` of each configured series; flagged when any sample lies within the band |f - f_n| <= band f_n."""

from __future__ import annotations

import numpy as np
from pydantic import Field

from mechsim.integrity.beam import first_frequency
from mechsim.integrity.plugin import CheckResult, CheckRow, IntegrityCheck, IntegrityError, RunInputs
from mechsim.schema_base import StrictModel

FLAG = "resonance"


class ExcitationSource(StrictModel):
    series: str = Field(min_length=1, description="Recorded quantity reference, e.g. `joint:x_axis.velocity`.")
    hz_per_unit: float = Field(gt=0.0, description="Excitation frequency in Hz per unit of the series' SI value.")


class ResonanceParameters(StrictModel):
    band: float = Field(gt=0.0, lt=1.0, description="Screening half-width as a fraction of the natural frequency.")
    excitations: list[ExcitationSource] = Field(default_factory=list)


class Resonance(IntegrityCheck):
    name = "resonance"
    parameters = ResonanceParameters

    def evaluate(self, inputs: RunInputs, parameters: StrictModel, tier: int) -> CheckResult:
        self.check_tier(tier)
        assert isinstance(parameters, ResonanceParameters)
        sources = []
        for source in parameters.excitations:
            try:
                times, values = inputs.series(source.series)
            except KeyError as error:
                raise IntegrityError(f"resonance: the run recorded no series {source.series!r}") from error
            sources.append((source.series, times, np.abs(values) * source.hz_per_unit))
        rows: list[CheckRow] = []
        for member in inputs.members:
            natural = member.natural_frequency
            if natural is None:
                natural = first_frequency(member)
            for name, times, frequency in sources:
                if frequency.size == 0:
                    continue
                distance = np.abs(frequency - natural)
                closest = int(np.argmin(distance))
                inside = np.nonzero(distance <= parameters.band * natural)[0]
                flags: tuple[str, ...] = ()
                time = float(times[closest])
                if inside.size:
                    flags = (FLAG,)
                    time = float(times[int(inside[0])])
                detail = (("band", parameters.band), ("natural_frequency", natural))
                rows.append(
                    CheckRow(
                        member.name,
                        f"excitation:{name}",
                        float(frequency[closest]),
                        time,
                        1,
                        natural,
                        None,
                        flags,
                        detail,
                    )
                )
        return CheckResult(self.name, tier, tuple(rows))


CHECK = Resonance
