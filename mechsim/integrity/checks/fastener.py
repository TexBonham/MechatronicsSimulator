"""Fastener (INT-010): each fastener set's peak load from its joint's recorded reaction force, against the grade's
rating from the configuration's `ratings` (catalog grade ratings are owed, ruling 27), margin = rating / peak."""

from __future__ import annotations

import numpy as np
from pydantic import Field

from mechsim.integrity.beam import peak
from mechsim.integrity.plugin import (
    CheckResult,
    CheckRow,
    ForceQuantity,
    IntegrityCheck,
    IntegrityError,
    RunInputs,
    Spread,
)
from mechsim.schema_base import StrictModel

FLAG = "fastener_margin_below_minimum"


class FastenerParameters(StrictModel):
    ratings: dict[str, ForceQuantity] = Field(default_factory=dict, description="Load rating per fastening name.")


class Fastener(IntegrityCheck):
    name = "fastener"
    parameters = FastenerParameters

    def evaluate(self, inputs: RunInputs, parameters: StrictModel, tier: int) -> CheckResult:
        self.check_tier(tier)
        assert isinstance(parameters, FastenerParameters)
        rows: list[CheckRow] = []
        for joint in inputs.joints:
            if joint.kind != "fastener":
                continue
            rating = parameters.ratings.get(joint.name)
            if rating is None or rating <= 0.0:
                raise IntegrityError(f"fastener {joint.name!r}: no load rating")
            value, time = peak(np.linalg.norm(joint.force * inputs.load_factor, axis=1), inputs.times)
            ratio = float("inf")
            if value > 0.0:
                ratio = rating / value
            minimum = 1.0
            if inputs.minimum_safety_factor is not None:
                minimum = inputs.minimum_safety_factor
            flags: tuple[str, ...] = ()
            if ratio < minimum:
                flags = (FLAG,)
            rows.append(
                CheckRow(joint.name, "fastener_load", value, time, 1, rating, Spread(ratio, ratio, ratio), flags)
            )
        return CheckResult(self.name, tier, tuple(rows))


CHECK = Fastener
