"""Fastener (INT-010): each rated fastening's peak load from its joint's recorded reaction force, against the grade's
rating from the configuration's `ratings` (catalog grade ratings are owed, ruling 27), margin = rating / peak. Which
fastenings are rated is `rating`, the one rule the tier 1 limits table uses too: every fastener set (a rating
required) and every mount given one."""

from __future__ import annotations

from collections.abc import Mapping

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


def rating(name: str, kind: str, ratings: Mapping[str, float]) -> float | None:
    """The load rating a fastening is judged against (INT-010, INT-023), None when it is not rated: a fastener set must
    have a positive rating, a mount is rated when the configuration gives it one; a rating at or below zero is an
    error either way."""
    value = ratings.get(name)
    if value is None and kind != "fastener":
        return None
    if value is None or value <= 0.0:
        raise IntegrityError(f"fastener {name!r}: no load rating")
    return float(value)


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
            limit = rating(joint.name, joint.kind, parameters.ratings)
            if limit is None:
                continue
            value, time = peak(np.linalg.norm(joint.force * inputs.load_factor, axis=1), inputs.times)
            ratio = float("inf")
            if value > 0.0:
                ratio = limit / value
            minimum = 1.0
            if inputs.minimum_safety_factor is not None:
                minimum = inputs.minimum_safety_factor
            flags: tuple[str, ...] = ()
            if ratio < minimum:
                flags = (FLAG,)
            rows.append(
                CheckRow(joint.name, "fastener_load", value, time, 1, limit, Spread(ratio, ratio, ratio), flags)
            )
        return CheckResult(self.name, tier, tuple(rows))


CHECK = Fastener
