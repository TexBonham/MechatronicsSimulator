"""PreparedModel assembly for the native adapter (PHY-020): the engine model document goes in the model JSON field,
the engine artifact is empty, and every approximation line rides along (ADR 0003, SCO-013)."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from mechsim.physics.native.translate import translate
from mechsim.physics.neutral import PhysicsModel


@dataclass(frozen=True)
class NativeArtifacts:
    """What `PreparedModel(model_json, "", version)` takes for the native adapter, plus the approximation report."""

    model_json: str
    version: str
    report: tuple[str, ...]


def prepare_model(
    model: PhysicsModel,
    starts: Mapping[str, float] | None = None,
    step: float | None = None,
    payloads: Mapping[str, str] | None = None,
    substeps: int = 1,
) -> NativeArtifacts:
    """The native adapter's prepared model; arguments as the bridge's `prepare_model` (start coordinates of single-DOF
    joints, the run's physics step, payload body to carrier); a payload starts where its carrier's start moves it;
    `substeps` engine steps per adapter step (h / substeps each)."""
    translation = translate(model, step, payloads, starts, substeps)
    return NativeArtifacts(translation.text, model.design.version, translation.report)
