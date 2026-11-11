"""`build`: a validated design to the neutral physics model, wiring table, chip configuration and member list."""

from __future__ import annotations

from collections.abc import Collection, Mapping
from pathlib import Path

from mechsim.builder.physics import Collector, build_physics
from mechsim.builder.result import BuildError, BuildResult
from mechsim.builder.wiring import build_wiring
from mechsim.design.cad import StepImporter
from mechsim.design.chip import Chip, shipped_chips
from mechsim.design.schema import Design
from mechsim.library.registry import Registry
from mechsim.materials.schema import Material

UNSET_BLOB = "0" * 40

# KER-009 tick order: physics, tier 1 integrity, sensors, controller, signal layer, then the rail (power and its
# safety gates), drivers, steppers and transmissions; the recorder last. Instances keep design order in a group.
COMPONENT_GROUPS: tuple[tuple[str, ...], ...] = (("power", "safety"), ("driver",), ("actuator",), ("transmission",))


def module_order(design: Design, registry: Registry) -> tuple[str, ...]:
    """Module instances in the KER-009 tick order, the order the runner spawns seeds in (D1)."""

    def category(name: str) -> str:
        return registry.component_type(name).category

    components = [component for component in design.components if registry.has_type(component.type)]
    order = ["physics", "integrity"]
    order += [component.name for component in components if category(component.type) in ("sensor", "front_end")]
    order += ["controller", "signals"]
    for group in COMPONENT_GROUPS:
        order += [component.name for component in components if category(component.type) in group]
    placed = set(order)
    order += [component.name for component in components if component.name not in placed]
    order.append("recorder")
    return tuple(order)


def build(
    design: Design,
    registry: Registry,
    materials: Mapping[str, Material],
    *,
    chips: Mapping[str, Chip] | None = None,
    supported_elements: Collection[str] | None = None,
    adapter: str = "any",
    step_importer: StepImporter | None = None,
    base: Path | None = None,
    blob: str | None = None,
    rigid_members: bool = False,
) -> BuildResult:
    """Build `design`; raise `BuildError` listing every failure, emitting nothing (BLD-025).

    `supported_elements` (element kinds, `physics.element_kinds`) is the target adapter's list; None accepts all.
    `blob` is the design file's git blob hash (D3); it defaults to the design's source_hash, or zeros when unset.
    """
    collector = Collector()
    stamp = blob
    if stamp is None:
        stamp = design.source_hash
        if stamp == "unset":
            stamp = UNSET_BLOB
    if chips is None:
        chips = shipped_chips()
    physics = build_physics(
        design,
        registry,
        dict(materials),
        collector,
        blob=stamp,
        supported_elements=supported_elements,
        adapter=adapter,
        step_importer=step_importer,
        base=base,
        rigid_members=rigid_members,
    )
    wiring, chip = build_wiring(design, registry, chips, collector)
    if len(collector.failures) > 0:
        raise BuildError(collector.failures)
    if len(collector.pending) > 0 and design.status == "release":
        raise BuildError([f"release design holds an open value at {path}" for path in collector.pending])
    return BuildResult(
        design=design,
        module_instances=module_order(design, registry),
        physics=physics.model,
        wiring=wiring,
        chip=chip,
        members=physics.members,
        lumped=physics.lumped,
        pending=tuple(sorted(set(collector.pending))),
        notes=tuple(collector.notes),
    )
