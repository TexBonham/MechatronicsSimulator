"""The per-run damping lines (BLD-040, BLD-041, PHY-037, ruling 80): the ratio a design states is anchored at a
member's unloaded first mode and at a transmission's driven subtree's build-time mass; what a run realises is lower
when it carries more. Each line prints three figures: the ratio stated in the data, exactly as stated; the ratio
realised at build time (the build's model); the ratio realised in this run (the run's model, which adds run-time bodies
such as a rope payload's end fitting `<payload>_shackle` fixed to its carrier, plus the case payload when its carrier
is in the subtree: a payload hung on a rope or welded is parented to the world, so its carrier tells where it hangs).

Per damped member (node damping c, node stiffness k = E I / l): realised zeta = c omega_c / (2 k), omega_c the chain's
first mode with the carried mass at its worst foot point (`structure.lowest_carried_frequency`; omega_1 unloaded, which
returns the stated ratio). Per damped compliant transmission (output damper D, stiffness k): realised zeta = D / (2
sqrt(k m)), m the driven subtree's mass (the build-time m returns the stated ratio)."""

from __future__ import annotations

import math

from mechsim.builder import structure
from mechsim.builder.physics import transmission_values
from mechsim.builder.result import BuildResult
from mechsim.library.registry import Registry
from mechsim.physics.neutral import BeltPulley, LeadScrew, LumpedMember, PhysicsModel

CasePayload = tuple[str, str, float, bool]


def _subtree(model: PhysicsModel, root: str) -> tuple[set[str], float]:
    """The bodies of `root`'s subtree and their mass; the model lists every parent before its children."""
    names = {root}
    total = 0.0
    for body in model.bodies:
        if body.name in names or body.parent in names:
            names.add(body.name)
            total += float(body.mass)
    return names, total


class _Carried:
    """What a set of subtrees carries at build time and in the run."""

    def __init__(self, built: PhysicsModel, run: PhysicsModel, roots: tuple[str, ...], payload: CasePayload | None):
        self.built = 0.0
        self.run = 0.0
        built_names: set[str] = set()
        run_names: set[str] = set()
        for root in roots:
            names, mass = _subtree(built, root)
            built_names |= names
            self.built += mass
            names, mass = _subtree(run, root)
            run_names |= names
            self.run += mass
        self.added = sorted(run_names - built_names)
        self.payload = "this run has no case payload"
        if payload is not None:
            name, carrier, mass, _ = payload
            self.added = [body for body in self.added if body != name]
            label = f"case payload '{name}' {float(mass):.6g} kg on '{carrier}'"
            self.payload = f"its {label} is not on it"
            if name in run_names:
                self.payload = f"with the {label} as a body of the subtree"
            elif carrier in run_names:
                self.run += float(mass)
                self.payload = f"with the {label} attached"

    def describe(self) -> str:
        bodies = "no run-time bodies"
        if len(self.added) > 0:
            bodies = "run-time bodies " + ", ".join(f"'{name}'" for name in self.added)
        return f"{bodies}; {self.payload}"


def _member_ratio(member: LumpedMember, carried: float) -> float:
    section = member.section
    length = float(section.length)
    segments = int(member.segments)
    flexural = float(section.elastic_modulus) * float(section.second_moment_of_area)
    per_length = float(section.density) * float(section.area)
    omega = structure.loaded_first_frequency(member.root, member.end, segments, length, flexural, per_length)
    if carried > 0.0:
        omega, _ = structure.lowest_carried_frequency(
            member.root, member.end, segments, length, flexural, per_length, carried
        )
    return float(member.node_damping) * omega / (2.0 * flexural * segments / length)


def _stated(value: float | None) -> str:
    if value is None:
        return "not found in the build"
    return repr(value)


def _member_line(
    built: PhysicsModel, run: PhysicsModel, member: LumpedMember, stated: float | None, payload: CasePayload | None
) -> str:
    carried = _Carried(built, run, tuple(member.carries), payload)
    title = member.name
    if member.structural_member is not None:
        title = member.structural_member
    return (
        f"flexible body '{title}': damping ratio stated {_stated(stated)} at the unloaded first mode; zeta = c "
        f"omega_c / (2 k) realised at build time {_member_ratio(member, carried.built):.6g} (carried mass "
        f"{carried.built:.6g} kg) and realised in this run {_member_ratio(member, carried.run):.6g} (carried mass "
        f"{carried.run:.6g} kg: {carried.describe()}); omega_c is the chain's first mode with the carried mass at its "
        "worst foot point, an estimate of the moving carriage's mode (BLD-040, PHY-037, ruling 80)"
    )


def _transmission_line(
    built: PhysicsModel,
    run: PhysicsModel,
    transmission: BeltPulley | LeadScrew,
    stated: float | None,
    payload: CasePayload | None,
) -> str:
    driven = next(
        body for body in run.bodies if body.joint is not None and body.joint.name == transmission.driven_joint
    )
    carried = _Carried(built, run, (driven.name,), payload)
    damping = float(transmission.damping)
    stiffness = float(transmission.stiffness)
    at_build = damping / (2.0 * math.sqrt(stiffness * carried.built))
    in_run = damping / (2.0 * math.sqrt(stiffness * carried.run))
    return (
        f"transmission '{transmission.name}': damping ratio stated {_stated(stated)} on the driven subtree's "
        f"build-time mass; zeta = D / (2 sqrt(k m)) realised at build time {at_build:.6g} (m = "
        f"{carried.built:.6g} kg) and realised in this run {in_run:.6g} (m = {carried.run:.6g} kg: "
        f"{carried.describe()}); D = {damping:.6g} N s/m (BLD-041, PHY-037, ruling 80)"
    )


def stated_ratios(build: BuildResult, registry: Registry) -> dict[str, float]:
    """The damping ratio each lumped member body and each transmission instance states in the data, by name."""
    stated: dict[str, float] = {}
    for record in build.lumped:
        if record.damping_ratio is not None:
            stated[record.body] = float(record.damping_ratio)
    for component in build.design.components:
        if registry.has_type(component.type) and registry.component_type(component.type).category == "transmission":
            stated[component.name] = float(transmission_values(component, registry).get("damping_ratio", 0.0))
    return stated


def run_damping_lines(
    built: PhysicsModel, run: PhysicsModel, payload: CasePayload | None, stated: dict[str, float]
) -> tuple[str, ...]:
    """One line per damped lumped member and damped compliant transmission of the run's model `run` (the build's
    model `built` with the run's bodies and case payload added by `builder.payload.with_payload`); `payload` is the
    case payload as (body, carrier body, mass kg, attached at the start), None for none; `stated` from
    `stated_ratios`."""
    lines: list[str] = []
    for member in run.lumped_members:
        if float(member.node_damping) > 0.0:
            lines.append(_member_line(built, run, member, stated.get(member.parent), payload))
    for transmission in run.transmissions:
        if isinstance(transmission, BeltPulley | LeadScrew) and float(transmission.damping) > 0.0:
            lines.append(_transmission_line(built, run, transmission, stated.get(transmission.name), payload))
    return tuple(lines)
