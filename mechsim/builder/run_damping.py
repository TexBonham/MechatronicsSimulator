"""The per-run damping lines (BLD-040, BLD-041, PHY-037, ruling 80): the ratio a design states is anchored at a
member's unloaded first mode and at a transmission's driven subtree's build-time mass; what a run realises is lower
when it carries more. A case payload is added at run time (`builder.payload`), and one hung on a rope or releasable is
parented to the world, outside every subtree, so its share is found from its carrier body.

Per damped member: stated zeta = c omega_1 / (2 k) (the inverse of `structure.node_damping`); realised zeta omega_c /
omega_1 with omega_c the chain's first mode with the carried mass at its worst foot point
(`structure.lowest_carried_frequency`), at build time and with the run's payload added when its carrier is on a carried
subtree. Per damped compliant transmission: stated zeta = D / (2 sqrt(k m)), m the driven subtree's mass without the
payload; realised zeta sqrt(m / (m + m_p)) when the payload's carrier is in that subtree (D fixed, m raised)."""

from __future__ import annotations

import math

from mechsim.builder import structure
from mechsim.physics.neutral import BeltPulley, LeadScrew, LumpedMember, PhysicsModel

CasePayload = tuple[str, str, float, bool]


def _subtree(model: PhysicsModel, root: str, skip: str) -> tuple[set[str], float]:
    """The bodies of `root`'s subtree and their mass, leaving out the payload body `skip` and its descendants."""
    names = {root}
    total = 0.0
    for body in model.bodies:
        if body.name == skip:
            continue
        if body.name in names or body.parent in names:
            names.add(body.name)
            total += float(body.mass)
    return names, total


class _Run:
    def __init__(self, payload: CasePayload | None) -> None:
        self.name = ""
        self.carrier = ""
        self.mass = 0.0
        self.label = "this run has no case payload"
        if payload is not None:
            self.name = payload[0]
            self.carrier = payload[1]
            self.mass = float(payload[2])
            self.label = f"case payload '{self.name}' {self.mass:.6g} kg on '{self.carrier}'"

    def carried_by(self, names: set[str]) -> bool:
        return self.carrier != "" and self.carrier in names

    def outside(self) -> str:
        if self.carrier == "":
            return self.label
        return f"its {self.label} is not on it"


def _realised(member: LumpedMember, ratio: float, omega: float, carried: float) -> float:
    if carried <= 0.0:
        return ratio
    section = member.section
    lowest, _ = structure.lowest_carried_frequency(
        member.root,
        member.end,
        int(member.segments),
        float(section.length),
        float(section.elastic_modulus) * float(section.second_moment_of_area),
        float(section.density) * float(section.area),
        carried,
    )
    return ratio * lowest / omega


def _member_line(model: PhysicsModel, member: LumpedMember, run: _Run) -> str:
    section = member.section
    flexural = float(section.elastic_modulus) * float(section.second_moment_of_area)
    length = float(section.length)
    segments = int(member.segments)
    omega = structure.loaded_first_frequency(
        member.root, member.end, segments, length, flexural, float(section.density) * float(section.area)
    )
    ratio = float(member.node_damping) * omega / (2.0 * flexural * segments / length)
    names: set[str] = set()
    carried = 0.0
    for name in member.carries:
        found, mass = _subtree(model, name, run.name)
        names |= found
        carried += mass
    built = _realised(member, ratio, omega, carried)
    text = f"{built:.6g} (the same, {run.outside()})"
    if run.carried_by(names):
        loaded = carried + run.mass
        realised = _realised(member, ratio, omega, loaded)
        text = f"{realised:.6g} with the {run.label} attached (carried mass {loaded:.6g} kg)"
    title = member.name
    if member.structural_member is not None:
        title = member.structural_member
    return (
        f"flexible body '{title}': damping ratio stated {ratio:.6g} at the unloaded first mode omega_1 = {omega:.6g} "
        f"rad/s; realised zeta omega_c / omega_1 = {built:.6g} at build time (carried mass {carried:.6g} kg) and "
        f"{text} in this run; omega_c is the chain's first mode with the carried mass at its worst foot point, an "
        "estimate of the moving carriage's mode (BLD-040, PHY-037, ruling 80)"
    )


def _transmission_line(model: PhysicsModel, transmission: BeltPulley | LeadScrew, run: _Run) -> str:
    driven = next(
        body for body in model.bodies if body.joint is not None and body.joint.name == transmission.driven_joint
    )
    names, mass = _subtree(model, driven.name, run.name)
    damping = float(transmission.damping)
    ratio = damping / (2.0 * math.sqrt(float(transmission.stiffness) * mass))
    text = f"{ratio:.6g} ({run.outside()})"
    if run.carried_by(names):
        text = f"{ratio * math.sqrt(mass / (mass + run.mass)):.6g} with the {run.label} attached"
    return (
        f"transmission '{transmission.name}': damping ratio stated {ratio:.6g} on the driven subtree's build-time mass "
        f"m = {mass:.6g} kg (D = {damping:.6g} N s/m); realised zeta sqrt(m / (m + m_p)) = {text} in this run "
        "(BLD-041, PHY-037, ruling 80)"
    )


def run_damping_lines(model: PhysicsModel, payload: CasePayload | None) -> tuple[str, ...]:
    """One line per damped lumped member and damped compliant transmission of the run's model (with its case payload
    added by `builder.payload.with_payload`); `payload` is the case payload as (body, carrier body, mass kg, attached
    at the start), None for none."""
    run = _Run(payload)
    lines: list[str] = []
    for member in model.lumped_members:
        if float(member.node_damping) > 0.0:
            lines.append(_member_line(model, member, run))
    for transmission in model.transmissions:
        if isinstance(transmission, BeltPulley | LeadScrew) and float(transmission.damping) > 0.0:
            lines.append(_transmission_line(model, transmission, run))
    return tuple(lines)
