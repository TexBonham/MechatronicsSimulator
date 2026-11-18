"""The mass that moves rigidly with a joint's child body in the neutral model: its joint subtree and the lumped member
chains rooted on it (BLD-041, PHY-037). One count, read by the builder's damping anchor, the run-damping line and the
native translation's mode lines; it depends on the neutral model only."""

from __future__ import annotations

from dataclasses import dataclass

from mechsim.physics.neutral import LumpedMember, PhysicsModel


def chain_mass(member: LumpedMember) -> float:
    """The mass of a lumped member's chain bodies as the engine builds them (`core/engine` lumped beam): segments of
    density x area x span, l = L / N, N - 1 full spans and a half span at the tip, a pinned root adding a half-span pin
    segment (a clamped root leaves its half span on the parent, so rho A (L - l / 2)), and a roller body of
    roller_mass_fraction x rho A L at a roller or clamped far end."""
    section = member.section
    per_length = float(section.density) * float(section.area)
    length = float(section.length)
    segment = length / member.segments
    spans = [segment] * (member.segments - 1) + [0.5 * segment]
    if member.root == "pinned":
        spans = [0.5 * segment, *spans]
    total = 0.0
    for span in spans:
        total += per_length * span
    if member.end != "free":
        total += float(member.roller_mass_fraction) * per_length * length
    return total


@dataclass(frozen=True)
class DrivenSubtree:
    """What moves rigidly with a joint's child `root`: the bodies of its joint subtree, their mass, and the mass of the
    lumped member chains rooted on any of them (each chain's bodies are the engine's children of its parent body, so a
    body a member carries is a child of the member's parent and already in the subtree, ENG-052)."""

    bodies: frozenset[str]
    body_mass: float
    chain_mass: float

    @property
    def mass(self) -> float:
        return self.body_mass + self.chain_mass


def driven_subtree(model: PhysicsModel, root: str) -> DrivenSubtree:
    """`root`'s subtree in `model` (every parent listed before its children) with its member chains (BLD-041, PHY-037);
    the one count of the driven mass for the builder's damping anchor, the run-damping line and the mode line."""
    names = {root}
    total = 0.0
    for body in model.bodies:
        if body.name in names or body.parent in names:
            names.add(body.name)
            total += float(body.mass)
    chains = 0.0
    for member in model.lumped_members:
        if member.parent in names:
            chains += chain_mass(member)
    return DrivenSubtree(frozenset(names), total, chains)


def driven_mass(model: PhysicsModel, root: str) -> float:
    """The mass the engine moves with the joint whose child is `root`: its subtree's bodies and member chains."""
    return driven_subtree(model, root).mass
