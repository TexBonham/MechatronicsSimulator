"""Tier 1 integrity party setup (INT-023, INT-028, ADR 0010, round 5 rulings 27, 34, 35): the limits table the C++
party evaluates, from the build, the run's physics model and `integrity.yaml` beside the design file. Nothing
design-specific reaches C++: every limit arrives as data (invariant 2).

Tier 1 is on when `integrity.yaml` lists any of the checks `strength` (stress), `buckling` (buckling margin),
`deflection` or `fastener` (fastener load); listing `strength` at tier 2 adds the yield offset (ruling 35). A design
without the file, or a file listing none of them, registers no party and runs as before."""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from mechsim.integrity.beam import effective_length_factor
from mechsim.integrity.config import FILE_NAME, IntegrityConfig, load_config
from mechsim.integrity.inputs import member_axes, node_chains, section_properties
from mechsim.integrity.plugin import IntegrityError, MaterialView, MemberInputs, NodeChain, Spread, Wrenches
from mechsim.materials.database import MaterialSet
from mechsim.testing.schema import DESIGNS_ROOT

FORMAT = "mechsim-integrity-limits-1"
PARTY = "integrity"
MODEL_REQUESTS = "integrity.model_requests"
QUANTITIES = (("strength", "stress"), ("buckling", "buckling"), ("deflection", "deflection"), ("fastener", "fastener"))
SPRINGS_PER_JOINT = {"revolute": 1, "prismatic": 1, "planar": 3, "fixed": 0, "spherical": 0, "free": 0}
YIELD_RULE = (
    "integrity tier 2 yield: placeholder plasticity rule (ruling 35): at the member end with the larger bending moment "
    "the nearest node spring takes the permanent offset (peak |M_n| - M_y) / k, M_y = sigma_y I_z / c_y, k = E I / l, "
    "M_n by statics over the half segment (its inertia neglected); planar bending about the member z axis"
)
NOT_EVALUATED = (
    "integrity tier 1: not evaluated on this adapter: its members are rigid (rulings 29 and 34; tier 1 runs on the "
    "native adapter)"
)


@dataclass(frozen=True)
class Tier1Setup:
    """What the driver registers: the limits table (JSON), whether the party publishes yield requests and the run's
    approximation lines. Halting is the test case's field (rulings 4, 50), never this file's."""

    limits: str
    yield_offset: bool
    approximations: tuple[str, ...]


# The native document's `ends` of a member with a pinned root, whose spring-free pin segment publishes node 0.
PINNED_ROOT_ENDS = "simply_supported"


def enabled(config: IntegrityConfig) -> dict[str, bool]:
    """Tier 1 quantities and the yield offset the configuration turns on (listed checks only)."""
    listed = {str(entry.check): int(entry.tier) for entry in config.checks}
    out = {quantity: check in listed for check, quantity in QUANTITIES}
    out["yield_offset"] = listed.get("strength", 0) >= 2
    return out


def tier1_enabled(config: IntegrityConfig) -> bool:
    flags = enabled(config)
    return any(flags[quantity] for _, quantity in QUANTITIES)


def first_springs(native: Mapping[str, Any]) -> list[int]:
    """Each lumped member's first node spring, read from the prepared native model document (the adapter's own spring
    order, `core/physics/native/CLAUDE.md`): every revolute and prismatic DOF (one each) and planar joint (three) of the
    neutral bodies in document order, then each member's N node springs root to tip. A joint type the order does not
    name is an error, never a zero."""
    count = 0
    bodies = native["bodies"]
    for body in bodies[: int(native["neutral_bodies"])]:
        kind = str(body["joint"]["type"])
        if kind not in SPRINGS_PER_JOINT:
            raise IntegrityError(f"body {body['name']!r}: joint type {kind!r} has no place in the spring order")
        count += SPRINGS_PER_JOINT[kind]
    out: list[int] = []
    for member in native["members"]:
        out.append(count)
        count += int(member["segments"])
    return out


def published_names(model: Any, native: Mapping[str, Any]) -> tuple[str, ...]:
    """The bodies the native adapter publishes per tick, by name: the prepared document's neutral bodies (the build's
    plus those a case adds at run time, such as a payload and its shackle), then each lumped member's nodes root to
    tip, `<member>.node<k>`, each member's node count from its document entry (`segments`, plus node 0 for the pin
    segment of a pinned root, `ends` simply_supported; `mechsim.physics.native.member_node_names`), its name from the
    physics model's member at the same place (the document's member list must match it)."""
    names = [str(body["name"]) for body in native["bodies"][: int(native["neutral_bodies"])]]
    entries = list(native["members"])
    if len(entries) != len(model.lumped_members):
        raise IntegrityError(
            f"the native model lists {len(entries)} members, the physics model {len(model.lumped_members)}"
        )
    for lumped, entry in zip(model.lumped_members, entries, strict=True):
        count = int(entry["segments"])
        if entry["ends"] == PINNED_ROOT_ENDS:
            count += 1
        names.extend(f"{lumped.name}.node{k}" for k in range(count))
    return tuple(names)


def _parameters(config: IntegrityConfig, check: str) -> Any:
    for entry in config.checks:
        if str(entry.check) == check:
            return entry.parameters
    return None


def _supports(member: Any) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for support in member.supports:
        condition = str(support.condition)
        if condition == "roller":
            condition = "pinned"
        out.append({"at": float(support.at), "condition": condition})
    return out


def _effective_length(member: Any, section: Any, config: IntegrityConfig, buckling: bool) -> float:
    parameters = _parameters(config, "buckling")
    if parameters is not None and str(member.name) in parameters.effective_length_factors:
        return float(parameters.effective_length_factors[str(member.name)])
    empty = np.zeros((0, 3), dtype=np.float64)
    view = MemberInputs(
        name=str(member.name),
        material=str(member.material),
        section=section,
        supports=tuple((float(s.at), s.condition) for s in member.supports),
        loads=Wrenches(empty, empty, empty, empty),
    )
    try:
        return effective_length_factor(view)
    except IntegrityError:
        if buckling:
            raise
        return 1.0


def _nodes(chain: NodeChain | None) -> dict[str, Any] | None:
    if chain is None:
        return None
    return chain.to_limits()


def limits_table(
    build: Any,
    model: Any,
    config: IntegrityConfig,
    materials: Mapping[str, MaterialView],
    native: Mapping[str, Any],
) -> dict[str, Any]:
    """The limits table: one record per lumped member the build lists as a structural member (index in
    `PhysicsOutputs.members` = its place in `model.lumped_members`), one per rated fastening. `native` is the prepared
    native model document the run loads; its member list must match the model's. Node indices and the exact published
    body count (`bodies`, checked by the party) come from that document, never the build's layout (INT-006)."""
    flags = enabled(config)
    design = build.design
    structural = design.structural
    minimum = 1.0
    deflection_limit: float | None = None
    if structural is not None:
        if structural.minimum_safety_factor is not None:
            minimum = float(structural.minimum_safety_factor)
        deflection_limit = float(structural.accuracy_target)
    temperature = float(design.environment.ambient_temperature)
    structural_members = {str(member.name): member for member in build.members}
    records = {str(record.member): record for record in getattr(build, "lumped", ())}
    axes = member_axes(build)
    springs = first_springs(native)
    names = published_names(model, native)
    chains = node_chains(model, names)
    if len(springs) != len(model.lumped_members):
        raise IntegrityError(
            f"the native model lists {len(springs)} members, the physics model {len(model.lumped_members)}"
        )
    members: list[dict[str, Any]] = []
    for index, lumped in enumerate(model.lumped_members):
        name = str(lumped.name)
        if lumped.structural_member is not None:
            name = str(lumped.structural_member)
        member = structural_members.get(name)
        if member is None:
            continue
        section = section_properties(member.section)
        if str(member.material) not in materials:
            raise IntegrityError(f"member {name!r}: material {member.material!r} has no integrity view")
        view = materials[str(member.material)]
        axis = axes.get(name, (1.0, 0.0, 0.0))
        yield_strength = Spread.of(view.strength_spread("yield", temperature, axis)).min
        segments = int(lumped.segments)
        segment_length = float(lumped.section.length) / segments
        stiffness = float(lumped.section.elastic_modulus) * float(lumped.section.second_moment_of_area)
        stiffness = stiffness / segment_length
        record = records.get(name)
        if record is not None:
            segment_length = float(record.segment_length)
            stiffness = float(record.node_stiffness)
        members.append(
            {
                "name": name,
                "index": index,
                "area": section.area,
                "i_y": section.i_y,
                "i_z": section.i_z,
                "c_y": section.c_y,
                "c_z": section.c_z,
                "length": section.length,
                "elastic_modulus": section.elastic_modulus,
                "effective_length_factor": _effective_length(member, section, config, flags["buckling"]),
                "allowable_stress": yield_strength / minimum,
                "yield_strength": yield_strength,
                "deflection_limit": deflection_limit,
                "supports": _supports(member),
                "first_spring": springs[index],
                "segments": segments,
                "segment_length": segment_length,
                "node_stiffness": stiffness,
                "nodes": _nodes(chains.get(name)),
            }
        )
    fastenings: list[dict[str, Any]] = []
    if flags["fastener"]:
        ratings = _parameters(config, "fastener").ratings
        order = [str(body.joint.name) for body in model.bodies if body.joint is not None]
        for fastening in model.fastenings:
            rating = ratings.get(str(fastening.name))
            if rating is None and fastening.kind != "fastener":
                continue
            if rating is None or float(rating) <= 0.0:
                raise IntegrityError(f"fastener {fastening.name!r}: no load rating")
            if str(fastening.joint) not in order:
                raise IntegrityError(f"fastening {fastening.name!r}: joint {fastening.joint!r} is not in the model")
            fastenings.append(
                {
                    "name": str(fastening.name),
                    "joint": order.index(str(fastening.joint)),
                    "load_limit": float(rating) / minimum,
                }
            )
    return {
        "format": FORMAT,
        "minimum_safety_factor": minimum,
        "bodies": len(names),
        "checks": {
            "stress": flags["stress"],
            "buckling": flags["buckling"],
            "deflection": flags["deflection"] and deflection_limit is not None,
            "fastener": flags["fastener"],
            "yield_offset": flags["yield_offset"],
        },
        "members": members,
        "fastenings": fastenings,
    }


class Tier1Service:
    """`configuration(build)`: the design's `integrity.yaml` (None without the file, ruling 27); `(build, model,
    native) -> Tier1Setup | None`: None without the file or when it turns no tier 1 quantity on. `config` replaces the
    file; `design_dir` maps a design to its folder (default `<designs_root>/<design name>`)."""

    def __init__(
        self,
        designs_root: Path = DESIGNS_ROOT,
        config: IntegrityConfig | None = None,
        design_dir: Callable[[Any], Path] | None = None,
    ) -> None:
        self.designs_root = designs_root
        self.config = config
        self.design_dir = design_dir

    def _folder(self, design: Any) -> Path:
        if self.design_dir is not None:
            return self.design_dir(design)
        return self.designs_root / str(design.name)

    def configuration(self, build: Any) -> IntegrityConfig | None:
        if self.config is not None:
            return self.config
        path = self._folder(build.design) / FILE_NAME
        if not path.is_file():
            return None
        return load_config(path)

    def __call__(self, build: Any, model: Any, native: Mapping[str, Any]) -> Tier1Setup | None:
        design = build.design
        folder = self._folder(design)
        config = self.configuration(build)
        if config is None or not tier1_enabled(config):
            return None
        overrides: Path | None = None
        if design.materials is not None and design.materials.overrides is not None:
            overrides = folder / design.materials.overrides
        materials = MaterialSet.from_files(overrides)
        views: dict[str, MaterialView] = {}
        for member in build.members:
            views[str(member.material)] = materials.integrity_view(str(member.material))
        table = limits_table(build, model, config, views, native)
        approximations: tuple[str, ...] = ()
        if table["checks"]["yield_offset"]:
            approximations = (YIELD_RULE,)
        return Tier1Setup(
            json.dumps(table, sort_keys=True),
            bool(table["checks"]["yield_offset"]),
            approximations,
        )
