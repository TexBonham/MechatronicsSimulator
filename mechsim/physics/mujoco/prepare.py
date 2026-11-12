"""PreparedModel assembly for the MuJoCo bridge (ADR 0014): neutral JSON, MJCF and every mapping-report line.

The translation report rides along so no approximation is silent (ADR 0003). A start position for a hinge or slide
is written into the MJCF exactly: the child body is placed at the start coordinate and the joint's `ref` set to it,
so MuJoCo's reset state `qpos0` is the start while the coordinate keeps its meaning (zero at the neutral placement)
and the limits stay where the model puts them. The joint anchor is the body origin, as the translation writes it.
A payload (a free body under world) follows its carrier to the start configuration: its MJCF pose is moved by the
carrier's rigid displacement from the reference configuration, so the weld's relative pose is the modelled one.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from mechsim.physics.mujoco.translate import translate_report
from mechsim.physics.neutral import PhysicsModel

STARTABLE = ("revolute", "prismatic")


class StartStateError(ValueError):
    """A start state the bridge cannot place exactly."""


@dataclass(frozen=True)
class PreparedArtifacts:
    """What `PreparedModel(neutral_json, engine_artifact, version)` takes, plus the translation report."""

    neutral_json: str
    mjcf: str
    version: str
    report: tuple[str, ...]


def _numbers(text: str | None, default: Sequence[float]) -> list[float]:
    if text is None:
        return list(default)
    return [float(item) for item in text.split()]


def _text(values: Sequence[float]) -> str:
    return " ".join(repr(float(value)) for value in values)


def _multiply(a: Sequence[float], b: Sequence[float]) -> list[float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def _rotate(quaternion: Sequence[float], vector: Sequence[float]) -> list[float]:
    conjugate = [quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]]
    rotated = _multiply(_multiply(quaternion, [0.0, *vector]), conjugate)
    return rotated[1:]


def _unit(axis: Sequence[float]) -> list[float]:
    norm = math.sqrt(sum(value * value for value in axis))
    return [value / norm for value in axis]


def _place(root: ET.Element, joint: str, kind: str, start: float) -> None:
    for body in root.iter("body"):
        for element in body.findall("joint"):
            if element.get("name") != joint:
                continue
            axis = _unit(_numbers(element.get("axis"), (0.0, 0.0, 1.0)))
            position = _numbers(body.get("pos"), (0.0, 0.0, 0.0))
            orientation = _numbers(body.get("quat"), (1.0, 0.0, 0.0, 0.0))
            if kind == "revolute":
                half = start / 2.0
                turn = [math.cos(half), *(math.sin(half) * value for value in axis)]
                body.set("quat", _text(_multiply(orientation, turn)))
            else:
                shift = _rotate(orientation, [start * value for value in axis])
                body.set("pos", _text([p + s for p, s in zip(position, shift, strict=True)]))
            element.set("ref", repr(float(start)))
            return
    raise StartStateError(f"joint '{joint}': not in the MJCF (a reflected shaft starts where its driven joint does)")


def _world_pose(root: ET.Element, name: str) -> tuple[list[float], list[float]] | None:
    def walk(
        element: ET.Element, position: list[float], orientation: list[float]
    ) -> tuple[list[float], list[float]] | None:
        for body in element.findall("body"):
            offset = _rotate(orientation, _numbers(body.get("pos"), (0.0, 0.0, 0.0)))
            here = [p + o for p, o in zip(position, offset, strict=True)]
            turn = _multiply(orientation, _numbers(body.get("quat"), (1.0, 0.0, 0.0, 0.0)))
            if body.get("name") == name:
                return here, turn
            found = walk(body, here, turn)
            if found is not None:
                return found
        return None

    world = root.find("worldbody")
    if world is None:
        return None
    return walk(world, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])


def _follow(root: ET.Element, before: ET.Element, payloads: Mapping[str, str]) -> None:
    world = root.find("worldbody")
    if world is None:
        return
    for payload, carrier in sorted(payloads.items()):
        old = _world_pose(before, carrier)
        new = _world_pose(root, carrier)
        if old is None or new is None:
            raise StartStateError(f"payload '{payload}': carrier '{carrier}' is not in the MJCF")
        for body in world.findall("body"):
            if body.get("name") != payload:
                continue
            turn = _multiply(new[1], [old[1][0], -old[1][1], -old[1][2], -old[1][3]])
            position = _numbers(body.get("pos"), (0.0, 0.0, 0.0))
            relative = _rotate(turn, [p - o for p, o in zip(position, old[0], strict=True)])
            body.set("pos", _text([r + n for r, n in zip(relative, new[0], strict=True)]))
            body.set("quat", _text(_multiply(turn, _numbers(body.get("quat"), (1.0, 0.0, 0.0, 0.0)))))


def prepare_model(
    model: PhysicsModel,
    starts: Mapping[str, float] | None = None,
    step: float | None = None,
    payloads: Mapping[str, str] | None = None,
) -> PreparedArtifacts:
    """The bridge's prepared model; `starts` maps single-DOF joint names to their start coordinate (m or rad); `step`
    (s) is the run's physics step when it is not the model's; `payloads` maps payload bodies to their carriers."""
    translation = translate_report(model, step, payloads)
    chosen = dict(starts or {})
    kinds = {body.joint.name: body.joint.type for body in model.bodies if body.joint is not None}
    mjcf = translation.mjcf
    if len(chosen) > 0:
        root = ET.fromstring(mjcf)
        for name in sorted(chosen):
            kind = kinds.get(name)
            if kind is None:
                raise StartStateError(f"joint '{name}': no such joint in the physics model")
            if kind not in STARTABLE:
                raise StartStateError(f"joint '{name}': a {kind} joint takes no single start coordinate")
            _place(root, name, kind, chosen[name])
        if payloads is not None and len(payloads) > 0:
            _follow(root, ET.fromstring(mjcf), payloads)
        mjcf = ET.tostring(root, encoding="unicode")
    return PreparedArtifacts(model.model_dump_json(), mjcf, model.design.version, translation.report)
