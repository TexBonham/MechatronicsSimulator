"""What a build returns (BLD-001, BLD-006, BLD-007, BLD-008) and how a failed build reports (BLD-025)."""

from __future__ import annotations

import json
from typing import Literal

from pydantic import BaseModel, ConfigDict

from mechsim.design.schema import Design
from mechsim.physics.neutral import PhysicsModel, StructuralMember


class BuildError(ValueError):
    """The build failed; `failures` lists every failure found, each naming the element (BLD-025)."""

    def __init__(self, failures: list[str]) -> None:
        super().__init__("build failed:\n" + "\n".join(failures))
        self.failures = list(failures)


class _Out(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class MountBinding(_Out):
    """A cross-domain binding: a mount of the instance's type bound to a body or a joint (BLD-028)."""

    mount: str
    kind: str
    body: str | None
    joint: str | None


class Sensing(_Out):
    """A sensor instance's placement (DES-043, BLD-035): the sensed joint, case payload or body, and for a limit switch
    the switch position along the joint (SI) and its actuation side; for an encoder, whether it counts against the
    joint's positive sense. Exactly one of joint, payload and body is set."""

    joint: str | None
    payload: str | None = None
    body: str | None = None
    position: float | None
    actuate_above: bool | None
    reversed: bool


class InstanceWiring(_Out):
    """One component instance as the signal layer sees it."""

    name: str
    type: str
    category: str
    part: str
    parameters: dict[str, float | bool]
    emits: tuple[str, ...]
    consumes: tuple[str, ...]
    mounts: tuple[MountBinding, ...]
    rails: tuple[str, ...]
    senses: Sensing | None = None
    acts_on: str | None = None
    torque_speed: tuple[tuple[float, float], ...] = ()


class PinAssignment(_Out):
    """One bound signal: MCU pin, peripheral mode (runtime vocabulary) and its channel index within that mode."""

    signal: str
    instance: str
    port: str
    direction: Literal["emits", "consumes"]
    pin: str
    mode: str
    channel: int


class RailWiring(_Out):
    """A power rail: its supply, the gates the wiring passes through in order, and its consumers.

    `derived` is None while the supply or a gate is an open value; otherwise it equals `consumers` as a set.
    """

    name: str
    supply: str | None
    gates: tuple[str, ...]
    consumers: tuple[str, ...]
    derived: tuple[str, ...] | None


class Gate(_Out):
    """One link of a rail's gating chain (E-stop relay, watchdog): position from the supply and its heartbeat."""

    rail: str
    instance: str
    type: str
    position: int
    heartbeat: str | None


class OperatorLink(_Out):
    """The operator link instance and its UART pins, controller side."""

    instance: str
    baud: float
    tx_signal: str
    tx_pin: str
    rx_signal: str
    rx_pin: str


class WiringTable(_Out):
    instances: tuple[InstanceWiring, ...]
    bindings: tuple[PinAssignment, ...]
    rails: tuple[RailWiring, ...]
    controller_rail: str | None
    gating: tuple[Gate, ...]
    operator_links: tuple[OperatorLink, ...]


class ChipConfig(_Out):
    """The chip and pin configuration (BLD-007) and the firmware's `channels.h`."""

    name: str
    placeholder: bool
    bindings: tuple[PinAssignment, ...]
    channels_header: str


class LumpedRecord(_Out):
    """A lumped member as the engine will lump it, with the calibration the design records for it (BLD-019)."""

    member: str
    body: str
    segments: int
    segment_length: float
    node_stiffness: float
    spans: tuple[float, ...]
    masses: tuple[float, ...]
    calibration_source: str
    calibration_load: float
    calibration_deflection: float
    calibration_frequency: float


class BuildResult(_Out):
    """`physics` is None only for a draft design whose physics still holds open values, listed in `pending`.

    `module_instances` lists the module instances in KER-009 tick order (`build.module_order`)."""

    design: Design
    module_instances: tuple[str, ...]
    physics: PhysicsModel | None
    wiring: WiringTable
    chip: ChipConfig | None
    members: tuple[StructuralMember, ...]
    lumped: tuple[LumpedRecord, ...]
    pending: tuple[str, ...]
    notes: tuple[str, ...]

    def to_json(self) -> str:
        """Canonical JSON: the same build gives the same bytes (BLD-026)."""
        return json.dumps(self.model_dump(mode="json"), sort_keys=True, separators=(",", ":"))
