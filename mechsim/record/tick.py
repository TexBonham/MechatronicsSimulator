"""Python mirror of `core/kernel/include/mechsim/kernel/tick_record.hpp` and the physics outputs it carries.

Field for field with the C++ structs; frozen with tuples so a streaming consumer cannot change a row (REC-020).
Times are `SimTime` counts of base ticks (KER-001); every value is SI (REC-018).
"""

from __future__ import annotations

from dataclasses import dataclass

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

ZERO3: Vec3 = (0.0, 0.0, 0.0)
IDENTITY: Quat = (1.0, 0.0, 0.0, 0.0)
# `kWorldBody` (the C++ size_t maximum) as a payload carrier: released.
WORLD_BODY = -1


class RecordError(Exception):
    """A recording, archive or export cannot be produced as asked (invariant 7)."""


@dataclass(frozen=True, slots=True)
class Wrench:
    moment: Vec3 = ZERO3
    force: Vec3 = ZERO3


@dataclass(frozen=True, slots=True)
class BodyState:
    position: Vec3 = ZERO3
    orientation: Quat = IDENTITY
    angular_velocity: Vec3 = ZERO3
    linear_velocity: Vec3 = ZERO3


@dataclass(frozen=True, slots=True)
class JointForce:
    generalized_force: tuple[float, ...] = ()
    reaction: Wrench = Wrench()


@dataclass(frozen=True, slots=True)
class ConstraintForce:
    force: tuple[float, ...] = ()
    translational_error: float = 0.0
    rotational_error: float = 0.0


@dataclass(frozen=True, slots=True)
class ContactForce:
    body_a: int = 0
    body_b: int = 0
    point: Vec3 = ZERO3
    force: Vec3 = ZERO3


@dataclass(frozen=True, slots=True)
class MemberEndLoads:
    start: Wrench = Wrench()
    end: Wrench = Wrench()


@dataclass(frozen=True, slots=True)
class RopeTension:
    tension: float = 0.0
    slack: bool = False


@dataclass(frozen=True, slots=True)
class TransmissionState:
    """`power_flow` is the C++ `PowerFlow` value: 0 static, 1 forward (shaft drives), 2 backward (back-driven)."""

    slip: float = 0.0
    slipping: bool = False
    shaft_load_torque: float = 0.0
    force: float = 0.0
    stretch: float = 0.0
    power_flow: int = 0


@dataclass(frozen=True, slots=True)
class JointDriveTorque:
    """Torque or force a joint drive applied in the step, on its joint along the + axis: N m or N (PHY-034)."""

    joint: int = 0
    torque: float = 0.0


@dataclass(frozen=True, slots=True)
class PayloadState:
    """A payload body and its carrier after the step; `carrier` is `WORLD_BODY` once released, `force` the world-frame
    force the attachment applies, N (PHY-006, ENG-035)."""

    body: int = 0
    carrier: int = WORLD_BODY
    attached: bool = False
    force: Vec3 = ZERO3


@dataclass(frozen=True, slots=True)
class PhysicsOutputs:
    steps: int = 0
    bodies: tuple[BodyState, ...] = ()
    joints: tuple[JointForce, ...] = ()
    constraints: tuple[ConstraintForce, ...] = ()
    contacts: tuple[ContactForce, ...] = ()
    members: tuple[MemberEndLoads, ...] = ()
    ropes: tuple[RopeTension, ...] = ()
    transmissions: tuple[TransmissionState, ...] = ()
    joint_positions: tuple[float, ...] = ()
    joint_drive_torques: tuple[JointDriveTorque, ...] = ()
    payloads: tuple[PayloadState, ...] = ()


@dataclass(frozen=True, slots=True)
class SeriesSample:
    """One named scalar sample: dotted signal or rail name (SIG-023) and its unit (REC-018)."""

    name: str
    unit: str
    value: float


@dataclass(frozen=True, slots=True)
class RecordedEvent:
    """Injected fault, safety event, operator command or status, controller fault log entry (REC-007 to REC-009)."""

    time: int
    source: str
    kind: str
    detail: str


COUPLING_KINDS = ("none", "real", "integer", "flag", "payload")


@dataclass(frozen=True, slots=True)
class CouplingSample:
    """One coupling variable at the row's sync point (KER-022). A scalar (`real`, `integer`, `flag`) carries its value;
    an unset variable (`none`) and a typed payload carry NaN, the payload its content digest (`payload_digest`)."""

    name: str
    kind: str
    value: float = float("nan")
    digest: str = ""


@dataclass(frozen=True, slots=True)
class TickRecord:
    """One row per physics tick, simulated time first (REC-011)."""

    time: int
    physics: PhysicsOutputs = PhysicsOutputs()
    pin_signals: tuple[SeriesSample, ...] = ()
    rails: tuple[SeriesSample, ...] = ()
    actuator_efforts: tuple[SeriesSample, ...] = ()
    controller_state: str = ""
    events: tuple[RecordedEvent, ...] = ()
    coupling: tuple[CouplingSample, ...] = ()
