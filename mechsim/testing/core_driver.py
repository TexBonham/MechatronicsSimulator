"""The real simulation driver over the core binding (TST-012, TST-021, TST-036), the record view scoring reads, and
the archive writer and reader the runner and the CLI use (TST-033, L8).

`CoreDriver.run` prepares the neutral model for the physics adapter the run configuration names (`physics.adapter`
of the case or `--set`, TST-040; `mujoco` by default until X2; `native` is the own engine, PHY-020), translation
report kept (ADR 0003); `run` builds
a co-simulation kernel with the physics party and records every row through an in-memory sink into
`mechsim.record.Recorder`. Joint coordinates and poses are derived from the recorded body poses with the model's
kinematics (TST-038): a joint coordinate is zero at the neutral placement; a pose is the body frame's displacement
and rotation from its reference-configuration pose (every joint coordinate zero), world axes.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Callable, Mapping, Sequence
from dataclasses import replace
from fractions import Fraction
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np

from mechsim import _core as core
from mechsim.builder import BuildError, build
from mechsim.builder.payload import with_payload
from mechsim.builder.result import BuildResult as CoreBuild
from mechsim.design.schema import Design, DesignError, blob_hash, design_materials
from mechsim.integrity.config import IntegrityConfig
from mechsim.integrity.plugin import IntegrityError
from mechsim.integrity.service import IntegrityService
from mechsim.integrity.tier1 import MODEL_REQUESTS, NOT_EVALUATED, PARTY, Tier1Service, Tier1Setup, tier1_enabled
from mechsim.library.registry import default_registry
from mechsim.materials.schema import MaterialError
from mechsim.physics.mujoco.prepare import StartStateError, prepare_model
from mechsim.physics.mujoco.translate import UnmappableElementError
from mechsim.physics.native.prepare import prepare_model as prepare_native_model
from mechsim.physics.native.translate import StartStateError as NativeStartStateError
from mechsim.physics.native.translate import UnmappableElementError as NativeUnmappableElementError
from mechsim.physics.native.translate import (
    carried_member_lines,
    carried_validity_lines,
    driven_mode_lines,
    member_node_names,
)
from mechsim.physics.neutral import PhysicsModel
from mechsim.physics.neutral import Rope as NeutralRope
from mechsim.record.archive import RunEnd, RunFault, RunIdentity, repository_commit, tool_version
from mechsim.record.archive import write_archive as write_record_archive
from mechsim.record.layout import DRIVE_DRIVEN, DRIVE_TORQUE, JOINT_POSITIONS, MEMBER_SERIES, ROPE_SERIES, Columns
from mechsim.record.reader import DriverError as DriverError
from mechsim.record.reader import JointKinematics as JointKinematics
from mechsim.record.reader import Kinematics as Kinematics
from mechsim.record.reader import RecordArchiveReader as RecordArchiveReader
from mechsim.record.reader import RecordingView as RecordingView
from mechsim.record.stream import Recorder
from mechsim.record.tick import (
    WORLD_BODY,
    BodyState,
    ConstraintForce,
    ContactForce,
    CouplingSample,
    JointDriveTorque,
    JointForce,
    MemberEndLoads,
    PayloadState,
    PhysicsOutputs,
    RopeTension,
    SeriesSample,
    TickRecord,
    TransmissionState,
    Wrench,
)
from mechsim.record.tick import RecordedEvent as RowEvent
from mechsim.testing.core_wiring import (
    OPERATOR_LINK_TYPE,
    WIRED_FAULTS,
    RunnerParty,
    RunnerState,
    WiringError,
    controller_party,
    event_axes,
    operator_message,
    power_on_state,
    signal_layer,
    work_directory,
)
from mechsim.testing.driver import (
    INTEGRITY_KIND,
    MATERIAL_OVERRIDES_KEY,
    BuildResult,
    EndReason,
    EndSpec,
    IntegrityCrossing,
    RunConfig,
    RunMetadata,
    RunReport,
    ScheduledCommand,
    ScheduledFault,
    members_json,
)
from mechsim.testing.runner import Services
from mechsim.testing.schema import (
    CASE_PAYLOAD_BODY,
    DEFAULT_PHYSICS_ADAPTER,
    DESIGNS_ROOT,
    NATIVE_ADAPTER,
    PHYSICS_ADAPTERS,
    ROOT,
    Case,
    InitialState,
    Quantity,
    case_payload_body,
)

OPERATOR_EVENT_KINDS = ("operator_status", "operator_command")
SIGNAL_LAYER_CATEGORIES = ("sensor", "front_end", "actuator", "driver", "power", "safety", "communication")
CASES_ROOT = ROOT / "test_cases"
Vec = tuple[float, float, float]
Quat = tuple[float, float, float, float]


# ===== Rows =============================================================


def _vec(values: Sequence[float]) -> Vec:
    return (float(values[0]), float(values[1]), float(values[2]))


def _canonical(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, (list, tuple)):
        return [_canonical(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _canonical(value[key]) for key in sorted(value)}
    if isinstance(value, np.ndarray):
        return _canonical(value.tolist())
    if hasattr(type(value), "__members__"):
        return str(value)
    fields = [name for name in dir(value) if not name.startswith("_") and not callable(getattr(value, name))]
    return {name: _canonical(getattr(value, name)) for name in sorted(fields)}


def payload_digest(value: Any) -> str:
    """SHA-256 of a coupling payload's canonical content: every public field, floats by repr (REC-022 coverage)."""
    text = json.dumps(_canonical(value), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def coupling_sample(name: str, value: Any) -> CouplingSample:
    if value is None:
        return CouplingSample(name, "none")
    if isinstance(value, bool):
        return CouplingSample(name, "flag", float(value))
    if isinstance(value, int):
        return CouplingSample(name, "integer", float(value))
    if isinstance(value, float):
        return CouplingSample(name, "real", value)
    return CouplingSample(name, "payload", float("nan"), payload_digest(value))


def _body_index(value: Any) -> int:
    """A contact body index; the core's world index (size_t max, kWorldBody) is the record's `WORLD_BODY`, -1."""
    index = int(value)
    if index >= 2**63:
        return WORLD_BODY
    return index


def tick_record(row: Any, events: Mapping[int, Sequence[RowEvent]] | None = None) -> TickRecord:
    """The record module's frozen mirror of one bound `TickRecord` (REC-020: a copy, never kernel memory); `events`
    maps a tick to decoded firmware events, which follow the row's own events."""
    data = row.to_dict()
    physics = data["physics"]
    bodies = tuple(
        BodyState(
            _vec(b["position"]),
            (
                float(b["orientation"][0]),
                float(b["orientation"][1]),
                float(b["orientation"][2]),
                float(b["orientation"][3]),
            ),
            _vec(b["angular_velocity"]),
            _vec(b["linear_velocity"]),
        )
        for b in physics["bodies"]
    )
    joints = tuple(
        JointForce(
            tuple(float(v) for v in j["generalized_force"]),
            Wrench(_vec(j["reaction"]["moment"]), _vec(j["reaction"]["force"])),
        )
        for j in physics["joints"]
    )
    constraints = tuple(
        ConstraintForce(
            tuple(float(v) for v in c["force"]), float(c["translational_error"]), float(c["rotational_error"])
        )
        for c in physics["constraints"]
    )
    contacts = tuple(
        ContactForce(_body_index(c["body_a"]), _body_index(c["body_b"]), _vec(c["point"]), _vec(c["force"]))
        for c in physics["contacts"]
    )
    members = tuple(
        MemberEndLoads(
            Wrench(_vec(m["start"]["moment"]), _vec(m["start"]["force"])),
            Wrench(_vec(m["end"]["moment"]), _vec(m["end"]["force"])),
        )
        for m in physics["members"]
    )
    ropes = tuple(RopeTension(float(r["tension"]), bool(r["slack"]), float(r["max_tension"])) for r in physics["ropes"])
    transmissions = tuple(
        TransmissionState(
            float(t["slip"]),
            bool(t["slipping"]),
            float(t["shaft_load_torque"]),
            float(t["force"]),
            float(t["stretch"]),
            int(t["power_flow"]),
        )
        for t in physics["transmissions"]
    )

    def samples(entries: list[dict[str, Any]]) -> tuple[SeriesSample, ...]:
        return tuple(SeriesSample(e["name"], e["unit"], float(e["value"])) for e in entries)

    positions = tuple(float(value) for value in physics.get("joint_positions", []))
    drives = tuple(
        JointDriveTorque(int(d["joint"]), float(d["torque"])) for d in physics.get("joint_drive_torques", [])
    )
    payloads = tuple(
        PayloadState(int(p["body"]), int(p["carrier"]), bool(p["attached"]), _vec(p["force"]))
        for p in physics.get("payloads", [])
    )
    coupling = data["coupling"]
    row_events = tuple(RowEvent(int(e["time"]), e["source"], e["kind"], e["detail"]) for e in data["events"])
    return TickRecord(
        time=int(data["time"]),
        physics=PhysicsOutputs(
            int(physics["steps"]),
            bodies,
            joints,
            constraints,
            contacts,
            members,
            ropes,
            transmissions,
            positions,
            drives,
            payloads,
        ),
        pin_signals=samples(data["pin_signals"]),
        rails=samples(data["rails"]),
        actuator_efforts=samples(data["actuator_efforts"]),
        controller_state=str(data["controller_state"]),
        events=row_events + tuple((events or {}).get(int(data["time"]), ())),
        coupling=tuple(coupling_sample(name, coupling[name]) for name in sorted(coupling)),
    )


# ===== Driver ===========================================================


def _rate(value: Fraction) -> Any:
    return core.Rate(value.numerator, value.denominator)


def _seconds(ticks: int, base_rate: Fraction) -> Any:
    value = Fraction(ticks) / base_rate
    return core.Seconds(value.numerator, value.denominator)


def _si(amount: object, joint: str) -> float:
    if isinstance(amount, Quantity):
        return amount.value
    if isinstance(amount, float | int) and not isinstance(amount, bool):
        return float(amount)
    raise DriverError(f"joint '{joint}': start value {amount!r} is not resolved to a quantity")


ROPE_SNAP_OFF = (
    "integrity tier 2 rope snap: off, no rope_snap entry in integrity.yaml (INT-031, ruling 53): the breaking "
    "tension of rope '{}' is not enforced during the run"
)
BRIDGE_NODE_POSES = (
    "member node poses: none recorded on the mujoco bridge, which builds members rigid (BLD-033, ruling 29); the body "
    "series name the neutral bodies only, the native adapter adds <member>.node<k> (PHY-003)"
)


def payload_stand_in_line(rope: NeutralRope, carrier: str) -> str:
    """The bridge's approximation line for a case payload whose design rope it does not model (BLD-032)."""
    return (
        f"payload rope '{rope.name}': not modeled on the mujoco bridge; the case payload is fixed to '{carrier}' "
        "(the design's suspension joints stand in for the rope swing); the native adapter hangs it on the rope"
    )


class CoreDriver:
    """`SimulationDriver` over the core binding. One driver per run (the runner's factory makes a fresh one)."""

    def __init__(
        self,
        subscribers: Sequence[Callable[[Any], None]] = (),
        designs_root: Path = DESIGNS_ROOT,
        tier1: Tier1Service | None = None,
    ) -> None:
        """`subscribers` receive each row as the kernel writes it (REC-019); one that raises is a recorder error,
        which ends the scoring as a run error and lands in the archive metadata (invariant 7). The physics adapter is
        the run configuration's `physics.adapter`, set by `configure`. The design's `integrity.yaml` (tier 1 party,
        INT-023; tier 2 rope snap, INT-031) is read from `<designs_root>/<design name>/`, `designs_root` the shipped
        `designs/` folder by default; `tier1` replaces that lookup (a `Tier1Service` with its own configuration)."""
        self._subscribers = tuple(subscribers)
        self._tier1: Tier1Service = Tier1Service(designs_root)
        if tier1 is not None:
            self._tier1 = tier1
        self._case: Case | None = None
        self.integrity_trace: Any = None
        self.adapter: str = DEFAULT_PHYSICS_ADAPTER
        self._build: CoreBuild | None = None
        self._model: PhysicsModel | None = None
        self._config: RunConfig | None = None
        self._starts: dict[str, float] = {}
        self._view: RecordingView | None = None
        self.approximations: tuple[str, ...] = ()
        self.subscriber_errors: tuple[str, ...] = ()
        self._commands: list[ScheduledCommand] = []
        self._faults: list[ScheduledFault] = []
        self._runner: RunnerState | None = None
        self.labels: dict[str, str] = {}
        self._payload: tuple[str, str, float, bool] | None = None
        self._physics_faults: list[ScheduledFault] = []
        self._freezes: list[ScheduledFault] = []

    def load(self, build: BuildResult, case: Case) -> None:
        if not isinstance(build, CoreBuild):
            raise DriverError("the core driver runs builds of mechsim.builder")
        if build.physics is None:
            raise DriverError("the design's physics is pending: " + ", ".join(build.pending))
        self._build = build
        self._model = build.physics
        self._case = case

    def reported_controller_state(self) -> Mapping[str, bool | str]:
        """`{"state": name, "fault": name}` at power on, before t = 0, from the design's code tables (DES-045);
        empty for a design without firmware."""
        build = self._build
        config = self._config
        if build is None or build.design.firmware is None:
            return {}
        if config is None or config.control_rate is None:
            raise DriverError("reported_controller_state before configure")
        try:
            return power_on_state(build, config.control_rate, work_directory())
        except WiringError as error:
            raise DriverError(str(error)) from error

    def _signals(self) -> bool:
        build = self._build
        if build is None:
            return False
        return build.design.firmware is not None or any(
            instance.category in SIGNAL_LAYER_CATEGORIES for instance in build.wiring.instances
        )

    def configure(self, config: RunConfig) -> None:
        firmware = self._build is not None and self._build.design.firmware is not None
        if not firmware and (config.control_rate is not None or config.controller_mode is not None):
            raise DriverError("this build has no controller; the run configuration names one")
        if firmware and config.controller_mode != "native":
            raise DriverError(f"controller mode {config.controller_mode!r}: milestone 1 runs native only")
        if config.base_rate != config.physics_rate:
            raise DriverError(
                f"base rate {config.base_rate} Hz must equal the physics rate {config.physics_rate} Hz (milestone 1 "
                "runs the controller at the physics rate)"
            )
        if config.physics_adapter not in PHYSICS_ADAPTERS:
            raise DriverError(f"physics.adapter {config.physics_adapter!r}: one of {', '.join(PHYSICS_ADAPTERS)}")
        self.adapter = config.physics_adapter
        self._config = config

    def _payload_name(self) -> str:
        """The body name the run gives the case payload: `testing.schema.case_payload_body` over the design's
        components and the model's joints (SCO-030), so the driver and scoring agree."""
        if self._build is None:
            return CASE_PAYLOAD_BODY
        joints: set[str] = set()
        if self._model is not None:
            joints = {body.joint.name for body in self._model.bodies if body.joint is not None}
        return case_payload_body((component.senses for component in self._build.design.components), joints)

    def set_initial_state(self, state: InitialState) -> None:
        """The case payload becomes a neutral body (BLD-036) on `initial_state.payload.body`; a case that attaches
        none gets no body, and a load cell reading a payload then has no target and reads zero (A4)."""
        self._payload = None
        if state.payload is not None and state.payload.attached:
            name = self._payload_name()
            self._payload = (name, str(state.payload.body), _si(state.payload.mass, "payload"), True)
        starts: dict[str, float] = {}
        for joint in state.joints:
            if _si(joint.velocity, joint.joint) != 0.0:
                raise DriverError(f"joint '{joint.joint}': a non-zero start velocity has no bridge input yet")
            starts[joint.joint] = _si(joint.position, joint.joint)
        self._starts = starts

    def schedule_command(self, command: ScheduledCommand) -> None:
        build = self._build
        if build is None or not any(i.type == OPERATOR_LINK_TYPE for i in build.wiring.instances):
            raise DriverError(f"command '{command.message}': this build has no operator link")
        self._commands.append(command)

    def schedule_fault(self, fault: ScheduledFault) -> None:
        """Signal faults go to the layer (power loss to the rail's supply), slip and payload changes to the physics
        party, a freeze to the controller party (TST-015 to TST-020)."""
        if fault.kind in ("slip", "payload_change"):
            self._physics_faults.append(fault)
            return
        if fault.kind == "controller_freeze":
            if self._build is None or self._build.design.firmware is None:
                raise DriverError("fault 'controller_freeze': this build has no controller")
            self._freezes.append(fault)
            return
        if fault.kind not in WIRED_FAULTS:
            raise DriverError(f"fault '{fault.kind}': no delivery to the {fault.owner} party")
        if not self._signals():
            raise DriverError(f"fault '{fault.kind}': this build has no signal layer")
        self._faults.append(fault)

    def run(self, end: EndSpec) -> RunReport:
        if self._model is None or self._config is None:
            raise DriverError("run before load and configure")
        if end.message is not None or end.controller_state is not None:
            raise DriverError("message and controller-state end triggers need a controller")
        config = self._config
        try:
            return self._run(end, config, self._model)
        except DriverError:
            raise
        except (StartStateError, UnmappableElementError, NativeStartStateError, NativeUnmappableElementError) as error:
            raise DriverError(str(error)) from error
        except WiringError as error:
            raise DriverError(str(error)) from error
        except (ValueError, RuntimeError, TypeError) as error:
            raise DriverError(f"{type(error).__name__}: {error}") from error

    def _run(self, end: EndSpec, config: RunConfig, model: PhysicsModel) -> RunReport:
        """Binding errors (AdapterError, KernelConfigError, SignalConfigError and the rest, all ValueError or
        RuntimeError) escape to `run`, which types them as DriverError with the cause kept (invariant 7)."""
        payloads: dict[str, str] = {}
        integrity = self._integrity_config()
        model, snap_lines = _rope_snap_gate(model, integrity)
        native = self.adapter == NATIVE_ADAPTER
        stand_in: tuple[str, ...] = ()
        if self._payload is not None:
            name, carrier, mass, _ = self._payload
            fixed = self._payload_fixed()
            rope = self._payload_rope(name)
            if rope is not None and not native:
                stand_in = (payload_stand_in_line(rope, carrier),)
                rope = None
            if rope is not None and not fixed:
                raise DriverError(f"payload '{name}': hangs on rope '{rope.name}'; a mid-run release has no rope form")
            try:
                model = with_payload(model, name, carrier, mass, fixed, rope, self._starts)
            except BuildError as failure:
                raise DriverError(str(failure)) from failure
            if not fixed:
                payloads[name] = carrier
        if config.physics_substeps > 1 and not native:
            raise DriverError(f"physics.substeps {config.physics_substeps}: only the native adapter substeps (TST-042)")
        step = float(1 / config.physics_rate)
        tier1: Tier1Setup | None = None
        if native:
            prepared_native = prepare_native_model(model, self._starts, step, payloads, config.physics_substeps)
            self.approximations = (f"physics adapter: {NATIVE_ADAPTER}", *prepared_native.report)
            prepared = core.PreparedModel(prepared_native.model_json, "", prepared_native.version)
            tier1 = self._tier1_setup(model, json.loads(prepared_native.model_json))
        else:
            artifacts = prepare_model(model, self._starts, step, payloads)
            self.approximations = (*artifacts.report, *stand_in)
            prepared = core.PreparedModel(artifacts.neutral_json, artifacts.mjcf, artifacts.version)
            if integrity is not None and tier1_enabled(integrity):
                self.approximations = (*self.approximations, NOT_EVALUATED)
        self.approximations = (*self.approximations, *snap_lines)
        adapter = core.make_physics_adapter(self.adapter)
        rate = _rate(config.physics_rate)
        adapter.load(prepared, rate)
        run_config = core.RunConfig()
        run_config.physics_rate = rate
        run_config.seed = config.seed
        if config.control_rate is not None:
            run_config.control_rate = _rate(config.control_rate)
        kernel = core.make_kernel(run_config)
        party = core.PhysicsPartyConfig()
        party.rate = rate
        party.inputs = []
        signals = self._signals()
        if signals:
            party.inputs = ["signals.physics_inputs"]
        if tier1 is not None and tier1.yield_offset:
            party.model_requests = MODEL_REQUESTS
        decay_record = None
        carried_record = None
        if native:
            decay_record = adapter.scheme_decay_record()
            carried_record = adapter.carried_record()
        physics = core.PhysicsParty(adapter, party)
        self._schedule_physics(physics, model)
        kernel.register_party(physics)
        halt = False
        self.integrity_trace = None
        if tier1 is not None:
            halt = self._halt()
            self.approximations = (*self.approximations, *tier1.approximations)
            settings = core.IntegrityPartyConfig()
            settings.name = PARTY
            settings.rate = rate
            settings.halt_on_crossing = halt
            self.integrity_trace = core.IntegrityTrace()
            kernel.register_party(core.IntegrityParty(tier1.limits, settings, self.integrity_trace))
            if halt:
                kernel.set_end_triggers(core.EndTriggers(None, "", PARTY))
        self.labels = {}
        if signals:
            self._register_signals(kernel, config, model, sorted(payloads))
        stop = end.max_tick
        if end.at_tick is not None:
            stop = min(stop, end.at_tick)
        sink = core.InMemoryRecorderSink()
        for subscriber in self._subscribers:
            sink.subscribe(subscriber)
        report = kernel.run(core.EndCondition(_seconds(stop, config.base_rate)), sink)
        bodies = tuple(body.name for body in model.bodies)
        if native:
            bodies = (*bodies, *member_node_names(model))
        if decay_record is not None:
            decays = [(int(joint), float(decay)) for joint, decay in decay_record.values()]
            self.approximations = (*self.approximations, *driven_mode_lines(model, decays))
        if carried_record is not None:
            carried_values = carried_record.values()
            self.approximations = (
                *self.approximations,
                *carried_member_lines(model, carried_values),
                *carried_validity_lines(model, carried_values),
            )
        if not native and self._build is not None and len(self._build.members) > 0:
            self.approximations = (*self.approximations, BRIDGE_NODE_POSES)
        recorder = Recorder(config.base_rate, model_columns(model, sorted(payloads)), bodies)
        frames = member_frames(model)
        if native:
            frames = ()
        decoded: list[RowEvent] = []
        if self._runner is not None:
            decoded = [RowEvent(tick, "controller", kind, detail) for tick, kind, detail in self._runner.events]
            decoded.sort(key=lambda event: event.time)
        taken = 0
        for row in sink.drain():
            # Rows are stamped at the end of their step (KER-009), so an event of the first controller step (tick 0)
            # has no row of its own: it goes with the first row at or after its tick.
            row_tick = int(row.time.count())
            due: list[RowEvent] = []
            while taken < len(decoded) and decoded[taken].time <= row_tick:
                due.append(decoded[taken])
                taken += 1
            record = _canonical_crossings(tick_record(row, {row_tick: due}))
            if not native:
                record = member_frame_loads(record, frames)
            recorder.write_row(record)
        self.subscriber_errors = tuple(str(error) for error in sink.subscriber_errors())
        self._view = RecordingView(
            recorder.recording(),
            Kinematics.of(model),
            self.approximations,
            self.labels,
            self._actuators(),
            self._states(),
        )
        reason: EndReason = "max_duration"
        if end.at_tick is not None and end.at_tick <= end.max_tick:
            reason = "end_time"
        notes: tuple[str, ...] = ()
        if report.end_reason == core.EndReason.party_state:
            reason = "integrity_halt"
            notes = (self._halt_line(int(report.ticks), config.base_rate),)
        error: str | None = None
        if report.outcome == core.RunOutcome.run_error:
            reason = "run_error"
            fault = report.fault
            error = "run error"
            if fault is not None:
                error = f"{fault.party}: {fault.cause}"
        sink_errors = list(dict.fromkeys([str(item) for item in report.sink_errors] + list(self.subscriber_errors)))
        if len(sink_errors) > 0 and error is None:
            reason = "run_error"
            error = "recorder: " + "; ".join(sink_errors)
        hook_errors: tuple[str, ...] = ()
        if self._runner is not None:
            hook_errors = tuple(self._runner.hook_errors)
        if len(hook_errors) > 0 and error is None:
            reason = "run_error"
            error = "; ".join(hook_errors)
        return RunReport(
            reason,
            int(report.ticks),
            config.base_rate,
            error,
            tuple(sink_errors),
            hook_errors,
            self.approximations,
            notes,
        )

    def _integrity_config(self) -> IntegrityConfig | None:
        if self._build is None:
            return None
        try:
            return self._tier1.configuration(self._build)
        except IntegrityError as error:
            raise DriverError(f"integrity: {error}") from error

    def _tier1_setup(self, model: PhysicsModel, native: Mapping[str, Any]) -> Tier1Setup | None:
        if self._build is None:
            return None
        try:
            return self._tier1(self._build, model, native)
        except IntegrityError as error:
            raise DriverError(f"integrity tier 1: {error}") from error

    def _halt(self) -> bool:
        """The case's `integrity.halt_on_crossing`, false when the case has no integrity section (rulings 4, 50)."""
        case = self._case
        if case is not None and case.integrity is not None:
            return case.integrity.halt_on_crossing
        return False

    def _halt_line(self, ticks: int, base_rate: Fraction) -> str:
        """The report line of a halted run (INT-026, ruling 51): what crossed, by how much, and when."""
        seconds = float(Fraction(ticks) / base_rate)
        what = "its first tier 1 crossing"
        if self.integrity_trace is not None and len(self.integrity_trace.crossings) > 0:
            first = self.integrity_trace.crossings[0]
            what = (
                f"the {first.quantity} of '{first.element}' at {first.value:.12g} against the limit "
                f"{first.limit:.12g} at {float(Fraction(int(first.time.count())) / base_rate):.12g} s"
            )
        return f"integrity halt: the run ended at {seconds:.12g} s on {what} (INT-026)"

    def _states(self) -> dict[str, str]:
        if self._build is None or self._build.design.firmware is None:
            return {}
        return dict(self._build.design.firmware.state_codes)

    def _actuators(self) -> dict[str, str]:
        """Each actuator instance's shaft joint: `actuator:<name>.position` reads it (TST-038)."""
        actuators: dict[str, str] = {}
        if self._build is None:
            return actuators
        for instance in self._build.wiring.instances:
            if instance.category == "actuator":
                for mount in instance.mounts:
                    if mount.kind == "shaft" and mount.joint is not None:
                        actuators[instance.name] = mount.joint
        return actuators

    def _case_payload(self) -> str | None:
        if self._payload is None:
            return None
        return self._payload[0]

    def _payload_rope(self, name: str) -> NeutralRope | None:
        """The design rope ending at the case payload `name` (BLD-032), if any."""
        if self._build is None:
            return None
        for rope in self._build.payload_ropes:
            if rope.to_body == name:
                return rope
        return None

    def _payload_fixed(self) -> bool:
        """Ruling 24: a payload attached at t = 0 that no fault releases is a fixed child of its carrier; a released
        or never-attached one is welded (BLD-036)."""
        if self._payload is None or not self._payload[3]:
            return False
        for fault in self._physics_faults:
            if fault.kind == "payload_change" and float(dict(fault.parameters)["mass"]) == 0.0:
                return False
        return True

    def _schedule_physics(self, physics: Any, model: PhysicsModel) -> None:
        """Slip on the transmission named by its instance, payload changes on the payload carried by the target body
        (by the payload's body index; 0 kg releases it to the world, carrier -1), each at its tick (TST-015,
        TST-020)."""
        transmissions = [item.name for item in model.transmissions]
        body = 0
        if self._payload is not None:
            body = [item.name for item in model.bodies].index(self._payload[0])
        for fault in self._physics_faults:
            parameters = dict(fault.parameters)
            if fault.kind == "slip":
                if fault.target not in transmissions or fault.end_tick is None:
                    raise DriverError(f"fault 'slip': no transmission '{fault.target}' in the model")
                physics.schedule_slip(
                    transmissions.index(str(fault.target)),
                    core.SimTime(fault.start_tick),
                    core.SimTime(fault.end_tick),
                    float(parameters["fraction"]),
                )
                continue
            if self._payload is None or not self._payload[3] or self._payload[1] != fault.target:
                raise DriverError(f"fault 'payload_change': no payload attached to '{fault.target}' at the start")
            mass = float(parameters["mass"])
            if mass == 0.0:
                physics.schedule_payload_change(body, core.SimTime(fault.start_tick), carrier=-1)
            else:
                physics.schedule_payload_change(body, core.SimTime(fault.start_tick), mass=mass)
        if self._payload is not None and not self._payload[3]:
            physics.schedule_payload_change(body, core.SimTime(0), carrier=-1)

    def _register_signals(self, kernel: Any, config: RunConfig, model: PhysicsModel, payloads: Sequence[str]) -> None:
        build = self._build
        if build is None:
            raise DriverError("run before load")
        seeds = dict(config.module_seeds)
        try:
            layer, externals, labels, _ = signal_layer(
                build,
                model,
                config.physics_rate,
                seeds.get("signals", config.seed),
                self._faults,
                config.base_rate,
                config.control_rate,
                payloads,
                self._case_payload(),
            )
            state = RunnerState(externals=externals)
            firmware = build.design.firmware
            if firmware is not None and config.control_rate is not None:
                state.event_names = {int(code): entry.name for code, entry in firmware.event_codes.items()}
                state.event_arguments = {int(code): entry.argument for code, entry in firmware.event_codes.items()}
                state.fault_names = {int(code): name for code, name in firmware.fault_codes.items()}
                state.state_names = {int(code): name for code, name in firmware.state_codes.items()}
                state.track_faults = len(firmware.fault_codes) > 0
                state.axes = event_axes(build)
                state.control_period = int(config.base_rate / config.control_rate)
            for command in sorted(self._commands, key=lambda item: item.tick):
                state.commands.append((command.tick, operator_message(build, command)))
            for fault in self._faults:
                if fault.kind == "estop_press":
                    if fault.end_tick is None:
                        raise WiringError("fault 'estop_press': needs a window")
                    state.presses.append((str(fault.target), fault.start_tick, fault.end_tick))
            self.labels = labels
            self._runner = state
            kernel.register_party(RunnerParty(state, _rate(config.physics_rate)))
            kernel.register_party(layer)
            if build.design.firmware is not None:
                if config.control_rate is None:
                    raise DriverError("a firmware design needs a control rate")
                controller = controller_party(build, config.control_rate, work_directory())
                for fault in self._freezes:
                    if fault.end_tick is None:
                        raise WiringError("fault 'controller_freeze': needs a window")
                    controller.schedule_freeze(core.SimTime(fault.start_tick), core.SimTime(fault.end_tick))
                kernel.register_party(controller)
        except WiringError as error:
            raise DriverError(str(error)) from error
        except (ValueError, RuntimeError, TypeError) as error:
            raise DriverError(f"signal layer: {type(error).__name__}: {error}") from error

    def recording(self) -> RecordingView:
        if self._view is None:
            raise DriverError("no recording before run")
        return self._view


def _canonical_crossings(record: TickRecord) -> TickRecord:
    """Tier 1 crossing events with the detail text of `IntegrityCrossing.detail()` (the party writes the same JSON
    with C++ number formatting)."""
    if not any(event.kind == INTEGRITY_KIND for event in record.events):
        return record
    events = []
    for event in record.events:
        if event.kind == INTEGRITY_KIND:
            event = replace(event, detail=IntegrityCrossing.parse(event.detail).detail())
        events.append(event)
    return replace(record, events=tuple(events))


def model_columns(model: PhysicsModel, payloads: Sequence[str] = ()) -> dict[str, Columns]:
    """Column labels of the index-column series from the neutral model (REC-018, ruling 18): joints in body order
    (prismatic m and N, revolute and the rest rad and N m), transmissions and payload bodies by name."""
    joints: list[str] = []
    positions: list[str] = []
    torques: list[str] = []
    for body in model.bodies:
        if body.joint is None:
            continue
        joints.append(body.joint.name)
        if body.joint.type == "prismatic":
            positions.append("m")
            torques.append("N")
        else:
            positions.append("rad")
            torques.append("N*m")
    columns = {
        JOINT_POSITIONS: Columns(tuple(joints), tuple(positions)),
        DRIVE_TORQUE: Columns(tuple(joints), tuple(torques)),
        DRIVE_DRIVEN: Columns(tuple(joints)),
    }
    transmissions = tuple(item.name for item in model.transmissions)
    if len(transmissions) > 0:
        for field in ("slip", "slipping", "shaft_load_torque", "force", "stretch", "power_flow"):
            columns[f"physics/transmissions/{field}"] = Columns(transmissions)
    if len(payloads) > 0:
        for field in ("body", "carrier", "attached", "force"):
            columns[f"physics/payloads/{field}"] = Columns(tuple(payloads))
    members = tuple(member.structural_member or member.name for member in model.lumped_members)
    if len(members) > 0:
        for path in MEMBER_SERIES:
            columns[path] = Columns(members)
    ropes = tuple(rope.name for rope in model.ropes)
    if len(ropes) > 0:
        for path in ROPE_SERIES:
            columns[path] = Columns(ropes)
    return columns


# ===== Member frames ====================================================


def _rotate(q: Quat, v: Vec) -> Vec:
    w, x, y, z = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


def _multiply(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _conjugate(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def member_frames(model: PhysicsModel) -> tuple[tuple[int, Quat], ...]:
    """Per lumped member, in neutral order: the index of its parent body (-1 for world) and its root frame's
    orientation in the parent frame (x along the member, ruling 24)."""
    index = {body.name: position for position, body in enumerate(model.bodies)}
    frames: list[tuple[int, Quat]] = []
    for member in model.lumped_members:
        parent = index.get(member.parent, -1)
        q = member.orientation
        frames.append((parent, (float(q[0]), float(q[1]), float(q[2]), float(q[3]))))
    return tuple(frames)


def member_frame_loads(record: TickRecord, frames: Sequence[tuple[int, Quat]]) -> TickRecord:
    """The bridge publishes member end-loads along world axes; ENG-030 states them in the member frame (x from the
    start to the end node, y and z the section's principal axes; ruling 24). Each wrench is rotated by the inverse
    of the member root frame's world orientation, the parent body's recorded orientation times the member's placement;
    the root frame stands for the member frame while the deflection is small. The native adapter publishes the member
    frame directly (the engine's member root frame axes, ENG-030), so `CoreDriver` applies this on the bridge only and
    never twice."""
    members = record.physics.members
    if len(members) == 0:
        return record
    if len(members) != len(frames):
        raise DriverError(f"{len(members)} member end-loads for {len(frames)} lumped members")
    bodies = record.physics.bodies
    loads: list[MemberEndLoads] = []
    for loads_of, (parent, placement) in zip(members, frames, strict=True):
        world = placement
        if parent >= 0:
            world = _multiply(bodies[parent].orientation, placement)
        inverse = _conjugate(world)
        loads.append(
            MemberEndLoads(
                Wrench(_rotate(inverse, loads_of.start.moment), _rotate(inverse, loads_of.start.force)),
                Wrench(_rotate(inverse, loads_of.end.moment), _rotate(inverse, loads_of.end.force)),
            )
        )
    return replace(record, physics=replace(record.physics, members=tuple(loads)))


# ===== Archives =========================================================


def _pairs(pairs: tuple[tuple[str, str], ...] | None) -> list[list[str]] | None:
    if pairs is None:
        return None
    return [list(item) for item in pairs]


def _case_file(case_hash: str, root: Path) -> bytes:
    for path in sorted(root.rglob("*.yaml")):
        if blob_hash(path) == case_hash:
            return path.read_bytes()
    return b""


class RecordArchiver:
    """`Archiver` through `mechsim.record`: identity, end, rows, kinematics and run metadata in one HDF5 file."""

    def __init__(self, designs_root: Path, cases_root: Path = CASES_ROOT, repository: Path = ROOT) -> None:
        self._designs_root = designs_root
        self._cases_root = cases_root
        self._repository = repository

    def write(self, path: Path, recording: Any, metadata: RunMetadata) -> str:
        if not isinstance(recording, RecordingView):
            raise DriverError("the record archiver writes recordings of the core driver")
        report = metadata.report
        design_path = self._designs_root / metadata.design_name / "design.yaml"
        design_file = b""
        if design_path.is_file():
            design_file = design_path.read_bytes()
        control_rate = recording.recording.base_rate
        if metadata.control_rate is not None:
            control_rate = metadata.control_rate
        identity = RunIdentity(
            design_name=metadata.design_name,
            design_version=metadata.design_version,
            design_source_hash=metadata.design_hash,
            test_case_id=metadata.case_id,
            test_case_hash=metadata.case_hash,
            seed=metadata.seed,
            controller_mode=metadata.controller_mode or "none",
            physics_rate=recording.recording.base_rate,
            control_rate=control_rate,
            repository_commit=repository_commit(self._repository),
            tool_version=tool_version(),
            design_file=design_file,
            test_case_file=_case_file(metadata.case_hash, self._cases_root),
            physics_adapter=metadata.physics_adapter,
        )
        fault: RunFault | None = None
        outcome = "end_condition_met"
        if report.errored:
            outcome = "run_error"
            fault = RunFault("run", report.end_tick, report.error or "")
        end = RunEnd(outcome, report.end_tick, report.end_tick, fault)
        extra = {
            "run_id": metadata.run_id,
            "end_reason": report.reason,
            "error": report.error or "",
            "sweep": json.dumps(metadata.sweep),
            "bindings": json.dumps([list(item) for item in metadata.bindings]),
            "kinematics": recording.kinematics.to_json(),
            "approximations": json.dumps(list(recording.approximations)),
            "labels": json.dumps(recording.labels, sort_keys=True),
            "actuators": json.dumps(recording.actuators, sort_keys=True),
            "states": json.dumps(recording.states, sort_keys=True),
            "sink_errors": json.dumps(list(report.sink_errors)),
            "hook_errors": json.dumps(list(report.hook_errors)),
            "members": members_json(metadata.members),
            **_notes_entry(report.notes),
            "variation": json.dumps(_pairs(metadata.variation)),
        }
        if metadata.material_overrides is not None:
            extra[MATERIAL_OVERRIDES_KEY] = metadata.material_overrides.text
            extra[f"{MATERIAL_OVERRIDES_KEY}_sha256"] = metadata.material_overrides.sha256
        if metadata.integrity is not None:
            extra["integrity"] = metadata.integrity.to_json()
        write_record_archive(path, identity, end, recording.recording, extra)
        return hashlib.sha256(path.read_bytes()).hexdigest()


def _notes_entry(notes: tuple[str, ...]) -> dict[str, str]:
    """Archive key of the run's report notes, written only when there are any so other archives keep their bytes."""
    if len(notes) == 0:
        return {}
    return {"notes": json.dumps(list(notes))}


def _rope_snap_gate(model: PhysicsModel, integrity: IntegrityConfig | None) -> tuple[PhysicsModel, tuple[str, ...]]:
    """Rope snap is a tier 2 entry of `integrity.yaml` (INT-031, ruling 53): without it every rope's breaking tension
    is dropped from the run's model, with one report line per rope that stated one."""
    if integrity is not None and integrity.rope_snap():
        return model, ()
    stated = [rope for rope in model.ropes if rope.breaking_tension is not None]
    if len(stated) == 0:
        return model, ()
    ropes = tuple(rope.model_copy(update={"breaking_tension": None}) for rope in model.ropes)
    lines = tuple(ROPE_SNAP_OFF.format(rope.name) for rope in stated)
    return model.model_copy(update={"ropes": ropes}), lines


# ===== Services =========================================================


def core_builder(design: Design, adapter: str = DEFAULT_PHYSICS_ADAPTER) -> CoreBuild:
    """The shipped catalog and chips, the shipped materials with the design's overrides (MAT-020); lumped members
    rigid on the bridge until X2 (BLD-033, ruling 29), flexible as the design states them on the native adapter."""
    rigid = adapter != NATIVE_ADAPTER
    try:
        return build(design, default_registry(), design_materials(design), rigid_members=rigid)
    except (BuildError, DesignError, MaterialError) as error:
        raise DriverError(str(error)) from error


def core_services(designs_root: Path = DESIGNS_ROOT) -> Services:
    """The runner's real services: a fresh `CoreDriver` per run, the builder, the record archiver, and integrity
    over the recording when the design has an `integrity.yaml` beside its design file (INT-022)."""
    return Services(
        driver=partial(CoreDriver, designs_root=designs_root),
        builder=core_builder,
        archiver=RecordArchiver(designs_root),
        designs_root=designs_root,
        integrity=IntegrityService(designs_root),
    )
