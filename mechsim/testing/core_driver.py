"""The real simulation driver over the core binding (TST-012, TST-021, TST-036), the record view scoring reads, and
the archive writer and reader the runner and the CLI use (TST-033, L8).

`CoreDriver.load` prepares the neutral model for the MuJoCo bridge (translation report kept, ADR 0003), `run` builds
a co-simulation kernel with the physics party and records every row through an in-memory sink into
`mechsim.record.Recorder`. Joint coordinates and poses are derived from the recorded body poses with the model's
kinematics (TST-038): a joint coordinate is zero at the neutral placement; a pose is the body frame's displacement
and rotation from its reference-configuration pose (every joint coordinate zero), world axes.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Callable, Mapping, Sequence
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np

from mechsim import _core as core
from mechsim.builder import BuildError, build
from mechsim.builder.result import BuildResult as CoreBuild
from mechsim.design.schema import Design, blob_hash, shipped_materials
from mechsim.library.registry import default_registry
from mechsim.physics.mujoco.prepare import StartStateError, prepare_model
from mechsim.physics.mujoco.translate import UnmappableElementError
from mechsim.physics.neutral import PhysicsModel
from mechsim.record.archive import RunEnd, RunFault, RunIdentity, repository_commit, tool_version
from mechsim.record.archive import write_archive as write_record_archive
from mechsim.record.reader import DriverError as DriverError
from mechsim.record.reader import JointKinematics as JointKinematics
from mechsim.record.reader import Kinematics as Kinematics
from mechsim.record.reader import RecordArchiveReader as RecordArchiveReader
from mechsim.record.reader import RecordingView as RecordingView
from mechsim.record.stream import Recorder
from mechsim.record.tick import (
    BodyState,
    ConstraintForce,
    ContactForce,
    CouplingSample,
    JointForce,
    MemberEndLoads,
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
    SENSOR_FAULTS,
    RunnerParty,
    RunnerState,
    WiringError,
    controller_party,
    operator_message,
    power_on_state,
    signal_layer,
    work_directory,
)
from mechsim.testing.driver import (
    BuildResult,
    EndReason,
    EndSpec,
    RunConfig,
    RunMetadata,
    RunReport,
    ScheduledCommand,
    ScheduledFault,
)
from mechsim.testing.runner import Services
from mechsim.testing.schema import DESIGNS_ROOT, ROOT, Case, InitialState, Quantity

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


def tick_record(row: Any) -> TickRecord:
    """The record module's frozen mirror of one bound `TickRecord` (REC-020: a copy, never kernel memory)."""
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
        ContactForce(int(c["body_a"]), int(c["body_b"]), _vec(c["point"]), _vec(c["force"]))
        for c in physics["contacts"]
    )
    members = tuple(
        MemberEndLoads(
            Wrench(_vec(m["start"]["moment"]), _vec(m["start"]["force"])),
            Wrench(_vec(m["end"]["moment"]), _vec(m["end"]["force"])),
        )
        for m in physics["members"]
    )
    ropes = tuple(RopeTension(float(r["tension"]), bool(r["slack"])) for r in physics["ropes"])
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

    coupling = data["coupling"]
    return TickRecord(
        time=int(data["time"]),
        physics=PhysicsOutputs(
            int(physics["steps"]), bodies, joints, constraints, contacts, members, ropes, transmissions
        ),
        pin_signals=samples(data["pin_signals"]),
        rails=samples(data["rails"]),
        actuator_efforts=samples(data["actuator_efforts"]),
        controller_state=str(data["controller_state"]),
        events=tuple(RowEvent(int(e["time"]), e["source"], e["kind"], e["detail"]) for e in data["events"]),
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


class CoreDriver:
    """`SimulationDriver` over the core binding. One driver per run (the runner's factory makes a fresh one)."""

    def __init__(self, subscribers: Sequence[Callable[[Any], None]] = ()) -> None:
        """`subscribers` receive each row as the kernel writes it (REC-019); one that raises is a recorder error,
        which ends the scoring as a run error and lands in the archive metadata (invariant 7)."""
        self._subscribers = tuple(subscribers)
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

    def load(self, build: BuildResult, case: Case) -> None:
        if not isinstance(build, CoreBuild):
            raise DriverError("the core driver runs builds of mechsim.builder")
        if build.physics is None:
            raise DriverError("the design's physics is pending: " + ", ".join(build.pending))
        self._build = build
        self._model = build.physics

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
        self._config = config

    def set_initial_state(self, state: InitialState) -> None:
        if state.payload is not None and state.payload.attached:
            raise DriverError("initial_state.payload: payload attachment has no bridge input in milestone 1")
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
        if fault.kind not in SENSOR_FAULTS and fault.kind != "estop_press":
            raise DriverError(
                f"fault '{fault.kind}': delivery to the {fault.owner} party is milestone 2 (KER-023 delivers sensor, "
                "motor-stall and E-stop faults in milestone 1)"
            )
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
        except (StartStateError, UnmappableElementError) as error:
            raise DriverError(str(error)) from error
        except WiringError as error:
            raise DriverError(str(error)) from error
        except (ValueError, RuntimeError, TypeError) as error:
            raise DriverError(f"{type(error).__name__}: {error}") from error

    def _run(self, end: EndSpec, config: RunConfig, model: PhysicsModel) -> RunReport:
        """Binding errors (AdapterError, KernelConfigError, SignalConfigError and the rest, all ValueError or
        RuntimeError) escape to `run`, which types them as DriverError with the cause kept (invariant 7)."""
        artifacts = prepare_model(model, self._starts, float(1 / config.physics_rate))
        self.approximations = artifacts.report
        adapter = core.make_physics_adapter("mujoco")
        prepared = core.PreparedModel(artifacts.neutral_json, artifacts.mjcf, artifacts.version)
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
        kernel.register_party(core.PhysicsParty(adapter, party))
        self.labels = {}
        if signals:
            self._register_signals(kernel, config)
        stop = end.max_tick
        if end.at_tick is not None:
            stop = min(stop, end.at_tick)
        sink = core.InMemoryRecorderSink()
        for subscriber in self._subscribers:
            sink.subscribe(subscriber)
        report = kernel.run(core.EndCondition(_seconds(stop, config.base_rate)), sink)
        recorder = Recorder(config.base_rate)
        for row in sink.drain():
            recorder.write_row(tick_record(row))
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
            reason, int(report.ticks), config.base_rate, error, tuple(sink_errors), hook_errors, self.approximations
        )

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

    def _register_signals(self, kernel: Any, config: RunConfig) -> None:
        build = self._build
        model = self._model
        if build is None or model is None:
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
            )
            state = RunnerState(externals=externals)
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
                kernel.register_party(controller_party(build, config.control_rate, work_directory()))
        except WiringError as error:
            raise DriverError(str(error)) from error
        except (ValueError, RuntimeError, TypeError) as error:
            raise DriverError(f"signal layer: {type(error).__name__}: {error}") from error

    def recording(self) -> RecordingView:
        if self._view is None:
            raise DriverError("no recording before run")
        return self._view


# ===== Archives =========================================================


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
        }
        write_record_archive(path, identity, end, recording.recording, extra)
        return hashlib.sha256(path.read_bytes()).hexdigest()


# ===== Services =========================================================


def core_builder(design: Design) -> CoreBuild:
    """The shipped catalog, materials and chips; rigid members (milestone 1 runs rigid, BLD-033)."""
    try:
        return build(design, default_registry(), shipped_materials(), rigid_members=True)
    except BuildError as error:
        raise DriverError(str(error)) from error


def core_services(designs_root: Path = DESIGNS_ROOT) -> Services:
    """The runner's real services: a fresh `CoreDriver` per run, the builder, the record archiver."""
    return Services(
        driver=CoreDriver, builder=core_builder, archiver=RecordArchiver(designs_root), designs_root=designs_root
    )
