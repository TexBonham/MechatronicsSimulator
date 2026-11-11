"""Signal layer, operator link and native controller assembled from a build (TST-012, TST-013, TST-036, KER-023).

`assemble` turns the builder's wiring table and the neutral model into the core parties: the `SignalLayer` (slots from
the pin table, driver-to-motor connections from the coil wires, rails with their gates and consumers, each stepper's
joint drive on its shaft joint with its torque recorded, sensors on their sensed joints, BLD-035), the operator link
from the design's messages, the `ControllerParty` over the native runtime, and the runner party that delivers scheduled
operator commands and E-stop presses at its first step at or after their tick (KER-023). Sensor and actuator faults are
injected into their component before t = 0 with their tick window, so the component applies them on its first step
inside it.

Milestone 2 instances (BLD-035): a signal wire between two layer instances (an output port to an input port, not
power, ground or a driver-to-motor coil) is a layer connection; a load cell reads its payload's attachment force by the
payload's index among the model's payload bodies; the amplifier converts with the design chip's ADC (`Chip.adc`); a
brake registers at the actuator stage by type and acts on its `acts_on` joint. On a prismatic joint driven by a
transmission the brake sits on that transmission's shaft: its holding force is brake_torque / c and its drive's inverse
inertia A0 = 1 / (m + sum J / c^2), m the mass of the joint's child body and every body below it, J the axial inertia
of each driving shaft body (the rotor inertia, BLD-034), c the transmission's metres per radian (lead / 2 pi for a lead
screw, the pulley radius for a belt): the joint's inverse inertia at the reference configuration with each rotor
reflected rigidly, which is MuJoCo's dof_invweight0 for that joint on the bridge. A payload welded to a carrier is a
separate free body and does not enter m. On a revolute joint the brake torque applies as given and A0 is not set.

Firmware event-log entries whose code the design names in `firmware.event_codes` are
collected by the runner party from `controller.pin_outputs` and recorded as `controller` events of that name, the
argument read as an axis: the instance of the pulse_output binding with that channel, else the number.
"""

from __future__ import annotations

import hashlib
import math
import tempfile
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from fractions import Fraction
from pathlib import Path
from typing import Any

from mechsim import _core as core
from mechsim.builder.result import BuildResult as CoreBuild
from mechsim.builder.result import InstanceWiring
from mechsim.design.chip import Chip, shipped_chips
from mechsim.library.registry import default_registry
from mechsim.physics.neutral import DOF_COUNTS, Body, PhysicsModel
from mechsim.testing.driver import ScheduledCommand, ScheduledFault
from mechsim.testing.runner import RunnerError
from mechsim.testing.schema import ROOT

STAGES: dict[str, Any] = {
    "power": core.Stage.power,
    "safety": core.Stage.power,
    "sensor": core.Stage.sensor,
    "front_end": core.Stage.front_end,
    "driver": core.Stage.driver,
    "actuator": core.Stage.actuator,
}
PIN_FIELDS: dict[str, Any] = {
    "digital_input": core.PinField.digital_in,
    "counter_input": core.PinField.counter_delta,
    "adc_input": core.PinField.adc_sample,
    "uart_rx": core.PinField.uart_rx,
    "pulse_output": core.PinField.step_count,
    "digital_output": core.PinField.digital_out,
    "uart_tx": core.PinField.uart_tx,
}
PIN_SLOTS: dict[str, Any] = {
    "digital_input": core.PinSlot.digital_in,
    "counter_input": core.PinSlot.counter_delta,
    "adc_input": core.PinSlot.adc_sample,
    "uart_rx": core.PinSlot.uart_rx,
    "pulse_output": core.PinSlot.step_count,
    "digital_output": core.PinSlot.digital_out,
    "uart_tx": core.PinSlot.uart_tx,
}
SENSOR_FAULTS: dict[str, Any] = {
    "sensor_stuck": core.FailureMode.stuck,
    "sensor_noisy": core.FailureMode.noisy,
    "sensor_dropped": core.FailureMode.dropped,
    "sensor_forced": core.FailureMode.forced_level,
    "motor_stall": core.FailureMode.stall,
}
LAYER_FAULTS: dict[str, Any] = {**SENSOR_FAULTS, "power_loss": core.FailureMode.power_loss}
WIRED_FAULTS: tuple[str, ...] = (*LAYER_FAULTS, "estop_press")
DRIVER_TO_MOTOR = ("full_steps", "direction", "enabled", "supply")
RAIL_INPUTS = {"step_dir_driver": "vmot", "brake": "vcc"}
TYPE_STAGES: dict[str, Any] = {"brake": core.Stage.actuator}
SIGNAL_PORT_KINDS = ("power", "ground")
CONTROLLER_OUTPUTS = "controller.pin_outputs"
ESTOP_TYPE = "estop_relay"
OPERATOR_LINK_TYPE = "uart_link"
NATIVE_BUILD_DIR = Path(tempfile.gettempdir()) / "mechsim-native-build"
_LIBRARIES: dict[tuple[str, int, str], str] = {}


class WiringError(RunnerError):
    """The build cannot be assembled into core parties; the message names the instance and what is missing."""


@dataclass
class RunnerState:
    """What the runner party delivers, kept outside the party object (the kernel owns the party, ADR 0005)."""

    commands: list[tuple[int, Any]] = field(default_factory=list)
    presses: list[tuple[str, int, int]] = field(default_factory=list)
    externals: dict[str, float] = field(default_factory=dict)
    posted: int = 0
    sequence: int = 0
    messages: list[Any] = field(default_factory=list)
    tick: int = 0
    faults: list[str] = field(default_factory=list)
    hook_errors: list[str] = field(default_factory=list)
    event_names: dict[int, str] = field(default_factory=dict)
    axes: dict[int, str] = field(default_factory=dict)
    control_period: int = 1
    events: list[tuple[int, str, str]] = field(default_factory=list)
    pending: list[tuple[int, int]] = field(default_factory=list)

    def note_fault(self, fault: Any) -> None:
        self.faults.append(f"{fault.party}: {fault.cause}")


class RunnerParty(core.Party):  # type: ignore[misc]
    """Publishes `signals.operator_commands` and `signals.external_inputs` (KER-008): a command or E-stop press reaches
    the signal layer at this party's first step at or after its tick (KER-023)."""

    def __init__(self, state: RunnerState, rate: Any) -> None:
        super().__init__()
        self._state = state
        self._rate = rate

    def name(self) -> str:
        return "runner"

    def nominal_rate(self) -> Any:
        return self._rate

    def published(self) -> list[str]:
        names = ["signals.operator_commands"]
        if len(self._state.externals) > 0:
            names.append("signals.external_inputs")
        return names

    def consumed(self) -> list[str]:
        if len(self._state.event_names) > 0:
            return [CONTROLLER_OUTPUTS]
        return []

    def start(self, base_rate: Any) -> None:
        self._state.posted = 0
        self._state.events = []

    def read_inputs(self, inputs: Any) -> None:
        state = self._state
        state.pending = []
        if len(state.event_names) == 0 or not inputs.has_value(CONTROLLER_OUTPUTS):
            return
        outputs = inputs.value(CONTROLLER_OUTPUTS)
        state.pending = [
            (int(outputs.event_log[index].code), int(outputs.event_log[index].arg))
            for index in range(int(outputs.event_count))
        ]

    def step(self, t: Any, dt: Any) -> None:
        """Controller outputs are read once per controller step (t a multiple of its period, before the controller
        steps at t), so each tick's event log is recorded once, stamped with the controller step that wrote it."""
        state = self._state
        state.tick = int(t.count())
        if state.tick == 0 or state.tick % state.control_period != 0:
            return
        for code, arg in state.pending:
            name = state.event_names.get(code)
            if name is not None:
                state.events.append((state.tick - state.control_period, name, state.axes.get(arg, str(arg))))

    def write_outputs(self, outputs: Any) -> None:
        state = self._state
        due: list[Any] = []
        while state.posted < len(state.commands) and state.commands[state.posted][0] <= state.tick:
            due.append(state.commands[state.posted][1])
            state.posted += 1
        if len(due) > 0:
            state.sequence += 1
            state.messages = due
        outputs.set("signals.operator_commands", core.OperatorCommands(state.sequence, state.messages))
        if len(state.externals) > 0:
            levels = dict(state.externals)
            for instance, start, end in state.presses:
                if start <= state.tick < end:
                    levels[f"{instance}.estop_loop"] = 0.0
            outputs.set("signals.external_inputs", levels)

    def on_fault(self, fault: Any) -> None:
        """A raise here could not propagate (the kernel calls on_fault noexcept), so it is kept for the run report."""
        try:
            self._state.note_fault(fault)
        except Exception as error:
            self._state.hook_errors.append(f"runner.on_fault: {type(error).__name__}: {error}")


@dataclass(frozen=True)
class JointPlace:
    body: int
    parent: int | None
    kind: str
    axis: tuple[float, float, float]
    offset: float
    dof: int


def _rotate(q: Sequence[float], v: Sequence[float]) -> tuple[float, float, float]:
    w, x, y, z = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


def joint_places(model: PhysicsModel) -> tuple[dict[str, JointPlace], int]:
    """Each joint's child and parent body index, kind, axis in the parent frame, the linear reading at the neutral
    placement, and its first generalized-force index (neutral body order, DOF_COUNTS per joint); and the DOF total."""
    index = {body.name: position for position, body in enumerate(model.bodies)}
    places: dict[str, JointPlace] = {}
    dof = 0
    for body in model.bodies:
        joint = body.joint
        if joint is None:
            raise WiringError(f"body '{body.name}': no joint to its parent")
        parent: int | None = None
        if body.parent != "world":
            parent = index[body.parent]
        raw = (0.0, 0.0, 1.0)
        if joint.axis is not None:
            norm = math.sqrt(sum(value * value for value in joint.axis))
            raw = (joint.axis[0] / norm, joint.axis[1] / norm, joint.axis[2] / norm)
        axis = _rotate(joint.orientation, raw)
        offset = sum(p * a for p, a in zip(joint.position, axis, strict=True))
        places[joint.name] = JointPlace(index[body.name], parent, joint.type, axis, offset, dof)
        linear, angular = DOF_COUNTS[joint.type]
        dof += linear + angular
    return places, dof


def _coordinate(place: JointPlace, instance: str) -> Any:
    if place.kind == "prismatic":
        kind = core.CoordinateKind.linear
    elif place.kind == "revolute":
        kind = core.CoordinateKind.angular
    else:
        raise WiringError(f"sensor '{instance}': senses a {place.kind} joint; only revolute and prismatic are read")
    return core.JointCoordinate(kind, place.body, place.parent, list(place.axis))


def _shaft_joint(instance: InstanceWiring) -> str:
    for mount in instance.mounts:
        if mount.kind == "shaft" and mount.joint is not None:
            return mount.joint
    raise WiringError(f"actuator '{instance.name}': no shaft mount bound to a joint")


def axial_inertia(body: Body) -> float:
    """a^T I a about the body's joint axis (joint frame, as the builder sets it, BLD-034); z without an axis."""
    axis: Sequence[float] = (0.0, 0.0, 1.0)
    if body.joint is not None and body.joint.axis is not None:
        axis = body.joint.axis
    norm = math.sqrt(sum(value * value for value in axis))
    a = [value / norm for value in axis]
    i = body.inertia
    tensor = ((i.ixx, i.ixy, i.ixz), (i.ixy, i.iyy, i.iyz), (i.ixz, i.iyz, i.izz))
    return sum(a[r] * tensor[r][c] * a[c] for r in range(3) for c in range(3))


def _fidelity(values: Mapping[str, float | bool], params: dict[str, Any]) -> None:
    for name in ("noise", "delay", "sample_rate"):
        if name in values:
            params[name] = values[name]


def _subtree_mass(model: PhysicsModel, root: str) -> float:
    children: dict[str, list[str]] = {}
    for body in model.bodies:
        children.setdefault(body.parent, []).append(body.name)
    masses = {body.name: body.mass for body in model.bodies}
    total = 0.0
    frontier = [root]
    while len(frontier) > 0:
        name = frontier.pop()
        total += masses[name]
        frontier.extend(children.get(name, []))
    return total


def _metres_per_radian(transmission: Any) -> float:
    if transmission.type == "lead_screw":
        return float(transmission.lead) / (2.0 * math.pi)
    if transmission.type == "belt_pulley":
        return float(transmission.pulley_radius)
    raise WiringError(f"transmission '{transmission.name}': a {transmission.type} drives no prismatic joint")


def brake_parameters(instance: InstanceWiring, places: Mapping[str, JointPlace], model: PhysicsModel) -> dict[str, Any]:
    """Brake on its `acts_on` joint (module docstring): holding limit and A0 on a driven prismatic joint."""
    values = instance.parameters
    joint = instance.acts_on
    if joint is None or joint not in places:
        raise WiringError(f"brake '{instance.name}': acts on no joint of the model (DES-043 acts_on)")
    place = places[joint]
    params: dict[str, Any] = {
        "joint": place.body,
        "brake_torque": values["brake_torque"],
        "engage_delay": values["engage_delay"],
        "release_delay": values["release_delay"],
        "power_off_engaged": values["power_off_engaged"],
    }
    if place.kind == "revolute":
        return params
    if place.kind != "prismatic":
        raise WiringError(f"brake '{instance.name}': acts on a {place.kind} joint; only revolute and prismatic")
    drives = [item for item in model.transmissions if item.driven_joint == joint]
    if len(drives) == 0:
        raise WiringError(f"brake '{instance.name}': prismatic joint '{joint}' has no transmission shaft to brake")
    shaft_bodies = {
        body.joint.name: body for body in model.bodies if body.joint is not None and body.joint.type == "revolute"
    }
    child = model.bodies[place.body].name
    effective = _subtree_mass(model, child)
    for drive in drives:
        c = _metres_per_radian(drive)
        effective += axial_inertia(shaft_bodies[drive.shaft_joint]) / (c * c)
    params["brake_torque"] = float(values["brake_torque"]) / _metres_per_radian(drives[0])
    params["inverse_inertia"] = 1.0 / effective
    return params


def component_parameters(
    instance: InstanceWiring,
    places: Mapping[str, JointPlace],
    model: PhysicsModel,
    payloads: Sequence[str] = (),
    chip: Chip | None = None,
) -> dict[str, Any]:
    """The `make_signal_component` parameters of one instance from its part values and placement (BLD-035);
    `payloads` are the model's payload bodies in body order, `chip` the design's chip (ADC of an
    amplifier)."""
    values = instance.parameters
    if instance.type == "load_cell":
        senses = instance.senses
        if senses is None:
            raise WiringError(f"load cell '{instance.name}': no senses block (DES-043)")
        cell: dict[str, Any] = {
            name: values[name]
            for name in (
                "capacity",
                "sensitivity",
                "excitation_voltage",
                "resolution",
                "overload_limit",
                "nonlinearity",
            )
            if name in values
        }
        _fidelity(values, cell)
        if senses.payload is not None:
            if senses.payload not in payloads:
                raise WiringError(f"load cell '{instance.name}': the run has no payload '{senses.payload}'")
            cell["payload"] = list(payloads).index(senses.payload)
            return cell
        if senses.joint is None or senses.joint not in places:
            raise WiringError(f"load cell '{instance.name}': senses unknown joint '{senses.joint}'")
        cell["joint"] = places[senses.joint].body
        return cell
    if instance.type == "imu":
        senses = instance.senses
        names = [body.name for body in model.bodies]
        if senses is None or senses.body is None or senses.body not in names:
            raise WiringError(f"imu '{instance.name}': senses no body of the model (DES-043)")
        found: dict[str, Any] = {
            "body": names.index(senses.body),
            "gravity": tuple(float(value) for value in model.gravity),
        }
        for name in ("accel_range", "gyro_range", "resolution", "gyro_resolution", "gyro_noise"):
            if name in values:
                found[name] = values[name]
        _fidelity(values, found)
        return found
    if instance.type == "load_cell_amplifier":
        if chip is None:
            raise WiringError(f"amplifier '{instance.name}': the design names no chip, so no ADC")
        amp: dict[str, Any] = {
            name: values[name] for name in ("gain", "offset", "output_full_scale", "bandwidth") if name in values
        }
        amp["adc_bits"] = int(chip.adc.resolution_bits)
        amp["adc_reference"] = float(chip.adc.reference_voltage)
        return amp
    if instance.type == "brake":
        return brake_parameters(instance, places, model)
    if instance.category == "sensor":
        senses = instance.senses
        if senses is None:
            raise WiringError(f"sensor '{instance.name}': no senses block (DES-043)")
        if senses.joint is None or senses.joint not in places:
            raise WiringError(f"sensor '{instance.name}': senses unknown joint '{senses.joint}'")
        place = places[senses.joint]
        params: dict[str, Any] = {"coordinate": _coordinate(place, instance.name)}
        if instance.type == "limit_switch":
            if senses.position is None or senses.actuate_above is None:
                raise WiringError(f"limit switch '{instance.name}': needs position and actuate_above (DES-043)")
            params["position"] = senses.position + place.offset
            params["actuate_above"] = senses.actuate_above
            for name in ("hysteresis", "resolution", "debounce_time", "normally_open"):
                if name in values:
                    params[name] = values[name]
        else:
            params["resolution"] = values["resolution"]
            params["reversed"] = senses.reversed
        return params
    if instance.type == "step_dir_driver":
        return {"microsteps": int(values["microsteps"])}
    if instance.type == "stepper_behavioral":
        shaft = _shaft_joint(instance)
        transmissions = [item.name for item in model.transmissions if item.shaft_joint == shaft]
        if len(transmissions) != 1:
            raise WiringError(f"actuator '{instance.name}': {len(transmissions)} transmissions on shaft '{shaft}'")
        if len(instance.torque_speed) == 0:
            raise WiringError(f"actuator '{instance.name}': part '{instance.part}' has no torque-speed curve")
        position = [item.name for item in model.transmissions].index(transmissions[0])
        body = model.bodies[places[shaft].body]
        return {
            "step_angle": values["step_angle"],
            "rotor_inertia": values["rotor_inertia"],
            "holding_torque": values["holding_torque"],
            "damping": values["viscous_damping"],
            "joint": places[shaft].body,
            "torque_speed": list(instance.torque_speed),
            "transmission": position,
            "shaft_axial_inertia": axial_inertia(body),
            "shaft_body": body.name,
        }
    if instance.type == "power_supply":
        return {"voltage": values["output_voltage"]}
    if instance.type == "estop_relay":
        return {"cutoff_delay": values["cutoff_delay"], "manual_reset": values["manual_reset"]}
    if instance.type == "watchdog":
        return {
            "timeout": values["timeout"],
            "cutoff_delay": values["cutoff_delay"],
            "manual_reset": values["manual_reset"],
        }
    raise WiringError(f"instance '{instance.name}': type '{instance.type}' has no signal-layer model")


def _ticks(tick: int) -> Any:
    return core.SimTime(tick)


def _injection(fault: ScheduledFault, base_rate: Fraction) -> Any:
    if fault.end_tick is None:
        raise WiringError(f"fault '{fault.kind}': needs a window")
    injection = core.FailureInjection()
    injection.mode = LAYER_FAULTS[fault.kind]
    injection.start = _ticks(fault.start_tick)
    injection.end = _ticks(fault.end_tick)
    parameters = dict(fault.parameters)
    if fault.kind == "sensor_noisy":
        injection.noise_magnitude = float(parameters["standard_deviation"])
    if fault.kind == "sensor_forced":
        level = 0.0
        if parameters["level"] == "high":
            level = 1.0
        injection.forced_level = level
    if fault.kind == "motor_stall":
        injection.fraction = float(parameters["available_torque"])
    return injection


@dataclass
class Assembly:
    """The parties for one run in registration order, the runner's state and the operator label table."""

    parties: list[Any]
    runner: RunnerState
    labels: dict[str, str]
    velocity_dofs: int


def _message_specs(build: CoreBuild) -> tuple[list[Any], dict[str, str]]:
    specs: list[Any] = []
    labels: dict[str, str] = {}
    for message in build.design.operator_messages:
        direction = core.MessageDirection.from_controller
        if message.destination == "controller":
            direction = core.MessageDirection.to_controller
        specs.append(core.MessageSpec(message.name, message.label, [item.name for item in message.fields], direction))
        labels[message.label] = message.name
    return specs, labels


def host_build_dir(tree: Path, tick_hz: int, target: str) -> Path:
    """One host build directory per firmware tree, target and tick rate: a CMake build tree binds to one source tree,
    so two checkouts sharing a temp directory must not share a build."""
    tree_key = hashlib.sha1(str(tree.resolve()).encode()).hexdigest()[:12]
    return NATIVE_BUILD_DIR / f"{target}-{tick_hz}-{tree_key}"


def _library(tree: Path, tick_hz: int, target: str) -> str:
    key = (str(tree), tick_hz, target)
    if key not in _LIBRARIES:
        request = core.HostBuildRequest()
        request.firmware_tree = str(tree)
        request.work_dir = str(host_build_dir(tree, tick_hz, target))
        request.tick_hz = tick_hz
        request.app_target = target
        try:
            result = core.build_host_firmware(request)
        except core.ControllerConfigError as error:
            raise WiringError(f"firmware host build failed: {error}") from error
        _LIBRARIES[key] = str(result.library)
    return _LIBRARIES[key]


def work_directory() -> Path:
    """The per-process directory the native runtime copies firmware libraries into (CTL-012 isolation)."""
    path = NATIVE_BUILD_DIR / "runs"
    path.mkdir(parents=True, exist_ok=True)
    return path


def app_target(build: CoreBuild) -> str:
    """The firmware tree's application target of a design: `mechsim_fw_<design name>` (firmware/CMakeLists.txt)."""
    return f"mechsim_fw_{build.design.name}"


def native_runtime(build: CoreBuild, control_rate: Fraction, work_dir: Path) -> Any:
    """A loaded native runtime over the design's firmware (CTL-002 to CTL-007)."""
    firmware = build.design.firmware
    if firmware is None or build.chip is None:
        raise WiringError("the design has no firmware")
    if control_rate.denominator != 1:
        raise WiringError(f"control rate {control_rate} Hz is not a whole number of hertz")
    tick_hz = int(control_rate)
    channels = core.ChannelMap()
    steps: dict[str, int] = {}
    outs: dict[str, int] = {}
    for binding in build.chip.bindings:
        channels.add(core.ChannelAssignment(binding.signal, PIN_SLOTS[binding.mode], binding.channel))
        if binding.mode == "pulse_output":
            steps[binding.instance] = binding.channel
        if binding.mode == "digital_output":
            outs[binding.signal] = binding.channel
    for instance, axis in sorted(steps.items(), key=lambda item: item[1]):
        direction = outs.get(f"{instance}.dir")
        enable = outs.get(f"{instance}.enable")
        if direction is not None and enable is not None:
            channels.assign_axis(axis, core.AxisChannels(direction, enable))
    config = core.NativeRuntimeConfig()
    config.control_rate = core.Rate(tick_hz)
    config.work_dir = str(work_dir)
    config.channels = channels
    config.app_target = app_target(build)
    runtime = core.NativeControllerRuntime(config)
    image = core.FirmwareImage()
    image.path = _library(ROOT / str(firmware.path), tick_hz, app_target(build))
    image.tick_hz = tick_hz
    try:
        runtime.load(image)
    except core.ControllerConfigError as error:
        raise WiringError(f"firmware load failed: {error}") from error
    return runtime


def power_on_state(build: CoreBuild, control_rate: Fraction, work_dir: Path) -> dict[str, bool | str]:
    """The firmware's reported state before t = 0 (TST, DES-045): a private runtime reset at the controller rail's
    supply voltage and ticked once with idle inputs; its state and fault codes named by the design's tables."""
    firmware = build.design.firmware
    if firmware is None:
        return {}
    runtime = native_runtime(build, control_rate, work_dir)
    rail = core.PowerRail()
    rail.name = build.wiring.controller_rail or ""
    rail.voltage = _rail_voltage(build)
    rail.on = rail.voltage > 0.0
    runtime.reset(rail)
    outputs = core.PinOutputs()
    runtime.tick(core.PinInputs(), outputs)
    reported: dict[str, bool | str] = {}
    state = firmware.state_name(int(outputs.state_code))
    fault = firmware.fault_name(int(outputs.fault_code))
    if state is None:
        raise WiringError(f"firmware state code {outputs.state_code} has no name in firmware.state_codes")
    reported["state"] = state
    if fault is not None:
        reported["fault"] = fault
    return reported


def _rail_voltage(build: CoreBuild) -> float:
    for rail in build.wiring.rails:
        if rail.name == build.wiring.controller_rail and rail.supply is not None:
            for instance in build.wiring.instances:
                if instance.name == rail.supply:
                    return float(instance.parameters["output_voltage"])
    raise WiringError("the controller rail has no supply")


def controller_party(build: CoreBuild, control_rate: Fraction, work_dir: Path) -> Any:
    runtime = native_runtime(build, control_rate, work_dir)
    tick_hz = int(control_rate)
    party = core.ControllerPartyConfig()
    party.rate = core.Rate(tick_hz)
    party.pin_inputs = "signals.pin_inputs"
    party.rails = "signals.rails"
    if build.wiring.controller_rail is not None:
        party.rail = build.wiring.controller_rail
    return core.ControllerParty(runtime, party)


def signal_layer(
    build: CoreBuild,
    model: PhysicsModel,
    physics_rate: Fraction,
    seed: int,
    faults: Sequence[ScheduledFault],
    base_rate: Fraction,
    control_rate: Fraction | None = None,
    payloads: Sequence[str] = (),
    notes: list[str] | None = None,
) -> tuple[Any, dict[str, float], dict[str, str], int]:
    """`notes` receives one line per instance the bridge cannot carry (a brake on a transmission's driven joint: the
    translation emits no drive there, PHY-034), which is then not registered; the driver reports the lines with the
    bridge's approximations (SCO-013)."""
    places, dofs = joint_places(model)
    chip = design_chip(build)
    wiring = build.wiring
    instances = {instance.name: instance for instance in wiring.instances}
    config = core.SignalLayerConfig()
    config.rate = core.Rate(physics_rate.numerator, physics_rate.denominator)
    if control_rate is not None:
        config.controller_rate = core.Rate(control_rate.numerator, control_rate.denominator)
    config.seed = seed
    unheld: dict[str, str] = {}
    for instance in wiring.instances:
        line = undrivable_brake(instance, model)
        if line is not None:
            if notes is None:
                raise WiringError(line)
            notes.append(line)
            unheld[instance.name] = line
    table = core.PinBindingTable()
    slots: list[Any] = []
    for binding in wiring.bindings:
        if binding.instance in unheld:
            continue
        table.add(core.PinBinding(binding.signal, binding.pin, getattr(core.PeripheralMode, binding.mode)))
        slots.append(core.ChannelSlot(binding.signal, PIN_FIELDS[binding.mode], binding.channel))
    config.bindings = table
    config.slots = slots
    connections: list[Any] = []
    paired: set[tuple[str, str]] = set()
    for wire in build.design.wiring:
        source, destination = wire.source.split(".")[0], wire.destination.split(".")[0]
        if source not in instances or destination not in instances:
            continue
        coil = instances[source].type == "step_dir_driver" and instances[destination].type == "stepper_behavioral"
        if coil and (source, destination) not in paired:
            paired.add((source, destination))
            for port in DRIVER_TO_MOTOR:
                connections.append(core.Connection(f"{source}.{port}", f"{destination}.{port}"))
            continue
        if not coil and _signal_wire(instances[source], wire.source, instances[destination], wire.destination):
            connections.append(core.Connection(wire.source, wire.destination))
    config.connections = connections
    externals: dict[str, float] = {}
    for instance in wiring.instances:
        if instance.type == ESTOP_TYPE:
            externals[f"{instance.name}.estop_loop"] = 1.0
            externals[f"{instance.name}.reset"] = 0.0
    config.externals = [core.ExternalInput(name, value) for name, value in externals.items()]
    rails: list[Any] = []
    for rail in wiring.rails:
        if rail.supply is None:
            raise WiringError(f"rail '{rail.name}': supply is not set")
        consumers = [
            f"{name}.{RAIL_INPUTS[instances[name].type]}"
            for name in rail.consumers
            if instances[name].type in RAIL_INPUTS and name not in unheld
        ]
        gates = [f"{gate}.contact_closed" for gate in rail.gates]
        rails.append(core.RailSpec(rail.name, f"{rail.supply}.voltage", gates, consumers))
    config.rails = rails
    if wiring.controller_rail is not None:
        config.controller_rail = wiring.controller_rail
    actuators: list[Any] = []
    for instance in wiring.instances:
        if instance.type == "stepper_behavioral":
            actuators.append(
                core.ActuatorOutput(f"{instance.name}.torque", places[_shaft_joint(instance)].dof, applied=False)
            )
    config.actuators = actuators
    config.velocity_dofs = dofs
    layer = core.SignalLayer(config)
    by_instance: dict[str, list[ScheduledFault]] = {}
    supplies = {rail.name: rail.supply for rail in wiring.rails}
    for fault in faults:
        target = fault.target
        if fault.kind == "power_loss":
            target = supplies.get(str(fault.target))
            if target is None:
                raise WiringError(f"fault 'power_loss': rail '{fault.target}' has no supply")
        if fault.kind in LAYER_FAULTS:
            if target not in instances:
                raise WiringError(f"fault '{fault.kind}': unknown instance '{target}'")
            by_instance.setdefault(str(target), []).append(fault)
    specs, labels = _message_specs(build)
    for instance in wiring.instances:
        if instance.type == OPERATOR_LINK_TYPE:
            layer.add_operator_link(core.make_operator_link(instance.name, specs, int(instance.parameters["baud"])))
            continue
        if instance.category not in STAGES:
            continue
        if instance.name in unheld:
            continue
        component = core.make_signal_component(
            instance.type, instance.name, component_parameters(instance, places, model, payloads, chip)
        )
        for fault in by_instance.get(instance.name, []):
            component.inject(_injection(fault, base_rate))
        layer.add_component(component, TYPE_STAGES.get(instance.type, STAGES[instance.category]))
    return layer, externals, labels, dofs


def undrivable_brake(instance: InstanceWiring, model: PhysicsModel) -> str | None:
    if instance.type != "brake" or instance.acts_on is None:
        return None
    driven = {item.driven_joint: item.name for item in model.transmissions}
    if instance.acts_on not in driven:
        return None
    return (
        f"brake '{instance.name}': not applied: joint '{instance.acts_on}' is driven by transmission "
        f"'{driven[instance.acts_on]}' and the MuJoCo bridge carries no second drive there (PHY-034); the lift is "
        "unheld while its motor is off"
    )


def design_chip(build: CoreBuild) -> Chip | None:
    if build.chip is None:
        return None
    chips = shipped_chips()
    if build.chip.name not in chips:
        raise WiringError(f"chip '{build.chip.name}': not a shipped chip")
    return chips[build.chip.name]


def _port_kind(instance: InstanceWiring, port: str) -> tuple[str, str] | None:
    registry = default_registry()
    if not registry.has_type(instance.type):
        return None
    for pin in registry.component_type(instance.type).electrical.pins:
        if pin.name == port:
            return pin.direction, pin.signal_kind
    return None


def _signal_wire(source: InstanceWiring, source_port: str, destination: InstanceWiring, destination_port: str) -> bool:
    """An output port wired to an input port, neither power nor ground: a signal-layer connection (DES-041)."""
    if source.category not in STAGES or destination.category not in STAGES:
        return False
    out = _port_kind(source, source_port.split(".")[1])
    into = _port_kind(destination, destination_port.split(".")[1])
    if out is None or into is None:
        return False
    if out[1] in SIGNAL_PORT_KINDS or into[1] in SIGNAL_PORT_KINDS:
        return False
    return out[0] in ("output", "bidirectional") and into[0] in ("input", "bidirectional")


def event_axes(build: CoreBuild) -> dict[int, str]:
    """Axis index (pulse_output channel) to the instance of that step binding (FW-028 event arguments)."""
    axes: dict[int, str] = {}
    if build.chip is None:
        return axes
    for binding in build.chip.bindings:
        if binding.mode == "pulse_output":
            axes[binding.channel] = binding.instance
    return axes


def operator_message(build: CoreBuild, command: ScheduledCommand) -> Any:
    for message in build.design.operator_messages:
        if message.name == command.message:
            payload = dict(command.payload)
            return core.OperatorMessage(command.message, [float(payload[item.name]) for item in message.fields])
    raise WiringError(f"command '{command.message}': not an operator message of the design")
