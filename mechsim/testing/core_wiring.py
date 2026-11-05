"""Signal layer, operator link and native controller assembled from a build (TST-012, TST-013, TST-036, KER-023).

`assemble` turns the builder's wiring table and the neutral model into the core parties: the `SignalLayer` (slots from
the pin table, driver-to-motor connections from the coil wires, rails with their gates and consumers, each stepper's
joint drive on its shaft joint with its torque recorded, sensors on their sensed joints, BLD-035), the operator link
from the design's messages, the `ControllerParty` over the native runtime, and the runner party that delivers scheduled
operator commands and E-stop presses at its first step at or after their tick (KER-023). Sensor and actuator faults are
injected into their component before t = 0 with their tick window, so the component applies them on its first step
inside it.
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
from mechsim.physics.neutral import DOF_COUNTS, Body, PhysicsModel
from mechsim.testing.driver import ScheduledCommand, ScheduledFault
from mechsim.testing.runner import RunnerError
from mechsim.testing.schema import ROOT

STAGES: dict[str, Any] = {
    "power": core.Stage.power,
    "safety": core.Stage.power,
    "sensor": core.Stage.sensor,
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
DRIVER_TO_MOTOR = ("full_steps", "direction", "enabled", "supply")
RAIL_INPUTS = {"step_dir_driver": "vmot"}
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
        return []

    def start(self, base_rate: Any) -> None:
        self._state.posted = 0

    def step(self, t: Any, dt: Any) -> None:
        self._state.tick = int(t.count())

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


def component_parameters(
    instance: InstanceWiring, places: Mapping[str, JointPlace], model: PhysicsModel
) -> dict[str, Any]:
    """The `make_signal_component` parameters of one instance from its part values and placement (BLD-035)."""
    values = instance.parameters
    if instance.category == "sensor":
        senses = instance.senses
        if senses is None:
            raise WiringError(f"sensor '{instance.name}': no senses block (DES-043)")
        if senses.joint not in places:
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
    injection.mode = SENSOR_FAULTS[fault.kind]
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
) -> tuple[Any, dict[str, float], dict[str, str], int]:
    places, dofs = joint_places(model)
    wiring = build.wiring
    instances = {instance.name: instance for instance in wiring.instances}
    config = core.SignalLayerConfig()
    config.rate = core.Rate(physics_rate.numerator, physics_rate.denominator)
    if control_rate is not None:
        config.controller_rate = core.Rate(control_rate.numerator, control_rate.denominator)
    config.seed = seed
    table = core.PinBindingTable()
    slots: list[Any] = []
    for binding in wiring.bindings:
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
            if instances[name].type in RAIL_INPUTS
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
    for fault in faults:
        if fault.kind in SENSOR_FAULTS:
            if fault.target not in instances:
                raise WiringError(f"fault '{fault.kind}': unknown instance '{fault.target}'")
            by_instance.setdefault(str(fault.target), []).append(fault)
    specs, labels = _message_specs(build)
    for instance in wiring.instances:
        if instance.type == OPERATOR_LINK_TYPE:
            layer.add_operator_link(core.make_operator_link(instance.name, specs, int(instance.parameters["baud"])))
            continue
        if instance.category not in STAGES:
            continue
        component = core.make_signal_component(
            instance.type, instance.name, component_parameters(instance, places, model)
        )
        for fault in by_instance.get(instance.name, []):
            component.inject(_injection(fault, base_rate))
        layer.add_component(component, STAGES[instance.category])
    return layer, externals, labels, dofs


def operator_message(build: CoreBuild, command: ScheduledCommand) -> Any:
    for message in build.design.operator_messages:
        if message.name == command.message:
            payload = dict(command.payload)
            return core.OperatorMessage(command.message, [float(payload[item.name]) for item in message.fields])
    raise WiringError(f"command '{command.message}': not an operator message of the design")
