"""Signal wiring table and chip configuration (BLD-006, BLD-007, BLD-023, BLD-027)."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Literal

from mechsim.builder.physics import Collector
from mechsim.builder.result import (
    ChipConfig,
    Gate,
    InstanceWiring,
    MountBinding,
    OperatorLink,
    PinAssignment,
    RailWiring,
    Sensing,
    WiringTable,
)
from mechsim.design.chip import Chip
from mechsim.design.schema import ComponentInstance, Design, OpenValue, Rail, open_name
from mechsim.library.registry import Registry, RegistryError
from mechsim.library.schema import QuantityParameter, dimensions_equal

OPERATOR_LINK_TYPE = "uart_link"
HEARTBEAT_PORT = "heartbeat"
GATE_CATEGORY = "safety"


# ===== Instances ========================================================


def _instance(
    index: int, component: ComponentInstance, registry: Registry, rails: Mapping[str, list[str]], collector: Collector
) -> InstanceWiring | None:
    if not registry.has_type(component.type):
        collector.failures.append(f"component '{component.name}': unknown component type '{component.type}'")
        return None
    kind = registry.component_type(component.type)
    concrete: dict[str, str | bool] = {}
    for key, value in component.overrides.items():
        if open_name(value) is not None:
            collector.pending.append(f"components[{index}].overrides.{key}")
            continue
        concrete[key] = value
    try:
        part = registry.resolve_part(component.part, concrete)
    except RegistryError as error:
        collector.failures.append(f"component '{component.name}': {error} (BLD-027)")
        return None
    if part.type != component.type:
        collector.failures.append(
            f"component '{component.name}': part '{component.part}' is not of type '{component.type}' (BLD-027)"
        )
        return None
    mount_kinds = {mount.name: mount.kind for mount in kind.mechanical.mounts}
    mounts: list[MountBinding] = []
    for binding in component.mechanical:
        body = binding.body
        joint = binding.joint
        if isinstance(body, OpenValue):
            collector.pending.append(f"components[{index}].mechanical.{binding.mount}.body")
            body = None
        if isinstance(joint, OpenValue):
            collector.pending.append(f"components[{index}].mechanical.{binding.mount}.joint")
            joint = None
        mounts.append(
            MountBinding(mount=binding.mount, kind=mount_kinds.get(binding.mount, "unknown"), body=body, joint=joint)
        )
    torque_speed: tuple[tuple[float, float], ...] = ()
    if part.torque_speed_curve is not None:
        torque_speed = tuple((float(point.speed), float(point.torque)) for point in part.torque_speed_curve)
    return InstanceWiring(
        name=component.name,
        type=component.type,
        category=kind.category,
        part=component.part,
        parameters=dict(sorted(part.values.items())),
        emits=tuple(signal.name for signal in component.signals if signal.direction == "emits"),
        consumes=tuple(signal.name for signal in component.signals if signal.direction == "consumes"),
        mounts=tuple(mounts),
        rails=tuple(rails.get(component.name, [])),
        senses=_sensing(index, component, collector),
        acts_on=_acts_on(index, component, collector),
        torque_speed=torque_speed,
        core_type=kind.plugin.cpp,
        length_parameters=tuple(
            item.name
            for item in kind.parameters
            if isinstance(item, QuantityParameter) and dimensions_equal(item.dimension, "[length]")
        ),
    )


def _sensing(index: int, component: ComponentInstance, collector: Collector) -> Sensing | None:
    senses = component.senses
    if senses is None:
        return None
    joint = senses.joint
    body = senses.body
    position = senses.position
    if isinstance(joint, OpenValue):
        collector.pending.append(f"components[{index}].senses.joint")
        return None
    if isinstance(body, OpenValue):
        collector.pending.append(f"components[{index}].senses.body")
        return None
    if isinstance(position, OpenValue):
        collector.pending.append(f"components[{index}].senses.position")
        return None
    return Sensing(
        joint=joint,
        payload=senses.payload,
        joint_reaction=senses.joint_reaction,
        body=body,
        position=position,
        actuate_above=senses.actuate_above,
        reversed=senses.reversed is True,
    )


def _acts_on(index: int, component: ComponentInstance, collector: Collector) -> str | None:
    if component.acts_on is None:
        return None
    joint = component.acts_on.joint
    if isinstance(joint, OpenValue):
        collector.pending.append(f"components[{index}].acts_on.joint")
        return None
    return joint


# ===== Pin bindings =====================================================


def _bindings(design: Design, collector: Collector) -> list[PinAssignment]:
    """Channels count per peripheral mode in pin-binding order, from 0."""
    declared: dict[str, tuple[str, str, Literal["emits", "consumes"]]] = {}
    for component in design.components:
        for signal in component.signals:
            declared[signal.name] = (component.name, signal.pin, signal.direction)
    bound: set[str] = set()
    pins: dict[str, str] = {}
    channels: dict[str, int] = {}
    assignments: list[PinAssignment] = []
    for binding in design.pin_bindings:
        if binding.signal not in declared:
            collector.failures.append(f"pin binding: unknown signal '{binding.signal}'")
            continue
        if binding.signal in bound:
            collector.failures.append(f"signal '{binding.signal}' is bound to more than one pin")
            continue
        if binding.pin in pins:
            collector.failures.append(
                f"MCU pin '{binding.pin}' is bound twice: to '{pins[binding.pin]}' and '{binding.signal}'"
            )
            continue
        bound.add(binding.signal)
        pins[binding.pin] = binding.signal
        channel = channels.get(binding.mode, 0)
        channels[binding.mode] = channel + 1
        instance, port, direction = declared[binding.signal]
        assignments.append(
            PinAssignment(
                signal=binding.signal,
                instance=instance,
                port=port,
                direction=direction,
                pin=binding.pin,
                mode=binding.mode,
                channel=channel,
            )
        )
    for name in declared:
        if name not in bound:
            collector.failures.append(f"controller-facing signal '{name}' has no pin binding (BLD-023)")
    return assignments


# ===== Rails ============================================================


def _rail(
    rail: Rail, index: int, design: Design, registry: Registry, collector: Collector
) -> tuple[RailWiring, list[Gate]]:
    """Walk from the supply's power_out ports along wires; a safety instance passes power from its power_in to its
    power_out ports and joins the gating chain; any other power_in port reached (a safety instance without a power_out
    port, such as a brake coil, included) makes its instance a consumer."""
    gates_declared = [gate for gate in rail.gated_by if isinstance(gate, str)]
    open_gates = len(gates_declared) != len(rail.gated_by)
    if open_gates:
        collector.pending.append(f"power.rails[{index}].gated_by")
    supply = rail.supply
    if isinstance(supply, OpenValue):
        collector.pending.append(f"power.rails[{index}].supply")
        return RailWiring(
            name=rail.name, supply=None, gates=tuple(gates_declared), consumers=tuple(rail.consumers), derived=None
        ), []
    components = {component.name: component for component in design.components}
    directions: dict[str, dict[str, str]] = {}
    categories: dict[str, str] = {}
    for component in design.components:
        if registry.has_type(component.type):
            kind = registry.component_type(component.type)
            directions[component.name] = {pin.name: pin.direction for pin in kind.electrical.pins}
            categories[component.name] = kind.category
    if supply not in components:
        collector.failures.append(f"rail '{rail.name}': unknown supply '{supply}'")
        return RailWiring(name=rail.name, supply=supply, gates=(), consumers=tuple(rail.consumers), derived=None), []
    chain: list[str] = []
    derived: list[str] = []
    frontier = [supply]
    visited = {supply}
    while len(frontier) > 0:
        source = frontier.pop(0)
        outputs = [port for port, direction in directions.get(source, {}).items() if direction == "power_out"]
        for wire in design.wiring:
            instance, port = wire.source.split(".")
            if instance != source or port not in outputs:
                continue
            target, target_port = wire.destination.split(".")
            if directions.get(target, {}).get(target_port) != "power_in":
                continue
            passes = "power_out" in directions.get(target, {}).values()
            if categories.get(target) == GATE_CATEGORY and passes:
                if target not in visited:
                    visited.add(target)
                    chain.append(target)
                    frontier.append(target)
                continue
            if target not in derived:
                derived.append(target)
    if not open_gates and chain != gates_declared:
        collector.failures.append(
            f"rail '{rail.name}': the wiring gates it through {chain}, gated_by declares {gates_declared}"
        )
    missing = [name for name in rail.consumers if name not in derived]
    extra = [name for name in derived if name not in rail.consumers]
    if len(missing) > 0 or len(extra) > 0:
        collector.failures.append(
            f"rail '{rail.name}': declared consumers do not match the wiring; declared but not wired {missing}, "
            f"wired but not declared {extra}"
        )
    gates: list[Gate] = []
    for position, name in enumerate(chain):
        heartbeat: str | None = None
        for signal in components[name].signals:
            if signal.pin == HEARTBEAT_PORT:
                heartbeat = signal.name
        gates.append(
            Gate(rail=rail.name, instance=name, type=components[name].type, position=position, heartbeat=heartbeat)
        )
    return RailWiring(
        name=rail.name, supply=supply, gates=tuple(chain), consumers=tuple(rail.consumers), derived=tuple(derived)
    ), gates


# ===== Operator link and chip ===========================================


def _operator_links(
    design: Design, registry: Registry, assignments: list[PinAssignment], collector: Collector
) -> list[OperatorLink]:
    by_port = {(assignment.instance, assignment.port): assignment for assignment in assignments}
    links: list[OperatorLink] = []
    for component in design.components:
        if component.type != OPERATOR_LINK_TYPE:
            continue
        tx = by_port.get((component.name, "uart_tx"))
        rx = by_port.get((component.name, "uart_rx"))
        if tx is None or rx is None:
            collector.failures.append(f"operator link '{component.name}': needs both uart_tx and uart_rx bound")
            continue
        try:
            baud = registry.resolve_part(component.part).values["baud"]
        except RegistryError as error:
            collector.failures.append(f"operator link '{component.name}': {error}")
            continue
        links.append(
            OperatorLink(
                instance=component.name,
                baud=float(baud),
                tx_signal=tx.signal,
                tx_pin=tx.pin,
                rx_signal=rx.signal,
                rx_pin=rx.pin,
            )
        )
    return links


def macro(signal: str) -> str:
    return "MECHSIM_CH_" + signal.replace(".", "_").upper()


def channels_header(design: Design, assignments: list[PinAssignment]) -> str:
    lines = [
        f"/* Generated by the mechsim model builder from design {design.name} {design.version}; do not edit. */",
        "/* Board channel of each bound signal: its index within its peripheral mode, in pin-binding order. */",
        "#ifndef MECHSIM_CHANNELS_H",
        "#define MECHSIM_CHANNELS_H",
        "",
    ]
    for assignment in assignments:
        lines.append(
            f"#define {macro(assignment.signal)} {assignment.channel} /* {assignment.mode} {assignment.pin} */"
        )
    lines += ["", "#endif /* MECHSIM_CHANNELS_H */", ""]
    return "\n".join(lines)


def _chip(
    design: Design, chips: Mapping[str, Chip], assignments: list[PinAssignment], collector: Collector
) -> ChipConfig | None:
    if design.firmware is None:
        return None
    name = design.firmware.chip
    if isinstance(name, OpenValue):
        collector.pending.append("firmware.chip")
        return None
    chip = chips.get(name)
    if chip is None:
        collector.failures.append(f"chip '{name}': no chip definition")
        return None
    modes = {pin.name: list(pin.modes) for pin in chip.pins}
    for assignment in assignments:
        if assignment.pin not in modes:
            collector.failures.append(f"chip '{name}' has no pin '{assignment.pin}' (bound to '{assignment.signal}')")
        elif assignment.mode not in modes[assignment.pin]:
            collector.failures.append(
                f"chip '{name}' pin '{assignment.pin}' lacks mode '{assignment.mode}' (bound to '{assignment.signal}')"
            )
    seen: dict[str, str] = {}
    for assignment in assignments:
        key = macro(assignment.signal)
        if key in seen:
            collector.failures.append(f"signals '{seen[key]}' and '{assignment.signal}' share the macro {key}")
        seen[key] = assignment.signal
    return ChipConfig(
        name=name,
        placeholder=chip.status == "placeholder",
        bindings=tuple(assignments),
        channels_header=channels_header(design, assignments),
    )


# ===== Wiring table =====================================================


def build_wiring(
    design: Design, registry: Registry, chips: Mapping[str, Chip], collector: Collector
) -> tuple[WiringTable, ChipConfig | None]:
    rails: list[RailWiring] = []
    gating: list[Gate] = []
    membership: dict[str, list[str]] = {}
    controller_rail: str | None = None
    if design.power is not None:
        controller_rail = design.power.controller_rail
        for index, rail in enumerate(design.power.rails):
            wiring, gates = _rail(rail, index, design, registry, collector)
            rails.append(wiring)
            gating += gates
            members = [wiring.supply, *wiring.gates, *wiring.consumers]
            for member in members:
                if member is not None and rail.name not in membership.setdefault(member, []):
                    membership[member].append(rail.name)
    instances: list[InstanceWiring] = []
    for index, component in enumerate(design.components):
        entry = _instance(index, component, registry, membership, collector)
        if entry is not None:
            instances.append(entry)
    assignments = _bindings(design, collector)
    if len(assignments) > 0 and design.firmware is None:
        collector.failures.append("pin bindings need a firmware section naming the chip")
    links = _operator_links(design, registry, assignments, collector)
    chip = _chip(design, chips, assignments, collector)
    table = WiringTable(
        instances=tuple(instances),
        bindings=tuple(assignments),
        rails=tuple(rails),
        controller_rail=controller_rail,
        gating=tuple(gating),
        operator_links=tuple(links),
    )
    return table, chip
