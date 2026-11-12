"""Builder signal wiring, rails, gating chain, operator link and chip configuration (BLD-006, BLD-007, BLD-023)."""

from __future__ import annotations

from typing import Any

import pytest

from mechsim.builder import BuildError, BuildResult, build
from mechsim.builder.result import ChipConfig
from mechsim.design.chip import Chip, ChipPin, shipped_chips
from mechsim.design.schema import Design, PinBinding, Wire, shipped_materials
from mechsim.library.registry import default_registry
from tests.unit.test_builder_support import filled_gantry, gantry, gantry_data, pendulum


def _build(design: Design, **options: Any) -> BuildResult:
    options.setdefault("rigid_members", True)
    return build(design, default_registry(), shipped_materials(), **options)


def _failures(design: Design, **options: Any) -> list[str]:
    options.setdefault("rigid_members", True)
    with pytest.raises(BuildError) as raised:
        _build(design, **options)
    return raised.value.failures


def _chip(design: Design) -> ChipConfig:
    chip = _build(design).chip
    assert chip is not None
    return chip


def _placeholder_chip(drop_pin: str | None = None, drop_mode: tuple[str, str] | None = None) -> dict[str, Chip]:
    chip = shipped_chips()["stm32_placeholder"].model_copy(deep=True)
    pins: list[ChipPin] = []
    for pin in chip.pins:
        if pin.name == drop_pin:
            continue
        if drop_mode is not None and pin.name == drop_mode[0]:
            pin.modes = [mode for mode in pin.modes if mode != drop_mode[1]]
        pins.append(pin)
    chip.pins = pins
    return {chip.name: chip}


# ===== Instances ========================================================


@pytest.mark.req("BLD-006")
def test_gantry_wiring_lists_every_instance_with_its_signals() -> None:
    design = gantry()
    wiring = _build(design).wiring
    assert [entry.name for entry in wiring.instances] == [component.name for component in design.components]
    entries = {entry.name: entry for entry in wiring.instances}
    assert entries["x_driver"].consumes == ("x_driver.step", "x_driver.dir", "x_driver.enable")
    assert entries["x_driver"].emits == ()
    assert entries["x_encoder"].emits == ("x_encoder.channel_a", "x_encoder.channel_b")
    assert entries["watchdog"].consumes == ("watchdog.heartbeat",)
    assert entries["operator_link"].emits == ("operator_link.uart_rx",)
    assert entries["operator_link"].consumes == ("operator_link.uart_tx",)
    assert entries["motor_supply"].category == "power"
    assert entries["x_belt"].parameters["ratio"] == pytest.approx(0.04 / (2.0 * 3.141592653589793), rel=1e-15)
    assert list(entries["x_belt"].parameters) == sorted(entries["x_belt"].parameters)


@pytest.mark.req("BLD-006")
def test_cross_domain_bindings_are_listed_per_instance() -> None:
    entries = {entry.name: entry for entry in _build(gantry()).wiring.instances}
    mounts = {(mount.mount, mount.kind, mount.body, mount.joint) for mount in entries["x_motor"].mounts}
    assert mounts == {("face", "face", "frame", None), ("shaft", "shaft", None, "x_motor_spin")}
    assert entries["x_encoder"].mounts[0].body is None


def test_open_mount_and_override_are_pending() -> None:
    data = gantry_data()
    data["components"][0]["overrides"] = {"mass": "${rated_load}"}
    result = _build(Design.model_validate(data))
    assert "components[0].overrides.mass" in result.pending
    assert "components[15].mechanical.flange.body" in _build(gantry()).pending


@pytest.mark.req("BLD-027")
def test_instance_parameters_combine_part_values_and_overrides() -> None:
    data = gantry_data()
    data["components"][-1]["overrides"] = {"baud": "9600 Hz"}
    wiring = _build(filled_gantry(data)).wiring
    link = next(entry for entry in wiring.instances if entry.name == "operator_link")
    assert link.parameters["baud"] == 9600.0
    assert link.parameters["mass"] == pytest.approx(0.005, rel=1e-15)


@pytest.mark.req("BLD-027")
def test_unknown_part_or_type_fails() -> None:
    design = gantry()
    design.components[0].part = "no_such_part"
    design.components[1].type = "no_such_type"
    failures = _failures(design)
    assert any("component 'x_motor'" in failure for failure in failures)
    assert any("component 'x_driver'" in failure and "no_such_type" in failure for failure in failures)


def test_part_of_another_type_fails() -> None:
    design = gantry()
    design.components[0].part = "microswitch_generic"
    failures = _failures(design)
    assert any("component 'x_motor'" in failure for failure in failures)


def test_pendulum_has_empty_wiring_and_no_chip() -> None:
    result = _build(pendulum())
    assert result.wiring.instances == ()
    assert result.wiring.bindings == ()
    assert result.wiring.rails == ()
    assert result.wiring.controller_rail is None
    assert result.chip is None


# ===== Pin bindings and channels =======================================


@pytest.mark.req("BLD-007")
def test_channels_count_per_mode_in_binding_order() -> None:
    design = gantry()
    chip = _chip(design)
    assert [entry.signal for entry in chip.bindings] == [binding.signal for binding in design.pin_bindings]
    counts: dict[str, int] = {}
    for entry in chip.bindings:
        assert entry.channel == counts.get(entry.mode, 0)
        counts[entry.mode] = entry.channel + 1
    assert counts == {
        "pulse_output": 3,
        "digital_output": 8,
        "digital_input": 7,
        "counter_input": 6,
        "uart_tx": 1,
        "uart_rx": 1,
        "adc_input": 1,
    }
    by_signal = {entry.signal: entry for entry in chip.bindings}
    assert (by_signal["x_driver.step"].pin, by_signal["x_driver.step"].channel) == ("PA0", 0)
    assert (by_signal["lift_driver.step"].pin, by_signal["lift_driver.step"].channel) == ("PA6", 2)
    assert (by_signal["watchdog.heartbeat"].mode, by_signal["watchdog.heartbeat"].channel) == ("digital_output", 6)
    assert by_signal["x_min_switch.out"].direction == "emits"
    assert by_signal["x_min_switch.out"].port == "out"


@pytest.mark.req("BLD-007")
def test_chip_configuration_names_the_chip_and_its_status() -> None:
    chip = _chip(gantry())
    assert chip.name == "stm32_placeholder"
    assert chip.placeholder is True
    verified = _placeholder_chip()
    verified["stm32_placeholder"].status = "verified"
    built = _build(gantry(), chips=verified).chip
    assert built is not None
    assert built.placeholder is False


@pytest.mark.req("BLD-007")
def test_channels_header_defines_every_bound_signal() -> None:
    chip = _chip(gantry())
    header = chip.channels_header
    assert header.startswith("/* Generated by the mechsim model builder from design gantry 0.1.0")
    assert "#ifndef MECHSIM_CHANNELS_H\n#define MECHSIM_CHANNELS_H\n" in header
    assert header.endswith("#endif /* MECHSIM_CHANNELS_H */\n")
    defines = [line for line in header.splitlines() if line.startswith("#define MECHSIM_CH_")]
    assert len(defines) == len(chip.bindings)
    assert "#define MECHSIM_CH_X_DRIVER_STEP 0 /* pulse_output PA0 */" in defines
    assert "#define MECHSIM_CH_OPERATOR_LINK_UART_TX 0 /* uart_tx PA9 */" in defines
    assert "#define MECHSIM_CH_OPERATOR_LINK_UART_RX 0 /* uart_rx PA10 */" in defines
    assert "#define MECHSIM_CH_WATCHDOG_HEARTBEAT 6 /* digital_output PB6 */" in defines


def test_wiring_and_chip_share_one_binding_table() -> None:
    result = _build(gantry())
    assert result.chip is not None
    assert result.chip.bindings == result.wiring.bindings


def test_open_chip_is_pending() -> None:
    data = gantry_data()
    data["firmware"]["chip"] = "${stm32_part}"
    result = _build(Design.model_validate(data))
    assert result.chip is None
    assert "firmware.chip" in result.pending
    assert len(result.wiring.bindings) == 27


@pytest.mark.req("BLD-023")
def test_signal_with_no_pin_fails_naming_the_signal() -> None:
    design = gantry()
    design.pin_bindings = [binding for binding in design.pin_bindings if binding.signal != "y_encoder.channel_b"]
    failures = _failures(design)
    assert any("signal 'y_encoder.channel_b' has no pin binding" in failure for failure in failures)


def test_pin_bound_twice_fails() -> None:
    design = gantry()
    design.pin_bindings[1].pin = "PA0"
    failures = _failures(design)
    assert any("MCU pin 'PA0' is bound twice" in failure for failure in failures)


def test_signal_bound_twice_fails() -> None:
    design = gantry()
    design.pin_bindings.append(PinBinding(signal="x_driver.step", pin="PC6", mode="pulse_output"))
    failures = _failures(design)
    assert any("'x_driver.step' is bound to more than one pin" in failure for failure in failures)


def test_binding_of_an_unknown_signal_fails() -> None:
    design = gantry()
    design.pin_bindings.append(PinBinding(signal="ghost.out", pin="PC6", mode="digital_input"))
    failures = _failures(design)
    assert any("unknown signal 'ghost.out'" in failure for failure in failures)


def test_bindings_without_firmware_fail() -> None:
    design = gantry()
    design.firmware = None
    failures = _failures(design)
    assert any("firmware" in failure for failure in failures)


def test_chip_lacking_a_bound_pin_fails() -> None:
    failures = _failures(gantry(), chips=_placeholder_chip(drop_pin="PA10"))
    assert any("has no pin 'PA10'" in failure for failure in failures)


def test_chip_pin_lacking_the_bound_mode_fails() -> None:
    failures = _failures(gantry(), chips=_placeholder_chip(drop_mode=("PA9", "uart_tx")))
    assert any("pin 'PA9' lacks mode 'uart_tx'" in failure for failure in failures)


def test_unknown_chip_fails() -> None:
    failures = _failures(gantry(), chips={})
    assert any("chip 'stm32_placeholder'" in failure for failure in failures)


def test_signals_sharing_a_macro_name_fail() -> None:
    """x_min.switch_out and x.min_switch_out both spell MECHSIM_CH_X_MIN_SWITCH_OUT."""
    design = gantry()
    renames = {"x_min_switch.out": "x_min.switch_out", "x_max_switch.out": "x.min_switch_out"}
    for component in design.components:
        for signal in component.signals:
            signal.name = renames.get(signal.name, signal.name)
    for binding in design.pin_bindings:
        binding.signal = renames.get(binding.signal, binding.signal)
    failures = _failures(design)
    assert any("MECHSIM_CH_X_MIN_SWITCH_OUT" in failure for failure in failures)


# ===== Rails and gating =================================================


@pytest.mark.req("BLD-006")
def test_motor_rail_is_derived_from_the_wiring() -> None:
    wiring = _build(gantry()).wiring
    rails = {rail.name: rail for rail in wiring.rails}
    motor = rails["motor_power"]
    assert motor.supply == "motor_supply"
    assert motor.gates == ("estop", "watchdog")
    assert motor.derived == ("x_driver", "y_driver", "lift_driver", "lift_brake")
    assert motor.consumers == ("x_driver", "y_driver", "lift_driver", "lift_brake")
    assert wiring.controller_rail == "logic"
    entries = {entry.name: entry for entry in wiring.instances}
    assert entries["x_driver"].rails == ("motor_power",)
    assert entries["estop"].rails == ("motor_power",)
    assert entries["motor_supply"].rails == ("motor_power",)
    assert entries["x_encoder"].rails == ("logic",)
    assert entries["x_min_switch"].rails == ()


def test_rail_with_open_supply_keeps_declared_consumers_pending() -> None:
    result = _build(gantry())
    logic = next(rail for rail in result.wiring.rails if rail.name == "logic")
    assert logic.supply is None
    assert logic.derived is None
    assert logic.consumers == ("x_encoder", "y_encoder", "lift_encoder", "load_cell", "load_amp")
    assert "power.rails[1].supply" in result.pending


@pytest.mark.req("BLD-006")
def test_gating_chain_lists_estop_then_watchdog_with_the_heartbeat() -> None:
    gating = _build(gantry()).wiring.gating
    assert [(gate.instance, gate.type, gate.position) for gate in gating] == [
        ("estop", "estop_relay", 0),
        ("watchdog", "watchdog", 1),
    ]
    assert gating[0].heartbeat is None
    assert gating[1].heartbeat == "watchdog.heartbeat"
    assert {gate.rail for gate in gating} == {"motor_power"}


def test_wired_consumer_missing_from_the_rail_fails() -> None:
    design = gantry()
    assert design.power is not None
    design.power.rails[0].consumers = ["x_driver", "y_driver"]
    failures = _failures(design)
    assert any("rail 'motor_power'" in failure and "lift_driver" in failure for failure in failures)


def test_declared_consumer_not_wired_fails() -> None:
    design = gantry()
    design.wiring = [
        wire
        for wire in design.wiring
        if not (wire.source == "watchdog.contact_out" and wire.destination == "lift_driver.vmot")
    ]
    failures = _failures(design)
    assert any("declared but not wired ['lift_driver']" in failure for failure in failures)


def test_gate_order_or_set_mismatch_fails() -> None:
    design = gantry()
    assert design.power is not None
    design.power.rails[0].gated_by = ["watchdog", "estop"]
    assert any("gates it through ['estop', 'watchdog']" in failure for failure in _failures(design))
    design.power.rails[0].gated_by = ["estop"]
    assert any("gated_by declares ['estop']" in failure for failure in _failures(design))


def test_ungated_supply_feeding_consumers_directly() -> None:
    design = gantry()
    assert design.power is not None
    design.power.rails[0].gated_by = []
    design.wiring = [wire for wire in design.wiring if wire.source != "watchdog.contact_out"]
    design.wiring = [wire for wire in design.wiring if wire.source != "motor_supply.v_out"]
    for driver in ("x_driver", "y_driver", "lift_driver"):
        design.wiring.append(Wire(source="motor_supply.v_out", destination=f"{driver}.vmot"))
    design.wiring.append(Wire(source="motor_supply.v_out", destination="lift_brake.vcc"))
    rail = next(item for item in _build(design).wiring.rails if item.name == "motor_power")
    assert rail.gates == ()
    assert rail.derived == ("x_driver", "y_driver", "lift_driver", "lift_brake")


def test_unknown_rail_supply_fails() -> None:
    design = gantry()
    assert design.power is not None
    design.power.rails[0].supply = "ghost"
    assert any("unknown supply 'ghost'" in failure for failure in _failures(design))


def test_open_gate_is_pending() -> None:
    data = gantry_data()
    data["power"]["rails"][0]["gated_by"] = ["estop", "${logic_supply}"]
    result = _build(Design.model_validate(data))
    assert "power.rails[0].gated_by" in result.pending


# ===== Operator link ====================================================


@pytest.mark.req("BLD-006")
def test_operator_link_carries_its_uart_pins() -> None:
    links = _build(gantry()).wiring.operator_links
    assert len(links) == 1
    link = links[0]
    assert (link.instance, link.baud) == ("operator_link", 115200.0)
    assert (link.tx_signal, link.tx_pin) == ("operator_link.uart_tx", "PA9")
    assert (link.rx_signal, link.rx_pin) == ("operator_link.uart_rx", "PA10")


def test_operator_link_missing_a_uart_signal_fails() -> None:
    design = gantry()
    link = design.components[-1]
    link.signals = [signal for signal in link.signals if signal.pin != "uart_rx"]
    design.pin_bindings = [binding for binding in design.pin_bindings if binding.signal != "operator_link.uart_rx"]
    assert any("operator link 'operator_link'" in failure for failure in _failures(design))
