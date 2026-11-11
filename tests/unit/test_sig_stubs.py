"""SIG stubs, unit tier (T3); Catch2 tests replace them with the C++ code. Converted here: the fidelity parameters
through the binding (SIG-002, SIG-003, SIG-004), the milestone 2 built-ins (SIG-014, SIG-020), the linear encoder key (SIG-011) and the rail through
the kernel (SIG-018, SIG-019, SIG-022)."""

import math
from typing import Any

import pytest

from mechsim import _core as core

FULL_SCALE = 2e-3 * 5.0  # V at capacity, load_cell_5kg_generic
CAPACITY = 49.05


def run_component(component: Any, states: list[Any], inputs: list[list[float]] | None = None) -> list[list[float]]:
    """Steps a built-in through the C++ interface on a 1 kHz layer clock, one state per layer step."""
    component.start(core.ComponentClock(core.Rate(1000), core.Duration(1), 7))
    out: list[list[float]] = []
    for k, state in enumerate(states):
        values: list[float] = []
        if inputs is not None:
            values = inputs[k]
        component.begin_tick(core.SimTime(k))
        out.append(list(core.signal_component_tick(component, state, values)))
    return out


def joint_force(force: float) -> Any:
    state = core.PhysicsOutputs()
    joint = core.JointForce()
    joint.generalized_force = [force]
    state.joints = [joint]
    state.joint_positions = [0.0]
    return state


def load_cell(**fidelity: float) -> Any:
    params: dict[str, Any] = {
        "joint": 0,
        "capacity": CAPACITY,
        "sensitivity": 2e-3,
        "excitation_voltage": 5.0,
        "overload_limit": 73.575,
    }
    params.update(fidelity)
    return core.make_signal_component("load_cell", "cell", params)


@pytest.mark.req("SIG-002")
@pytest.mark.req("LIB-025")
def test_noise_parameter_reaches_the_sensor_through_the_binding() -> None:
    # N = 4000 draws of N(0, 0.1 N) on a 10 N reading, converted at FULL_SCALE / CAPACITY V/N. The sample mean is
    # N(0, sigma^2 / N): |mean| < 4 sigma / sqrt(N) fails with probability 6e-5; (N - 1) s^2 / sigma^2 is chi^2(N - 1),
    # so |s^2 / sigma^2 - 1| < 4 sqrt(2 / (N - 1)) fails with about the same. noise 0 draws nothing: exact.
    n = 4000
    sigma = 0.1
    out = run_component(load_cell(noise=sigma), [joint_force(10.0)] * n)
    forces = [row[0] * CAPACITY / FULL_SCALE - 10.0 for row in out]
    mean = sum(forces) / n
    variance = sum((f - mean) ** 2 for f in forces) / (n - 1)
    assert abs(mean) < 4.0 * sigma / math.sqrt(n)
    assert abs(variance / sigma**2 - 1.0) < 4.0 * math.sqrt(2.0 / (n - 1))
    exact = run_component(load_cell(), [joint_force(10.0)] * 3)
    assert {row[0] for row in exact} == {FULL_SCALE * 10.0 / CAPACITY}


@pytest.mark.req("SIG-003")
@pytest.mark.req("LIB-025")
def test_delay_parameter_reaches_the_sensor_through_the_binding() -> None:
    # delay 5 ms on a 1 kHz layer: ceil(5) = 5 periods; the output at step k is the reading of step k - 5, before
    # that the first reading (force 0).
    out = run_component(load_cell(delay=0.005), [joint_force(float(k)) for k in range(20)])
    for k, row in enumerate(out):
        assert row[0] == FULL_SCALE * (max(k - 5, 0) / CAPACITY)


@pytest.mark.req("SIG-004")
@pytest.mark.req("LIB-025")
def test_sample_rate_parameter_reaches_the_sensor_through_the_binding() -> None:
    # 200 Hz on a 1 kHz layer: a sample every 5 periods at steps 0, 5, 10, ..., held between. A rate above the
    # layer's is refused at start.
    out = run_component(load_cell(sample_rate=200.0), [joint_force(float(k)) for k in range(20)])
    for k, row in enumerate(out):
        assert row[0] == FULL_SCALE * ((k // 5 * 5) / CAPACITY)
    switch = core.make_signal_component(
        "limit_switch",
        "limit",
        {"coordinate": core.JointCoordinate(core.CoordinateKind.linear, 0), "sample_rate": 2000.0},
    )
    with pytest.raises(ValueError, match="sample_rate"):
        switch.start(core.ComponentClock(core.Rate(1000), core.Duration(1), 7))
    with pytest.raises(ValueError, match="no parameter 'jitter'"):
        load_cell(jitter=1.0)


@pytest.mark.req("SIG-014")
def test_load_cell_amplifier_turns_the_bridge_output_into_an_adc_count() -> None:
    # 2 kg payload, F = 19.62 N: V = 2e-3 x 5 x 19.62 / 49.05 = 4.000e-3 V; x 128 = 0.512 V; with the chip's ADC
    # (12 bits at 3.3 V): round(0.512 / 3.3 x 4095) = round(635.35) = 635.
    volts = run_component(load_cell(), [joint_force(19.62)])[0][0]
    amp = core.make_signal_component(
        "load_cell_amplifier",
        "amp",
        {"gain": 128.0, "output_full_scale": 3.3, "bandwidth": 10.0, "adc_bits": 12, "adc_reference": 3.3},
    )
    assert run_component(amp, [core.PhysicsOutputs()], [[volts]]) == [[635.0]]


@pytest.mark.req("SIG-014")
def test_load_cell_on_a_fixed_joint_reads_the_reaction_through_the_binding() -> None:
    # Ruling 24: a payload attached at t = 0 is a fixed child of the hook; `joint_reaction` reads the magnitude of that
    # joint's parent-on-child reaction force. A 2 kg child at rest is held by 2 x 9.81 = 19.62 N, so the bridge output
    # is 2e-3 x 5 x 19.62 / 49.05 = 4.000e-3 V, the same reading as the weld-force and joint modes.
    params: dict[str, Any] = {
        "joint_reaction": 0,
        "capacity": CAPACITY,
        "sensitivity": 2e-3,
        "excitation_voltage": 5.0,
        "overload_limit": 73.575,
    }
    cell = core.make_signal_component("load_cell", "cell", params)
    state = core.PhysicsOutputs()
    joint = core.JointForce()
    wrench = core.Wrench()
    wrench.force = [0.0, 0.0, 2.0 * 9.81]
    joint.reaction = wrench
    state.joints = [joint]
    volts = run_component(cell, [state])[0][0]
    assert math.isclose(volts, FULL_SCALE * 19.62 / CAPACITY, rel_tol=1e-12)


@pytest.mark.req("SIG-014")
@pytest.mark.req("LIB-025")
def test_load_cell_with_no_target_reads_zero_through_the_binding() -> None:
    # BLD-036 amendment: the key set allows none of payload, joint and joint_reaction; F = 0 gives V = 0 exactly,
    # whatever force the state carries (the 10 N joint force here is not sensed).
    params: dict[str, Any] = {
        "capacity": CAPACITY,
        "sensitivity": 2e-3,
        "excitation_voltage": 5.0,
        "overload_limit": 73.575,
        "resolution": 0.01,
    }
    cell = core.make_signal_component("load_cell", "cell", params)
    assert run_component(cell, [joint_force(10.0)] * 3) == [[0.0], [0.0], [0.0]]


def carriage_at(x: float) -> Any:
    state = core.PhysicsOutputs()
    body = core.BodyState()
    body.position = [x, 0.0, 0.0]
    state.bodies = [body]
    return state


@pytest.mark.req("SIG-011")
@pytest.mark.req("LIB-025")
def test_linear_key_reaches_the_encoder_through_the_binding() -> None:
    # Ruling 32: `linear: true` with a prismatic coordinate, resolution 1 / 60000 m per count. From half a count in,
    # 1 mm of travel is floor(0.5 + 60) - floor(0.5) = 60 counts, 30 A and 30 B edges (from an even count position).
    # Without the key the same coordinate is the mixed case, a configuration error naming the instance.
    resolution = 1.0 / 60000.0
    params: dict[str, Any] = {
        "coordinate": core.JointCoordinate(core.CoordinateKind.linear, 0),
        "resolution": resolution,
        "linear": True,
    }
    encoder = core.make_signal_component("incremental_encoder", "y_encoder", params)
    out = run_component(encoder, [carriage_at(0.5 * resolution), carriage_at(0.5 * resolution + 1e-3)])
    assert out[1] == [30.0, 30.0]
    del params["linear"]
    with pytest.raises(ValueError, match="y_encoder"):
        core.make_signal_component("incremental_encoder", "y_encoder", params)


@pytest.mark.req("SIG-020")
@pytest.mark.req("LIB-027")
def test_brake_publishes_a_joint_drive_while_engaged() -> None:
    # release low (power-off brake): engaged at once on the first sample, a drive at the joint's position with
    # K_b = 0.5 / 0.01 = 50 N m/rad; release high with the rail on: released after floor(30 ms / 1 ms) = 30 periods.
    brake = core.make_signal_component(
        "brake", "brake", {"joint": 0, "brake_torque": 0.5, "engage_delay": 0.02, "release_delay": 0.03}
    )
    state = core.PhysicsOutputs()
    state.joint_positions = [0.125]
    inputs = [[0.0, 24.0]] + [[1.0, 24.0]] * 31
    out = run_component(brake, [state] * 32, inputs)
    assert [row[0] for row in out] == [1.0] * 31 + [0.0]
    assert brake.joint_drives() == []
    engaged = core.make_signal_component("brake", "brake", {"joint": 0, "brake_torque": 0.5})
    run_component(engaged, [state], [[0.0, 24.0]])
    drives = engaged.joint_drives()
    assert len(drives) == 1
    assert (drives[0].joint, drives[0].rest, drives[0].stiffness, drives[0].enabled) == (0, 0.125, 50.0, True)


def run_rails(layer: Any, ticks: int) -> list[dict[str, float]]:
    """Runs the layer with a null physics adapter and the null controller runtime on the logic rail; the rails the
    layer published each tick, as name -> voltage."""
    kernel = core.make_kernel(core.RunConfig())
    adapter = core.NullPhysicsAdapter()
    adapter.load(core.PreparedModel(), core.Rate(1000))
    kernel.register_party(core.PhysicsParty(adapter, core.PhysicsPartyConfig()))
    kernel.register_party(layer)
    config = core.ControllerPartyConfig()
    config.rail = "logic"
    runtime = core.make_controller_runtime("null", core.Rate(1000))
    firmware = core.FirmwareImage()
    firmware.path = "firmware/app"
    firmware.tick_hz = 1000
    runtime.load(firmware)
    kernel.register_party(core.ControllerParty(runtime, config))
    sink = core.InMemoryRecorderSink()
    report = kernel.run(core.EndCondition(core.Seconds(ticks, 1000)), sink)
    assert report.outcome == core.RunOutcome.end_condition_met
    return [{rail.name: rail.voltage for rail in row.to_dict()["coupling"]["signals.rails"]} for row in sink.drain()]


@pytest.mark.req("SIG-019")
@pytest.mark.req("SIG-022")
def test_power_loss_drops_every_rail_it_feeds_for_the_window_and_the_controller_receives_it() -> None:
    # power_loss on the logic supply for ticks [20, 30): both rails it feeds, the logic rail (published to the
    # controller party, SIG-022) and a sensor rail, read 0 V for exactly those ticks and 3.3 V before and after; a
    # rail on another supply is untouched. Consumers reading 0 V is covered by the Catch2 layer test.
    config = core.SignalLayerConfig()
    config.rails = [
        core.RailSpec("logic", "supply.voltage"),
        core.RailSpec("sensor_power", "supply.voltage"),
        core.RailSpec("motor_power", "motor_supply.voltage"),
    ]
    config.controller_rail = "logic"
    layer = core.SignalLayer(config)
    supply = core.make_signal_component("power_supply", "supply", {"voltage": 3.3})
    loss = core.FailureInjection()
    loss.mode = core.FailureMode.power_loss
    loss.start = core.SimTime(20)
    loss.end = core.SimTime(30)
    supply.inject(loss)
    layer.add_component(supply, core.Stage.power)
    layer.add_component(core.make_signal_component("power_supply", "motor_supply", {"voltage": 24.0}), core.Stage.power)
    rails = run_rails(layer, 40)
    assert len(rails) == 40
    for rail in ("logic", "sensor_power"):
        assert [k for k, published in enumerate(rails) if published[rail] == 0.0] == list(range(20, 30))
        assert {published[rail] for published in rails} == {0.0, 3.3}
    assert {published["motor_power"] for published in rails} == {24.0}


@pytest.mark.req("SIG-018")
def test_watchdog_opens_the_motor_rail_after_the_heartbeat_timeout_without_the_controller() -> None:
    # The null runtime never toggles the heartbeat pin: the first sample counts as an edge (step 0), so with a 50 ms
    # timeout on a 1 kHz layer the watchdog trips at the first step j with j - 0 > 50, j = 51, and with no cutoff
    # delay the motor rail is off from step 51. The controller takes no action.
    config = core.SignalLayerConfig()
    table = core.PinBindingTable()
    table.add(core.PinBinding("watchdog.heartbeat", "PB6", core.PeripheralMode.digital_output))
    config.bindings = table
    config.slots = [core.ChannelSlot("watchdog.heartbeat", core.PinField.digital_out, 0)]
    config.rails = [
        core.RailSpec("motor_power", "motor_supply.voltage", ["watchdog.contact_closed"]),
        core.RailSpec("logic", "logic_supply.voltage"),
    ]
    config.controller_rail = "logic"
    layer = core.SignalLayer(config)
    layer.add_component(core.make_signal_component("power_supply", "motor_supply", {"voltage": 24.0}), core.Stage.power)
    layer.add_component(core.make_signal_component("power_supply", "logic_supply", {"voltage": 3.3}), core.Stage.power)
    watchdog = core.make_signal_component("watchdog", "watchdog", {"timeout": 0.05, "cutoff_delay": 0.0})
    layer.add_component(watchdog, core.Stage.power)
    rails = run_rails(layer, 80)
    assert [k for k, published in enumerate(rails) if published["motor_power"] == 0.0] == list(range(51, 80))
    assert {published["logic"] for published in rails} == {3.3}


@pytest.mark.req("SIG-035")
@pytest.mark.xfail(strict=True, reason="stub")
def test_sig_035() -> None:
    """The stepper component shall reserve a phase-current fidelity tier slot, which v1 accepts in data and does not
    evaluate.
    """
    raise NotImplementedError
