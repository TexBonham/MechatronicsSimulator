#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mechsim/pins/pins.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/operator_link.hpp"
#include "mechsim/signals/safety.hpp"
#include "mechsim/signals/sensors.hpp"
#include "mechsim/signals/signal_layer.hpp"
#include "mechsim/signals/signal_name.hpp"
#include "mechsim/signals/stepper.hpp"
#include "signal_test_support.hpp"

using mechsim::pins::PinInputs;
using mechsim::pins::PinOutputs;
using mechsim::kernel::Duration;
using mechsim::kernel::Rate;
using mechsim::kernel::SimTime;
using mechsim::kernel::TickRecord;
using namespace mechsim::signals;  // NOLINT(google-build-using-namespace)
using signal_test::state_at;

namespace {

constexpr double resolution = 2.0 * std::numbers::pi / 2400.0;

struct Faults {
    std::vector<FailureInjection> encoder;
};

SignalLayerConfig layer_config(std::uint64_t seed) {
    SignalLayerConfig config;
    config.seed = seed;
    const std::vector<PinBinding> bindings{{"x_driver.step", "PA0", PeripheralMode::pulse_output},
                                           {"x_driver.dir", "PA1", PeripheralMode::digital_output},
                                           {"x_driver.enable", "PA2", PeripheralMode::digital_output},
                                           {"x_max_switch.out", "PB1", PeripheralMode::digital_input},
                                           {"x_encoder.channel_a", "PC0", PeripheralMode::counter_input},
                                           {"x_encoder.channel_b", "PC1", PeripheralMode::counter_input},
                                           {"watchdog.heartbeat", "PB6", PeripheralMode::digital_output},
                                           {"operator.uart_rx", "PA10", PeripheralMode::uart_rx},
                                           {"operator.uart_tx", "PA9", PeripheralMode::uart_tx}};
    for (const PinBinding& binding : bindings) {
        config.bindings.add(binding);
    }
    config.slots = {{"x_driver.step", PinField::step_count, 0},       {"x_driver.dir", PinField::direction, 0},
                    {"x_driver.enable", PinField::enable, 0},         {"x_max_switch.out", PinField::digital_in, 1},
                    {"x_encoder.channel_a", PinField::counter_delta, 0},
                    {"x_encoder.channel_b", PinField::counter_delta, 0},
                    {"watchdog.heartbeat", PinField::digital_out, 0}, {"operator.uart_rx", PinField::uart_rx, 0},
                    {"operator.uart_tx", PinField::uart_tx, 0}};
    config.connections = {{"x_driver.full_steps", "x_motor.full_steps"},
                          {"x_driver.direction", "x_motor.direction"},
                          {"x_driver.enabled", "x_motor.enabled"},
                          {"x_driver.supply", "x_motor.supply"}};
    config.externals = {{"estop.estop_loop", 1.0}, {"estop.reset", 0.0}};
    config.rails = {RailSpec{"motor_power", "motor_supply.voltage", {"estop.contact_closed", "watchdog.contact_closed"},
                             {"x_driver.vmot"}},
                    RailSpec{"logic", "logic_supply.voltage", {}, {}}};
    config.controller_rail = "logic";
    config.actuators = {ActuatorOutput{"x_motor.torque", 0, "N*m", false}};
    config.velocity_dofs = 1;
    return config;
}

std::vector<MessageSpec> messages() {
    return {MessageSpec{"target", "target (x, y)", {"x", "y"}, MessageDirection::to_controller},
            MessageSpec{"current_position", "current position", {"x", "y"}, MessageDirection::from_controller}};
}

// Every milestone 1 built-in in one layer (SIG-029).
std::unique_ptr<SignalLayer> make_layer(std::uint64_t seed, const Faults& faults = {},
                                        SignalLayerConfig config = layer_config(0)) {
    config.seed = seed;
    auto layer = std::make_unique<SignalLayer>(std::move(config));
    layer->add_component(std::make_unique<PowerSupply>("motor_supply", 24.0), Stage::power);
    layer->add_component(std::make_unique<PowerSupply>("logic_supply", 3.3), Stage::power);
    layer->add_component(std::make_unique<EStopRelay>("estop", EStopRelayParams{0.020, true}), Stage::power);
    layer->add_component(std::make_unique<Watchdog>("watchdog", WatchdogParams{0.050, 0.010, false}), Stage::power);
    EncoderParams encoder;
    encoder.coordinate = JointCoordinate{CoordinateKind::angular, 0, {}, {0.0, 0.0, 1.0}};
    encoder.resolution = resolution;
    auto x_encoder = std::make_unique<IncrementalEncoder>("x_encoder", encoder);
    for (const FailureInjection& fault : faults.encoder) {
        x_encoder->inject(fault);
    }
    layer->add_component(std::move(x_encoder), Stage::sensor);
    LimitSwitchParams limit;
    limit.coordinate = JointCoordinate{CoordinateKind::linear, 0, {}, {1.0, 0.0, 0.0}};
    limit.position = 0.5;
    layer->add_component(std::make_unique<LimitSwitch>("x_max_switch", limit), Stage::sensor);
    layer->add_component(std::make_unique<StepDirDriver>("x_driver", StepDirDriverParams{16, true}), Stage::driver);
    BehavioralStepperParams motor;
    motor.step_angle = std::numbers::pi / 100.0;
    motor.holding_torque = 0.40;
    motor.joint = 0;
    motor.torque_speed = {{0.0, 0.40}, {300.0 * std::numbers::pi / 30.0, 0.36}, {600.0 * std::numbers::pi / 30.0, 0.30}};
    layer->add_component(std::make_unique<BehavioralStepper>("x_motor", motor), Stage::actuator);
    layer->add_operator_link(
        std::make_unique<OperatorLink>("operator", std::make_unique<AsciiLineCodec>(messages()), UartTransport(115200)));
    return layer;
}

// The controller side: 16 pulses (one full step) per tick, forward, enabled, heartbeat toggling while beating.
PinOutputs controller_outputs(std::int64_t tick, bool beating) {
    PinOutputs out;
    out.step_count[0] = 16;
    out.direction[0] = 1;
    out.enable[0] = 1;
    out.digital_out[0] = static_cast<std::uint8_t>(beating && tick % 2 == 0);
    return out;
}

constexpr double full_step = std::numbers::pi / 100.0;

void step(SignalLayer& layer, std::int64_t tick, const PhysicalState& state, const PinOutputs& outputs) {
    layer.set_physical_state(state);
    layer.set_pin_outputs(outputs);
    layer.step(SimTime(tick), Duration(1));
}

}  // namespace

TEST_CASE("Pulses on the step pin become a shaft drive gated by the motor rail; an E-stop press cuts it",
          "[SIG-029][SIG-013][SIG-015][SIG-016][SIG-017][SIG-032][SIG-033][KER-027]") {
    // One full step per tick; the shaft follows one step behind, inside the pull-out angle, so the drive's rest is the
    // integrated command (k + 1) steps with K_hold = pi 0.40 / (2 step) = 20 N m/rad. The stepper torque is a
    // read-back, recorded and never added to the generalized forces. The press is seen at step 100; the relay's 20 ms
    // at 1 kHz opens the contacts at step 120, from when the drive is disabled.
    const std::unique_ptr<SignalLayer> layer = make_layer(1);
    layer->start(Rate(1000));
    for (std::int64_t k = 0; k < 140; ++k) {
        if (k == 100) {
            layer->set_external_input("estop.estop_loop", 0.0);
        }
        step(*layer, k, state_at(static_cast<double>(k) * full_step, 0.0, 0.1), controller_outputs(k, true));
        CHECK(layer->physics_inputs().generalized_forces[0] == 0.0);
        REQUIRE(layer->physics_inputs().joint_drives.size() == 1);
        const mechsim::kernel::JointDrive& hold = layer->physics_inputs().joint_drives[0];
        CHECK(hold.joint == 0);
        if (k < 120) {
            CHECK(hold.enabled);
            CHECK(std::fabs(hold.rest - (static_cast<double>(k + 1) * full_step)) < 1e-12);
            CHECK(std::fabs(hold.stiffness - 20.0) < 1e-12);
            CHECK(layer->rails()[0].on);
            CHECK(layer->rails()[0].voltage == 24.0);
            CHECK(layer->signal_value("x_driver.vmot") == 24.0);
        } else {
            CHECK_FALSE(hold.enabled);
            CHECK_FALSE(layer->rails()[0].on);
            CHECK(layer->rails()[0].voltage == 0.0);
        }
        if (k == 120) {
            TickRecord row;
            layer->record(row);
            REQUIRE(row.actuator_efforts.size() == 1);
            CHECK(row.actuator_efforts[0].name == "x_motor.torque");
            REQUIRE(!row.events.empty());
            CHECK(row.events[0].kind == "rail_off");
            CHECK(row.events[0].source == "motor_power");
        }
    }
    CHECK_THROWS_AS(layer->set_external_input("x_driver.vmot", 1.0), SignalConfigError);
}

TEST_CASE("The watchdog opens the motor rail when the heartbeat pin stops toggling", "[SIG-018][SIG-021]") {
    // Last heartbeat edge at step e = 99 (steps 0 .. 99 beating). Trip at e + 51 = 150, contacts open 10 later at
    // step 160 (timeout 50 ms, cutoff 10 ms, 1 kHz).
    const std::unique_ptr<SignalLayer> layer = make_layer(1);
    layer->start(Rate(1000));
    for (std::int64_t k = 0; k < 170; ++k) {
        step(*layer, k, state_at(static_cast<double>(k) * full_step, 0.0, 0.1), controller_outputs(k, k < 100));
        CHECK(layer->rails()[0].on == (k < 160));
    }
}

TEST_CASE("The controller rail voltage is published for the runtime's brownout check", "[SIG-022][SIG-015]") {
    const std::unique_ptr<SignalLayer> layer = make_layer(1);
    layer->start(Rate(1000));
    step(*layer, 0, state_at(0.0, 0.0), controller_outputs(0, true));
    CHECK(layer->controller_rail().has_value());
    const PowerRail rail = layer->controller_rail().value_or(PowerRail{"none", 0.0, false});
    CHECK(rail.name == "logic");
    CHECK(rail.voltage == 3.3);
    CHECK(rail.on);
}

TEST_CASE("Only bound slot signals reach pin inputs; an E-stop press shows only if bound", "[SIG-024][SIG-021]") {
    const std::unique_ptr<SignalLayer> layer = make_layer(1);
    layer->start(Rate(1000));
    // Encoder from 0.5 counts to 10.5 counts: 10 counts on counter channel 0 (A + B summed). Switch actuated
    // (x = 0.75 >= 0.5) on digital_in[1]. Unbound outputs (x_motor.stalled, estop.status) never appear.
    step(*layer, 0, state_at(0.5 * resolution, 0.0), controller_outputs(0, true));
    PinOutputs unbound = controller_outputs(1, true);
    unbound.step_count[3] = 1000;
    unbound.digital_out[7] = 1;
    layer->set_external_input("estop.estop_loop", 0.0);
    step(*layer, 1, state_at(10.5 * resolution, 0.75, 0.1), unbound);
    PinInputs expected;
    expected.counter_delta[0] = 10;
    expected.digital_in[1] = 1;
    const PinInputs& in = layer->pin_inputs();
    CHECK(in.digital_in == expected.digital_in);
    CHECK(in.counter_delta == expected.counter_delta);
    CHECK(in.adc_sample == expected.adc_sample);
    CHECK(in.uart_rx_count == 0);
    CHECK(layer->physics_inputs().generalized_forces.size() == 1);
}

TEST_CASE("The layer rejects wiring that bypasses or misuses the pin-binding table", "[SIG-024]") {
    const auto rejects = [](SignalLayerConfig config) {
        const std::unique_ptr<SignalLayer> layer = make_layer(1, {}, std::move(config));
        CHECK_THROWS_AS(layer->start(Rate(1000)), SignalConfigError);
    };
    SignalLayerConfig unbound = layer_config(0);
    unbound.slots.push_back({"x_motor.stalled", PinField::digital_in, 2});
    rejects(unbound);
    SignalLayerConfig wrong_mode = layer_config(0);
    wrong_mode.slots[3].field = PinField::counter_delta;
    rejects(wrong_mode);
    SignalLayerConfig outside = layer_config(0);
    outside.slots[3].channel = 32;
    rejects(outside);
    SignalLayerConfig reused = layer_config(0);
    reused.slots[6] = {"watchdog.heartbeat", PinField::enable, 0};
    rejects(reused);
    SignalLayerConfig two_sources = layer_config(0);
    two_sources.externals.push_back({"x_driver.vmot", 0.0});
    rejects(two_sources);
    SignalLayerConfig no_source = layer_config(0);
    no_source.externals.pop_back();
    rejects(no_source);
    SignalLayerConfig no_rail = layer_config(0);
    no_rail.controller_rail = "aux";
    rejects(no_rail);
    SignalLayerConfig bad_dof = layer_config(0);
    bad_dof.actuators[0].dof = 1;
    rejects(bad_dof);

    const std::unique_ptr<SignalLayer> layer = make_layer(1);
    CHECK_THROWS_AS(layer->add_component(std::make_unique<PowerSupply>("motor_supply", 1.0), Stage::power),
                    SignalNameError);
    CHECK_THROWS_AS(layer->add_component(nullptr, Stage::power), std::invalid_argument);
}

TEST_CASE("Operator commands reach uart_rx and status lines come back as labelled messages", "[SIG-026][SIG-028]") {
    const std::unique_ptr<SignalLayer> layer = make_layer(1);
    layer->start(Rate(1000));
    layer->post_operator_message(OperatorMessage{"target", {0.5, 0.25}});
    std::string received;
    for (std::int64_t k = 0; k < 3; ++k) {
        PinOutputs out = controller_outputs(k, true);
        if (k == 1) {
            const std::string line = "current_position 0.1 0.2\n";
            std::copy(line.begin(), line.end(), out.uart_tx.begin());
            out.uart_tx_count = static_cast<std::uint16_t>(line.size());
        }
        step(*layer, k, state_at(0.0, 0.0), out);
        const PinInputs& in = layer->pin_inputs();
        received.append(in.uart_rx.begin(), in.uart_rx.begin() + in.uart_rx_count);
        TickRecord row;
        layer->record(row);
        if (k == 0) {
            REQUIRE(row.events.size() == 1);
            CHECK(row.events[0].kind == "operator_command");
            CHECK(row.events[0].detail == "target (x, y) x=0.5 y=0.25");
        }
        if (k == 1) {
            REQUIRE(row.events.size() == 1);
            CHECK(row.events[0].kind == "operator_status");
            CHECK(row.events[0].detail == "current position x=0.1 y=0.2");
        }
    }
    CHECK(received == "target 0.5 0.25\n");
    const std::vector<StatusMessage> status = layer->take_operator_status();
    REQUIRE(status.size() == 1);
    CHECK(status[0].message.fields == std::vector<double>{0.1, 0.2});

    PinOutputs flood = controller_outputs(3, true);
    flood.uart_tx_count = 65;
    layer->set_pin_outputs(flood);
    CHECK_THROWS_AS(layer->step(SimTime(3), Duration(1)), SignalRunError);
}

TEST_CASE("Every bound pin signal, rail and actuator effort is recorded each tick", "[SIG-028]") {
    const std::unique_ptr<SignalLayer> layer = make_layer(1);
    layer->start(Rate(1000));
    for (std::int64_t k = 0; k < 3; ++k) {
        PhysicalState state = state_at(static_cast<double>(k) * full_step, 0.0, 0.1);
        state.joint_drive_torques.push_back(mechsim::kernel::JointDriveTorque{0, 0.25});
        step(*layer, k, state, controller_outputs(k, true));
        TickRecord row;
        layer->record(row);
        REQUIRE(row.pin_signals.size() == 9);
        CHECK(row.pin_signals[0].name == "x_driver.step");
        CHECK(row.pin_signals[0].unit == "count");
        CHECK(row.pin_signals[0].value == 16.0);
        CHECK(row.pin_signals[6].name == "watchdog.heartbeat");
        CHECK(row.pin_signals[6].value == static_cast<double>(k % 2 == 0));
        REQUIRE(row.rails.size() == 4);
        CHECK(row.rails[0].name == "motor_power.voltage");
        CHECK(row.rails[0].value == 24.0);
        CHECK(row.rails[3].name == "logic.on");
        REQUIRE(row.actuator_efforts.size() == 1);
        CHECK(row.actuator_efforts[0].value == 0.25);
    }
}

TEST_CASE("Two runs with one seed give identical layer outputs; noise follows the child seed", "[SIG-005]") {
    // A noisy encoder (sigma 3 counts) over 200 ticks of rotation: the same run seed must reproduce every pin input
    // bit for bit (invariant 4); another seed changes the child generator's draws.
    const Faults faults{{FailureInjection{FailureMode::noisy, SimTime(10), SimTime(190), 3.0, 0.0}}};
    const auto run = [&faults](std::uint64_t seed) {
        const std::unique_ptr<SignalLayer> layer = make_layer(seed, faults);
        layer->start(Rate(1000));
        std::vector<std::int32_t> counts;
        for (std::int64_t k = 0; k < 200; ++k) {
            step(*layer, k, state_at(0.01 * static_cast<double>(k), 0.0, 0.1), controller_outputs(k, true));
            counts.push_back(layer->pin_inputs().counter_delta[0]);
        }
        return counts;
    };
    CHECK(run(11) == run(11));
    CHECK(run(11) != run(12));
    CHECK(derive_child_seed(11, "x_encoder") != derive_child_seed(11, "y_encoder"));
    CHECK(derive_child_seed(11, "x_encoder") == derive_child_seed(11, "x_encoder"));
}

namespace {

struct Crossed {
    std::int64_t counts = 0;
    double full_steps = 0.0;
    std::string rx;
};

// The layer at `ratio` steps per controller step; the controller steps after the layer whenever the step index is a
// multiple of `ratio`, takes the pin inputs and sends one full step (16 pulses) for the layer to read next.
Crossed cross(std::int64_t ratio, std::int64_t controller_steps) {
    SignalLayerConfig config = layer_config(0);
    config.rate = Rate(1000 * ratio);
    config.controller_rate = Rate(1000);
    const std::unique_ptr<SignalLayer> layer = make_layer(1, {}, std::move(config));
    layer->start(Rate(1000 * ratio));
    layer->post_operator_message(OperatorMessage{"target", {0.5, 0.25}});
    Crossed crossed;
    PinOutputs outputs;
    const double per_step = 2.5 * resolution * 4.0 / static_cast<double>(ratio);
    for (std::int64_t k = 0; k <= controller_steps * ratio; ++k) {
        step(*layer, k, state_at(static_cast<double>(k) * per_step, 0.0, 0.1), outputs);
        crossed.full_steps += layer->signal_value("x_driver.full_steps");
        outputs = PinOutputs{};
        if (k % ratio == 0) {
            const PinInputs& in = layer->pin_inputs();
            crossed.counts += in.counter_delta[0];
            crossed.rx.append(in.uart_rx.begin(), in.uart_rx.begin() + in.uart_rx_count);
            outputs = controller_outputs(k / ratio, true);
        }
    }
    return crossed;
}

}  // namespace

TEST_CASE("At a 4 kHz layer and a 1 kHz controller every counter delta, UART byte and pulse crosses once", "[SIG-038]") {
    // The shaft turns 10 counts per controller step either way, 200 steps: the encoder's first sample is the
    // reference, so the deltas sum to 2000 counts at both rates. The line "target 0.5 0.25\n" leaves at 115200 baud
    // (11.5 bytes per ms) and arrives whole, in order, once. One full step per controller step is read once: 200.
    const Crossed equal = cross(1, 200);
    const Crossed fast = cross(4, 200);
    CHECK(equal.counts == 2000);
    CHECK(fast.counts == equal.counts);
    CHECK(equal.rx == "target 0.5 0.25\n");
    CHECK(fast.rx == equal.rx);
    CHECK(equal.full_steps == 200.0);
    CHECK(fast.full_steps == equal.full_steps);
}
