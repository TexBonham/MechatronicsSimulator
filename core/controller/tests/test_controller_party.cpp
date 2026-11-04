#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mechsim/controller/controller_party.hpp"
#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/kernel/cosim_kernel.hpp"
#include "mechsim/kernel/kernel.hpp"
#include "mechsim/kernel/memory_recorder_sink.hpp"
#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_party.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/pins/pins.hpp"
#include "mechsim/signals/pin_binding.hpp"
#include "mechsim/signals/safety.hpp"
#include "mechsim/signals/sensors.hpp"
#include "mechsim/signals/signal_layer.hpp"
#include "mechsim/signals/stepper.hpp"

using namespace mechsim;  // NOLINT(google-build-using-namespace)

namespace {

constexpr double load_torque = 0.01;
constexpr double turn_per_step = 0.01;

struct Seen {
    std::vector<kernel::PhysicsInputs> physics_inputs;
    std::vector<pins::PinInputs> pin_inputs;
    std::vector<double> resets;
};

// A shaft at x = 1 m turning turn_per_step rad per step, carrying a constant load torque on transmission 0.
class ShaftAdapter final : public kernel::PhysicsAdapter {
public:
    explicit ShaftAdapter(std::shared_ptr<Seen> seen) : seen_(std::move(seen)) {
        outputs_.bodies.resize(1);
        outputs_.bodies[0].position = {1.0, 0.0, 0.0};
        outputs_.joints.resize(1);
        outputs_.transmissions.resize(1);
        outputs_.transmissions[0].shaft_load_torque = load_torque;
    }
    [[nodiscard]] std::span<const std::string> supported_elements() const override { return {}; }
    void load(const kernel::PreparedModel& /*model*/, const kernel::Rate& /*rate*/) override {}
    void reset() override {}
    void step() override {
        seen_->physics_inputs.push_back(pending_);
        pending_ = kernel::PhysicsInputs{};
        ++outputs_.steps;
        const double angle = turn_per_step * static_cast<double>(outputs_.steps);
        outputs_.bodies[0].orientation = {std::cos(angle / 2.0), 0.0, 0.0, std::sin(angle / 2.0)};
    }
    [[nodiscard]] const kernel::PhysicsOutputs& outputs() const override { return outputs_; }
    [[nodiscard]] kernel::PhysicsHealth health() const override { return kernel::PhysicsHealth{}; }
    void set_inputs(const kernel::PhysicsInputs& inputs) override { pending_ = inputs; }
    void set_spring_offset(std::size_t /*spring*/, double /*offset*/) override {}
    void set_spring_stiffness(std::size_t /*spring*/, double /*stiffness*/) override {}
    void disable_constraint(std::size_t /*constraint*/) override {}
    void release_load(std::size_t /*load*/) override {}
    void set_payload_mass(std::size_t /*payload*/, double /*mass*/) override {}
    void set_payload_attachment(std::size_t /*payload*/, std::size_t /*body*/) override {}

private:
    std::shared_ptr<Seen> seen_;
    kernel::PhysicsOutputs outputs_;
    kernel::PhysicsInputs pending_;
};

// Sixteen microstep pulses (one full step) forward every tick, enabled; records what it was given.
class PulsingRuntime final : public controller::ControllerRuntime {
public:
    explicit PulsingRuntime(std::shared_ptr<Seen> seen) : seen_(std::move(seen)) {}
    void load(const controller::FirmwareImage& /*firmware*/) override {}
    void reset(const pins::PowerRail& rail) override { seen_->resets.push_back(rail.voltage); }
    void tick(const pins::PinInputs& inputs, pins::PinOutputs& outputs) override {
        seen_->pin_inputs.push_back(inputs);
        outputs = pins::PinOutputs{};
        outputs.step_count[0] = 16;
        outputs.direction[0] = 1;
        outputs.enable[0] = 1;
        outputs.state_code = 3;
    }
    void freeze(bool /*frozen*/) override {}
    void brownout(double voltage) override { seen_->resets.push_back(-voltage); }

private:
    std::shared_ptr<Seen> seen_;
};

std::unique_ptr<signals::SignalLayer> chain_layer() {
    using namespace signals;  // NOLINT(google-build-using-namespace)
    SignalLayerConfig config;
    for (const PinBinding& binding : std::vector<PinBinding>{
             {"x_driver.step", "PA0", PeripheralMode::pulse_output},
             {"x_driver.dir", "PA1", PeripheralMode::digital_output},
             {"x_driver.enable", "PA2", PeripheralMode::digital_output},
             {"x_max_switch.out", "PB1", PeripheralMode::digital_input},
             {"x_encoder.channel_a", "PC0", PeripheralMode::counter_input},
             {"x_encoder.channel_b", "PC1", PeripheralMode::counter_input}}) {
        config.bindings.add(binding);
    }
    config.slots = {{"x_driver.step", PinField::step_count, 0},       {"x_driver.dir", PinField::direction, 0},
                    {"x_driver.enable", PinField::enable, 0},         {"x_max_switch.out", PinField::digital_in, 1},
                    {"x_encoder.channel_a", PinField::counter_delta, 0},
                    {"x_encoder.channel_b", PinField::counter_delta, 0}};
    config.connections = {{"x_driver.full_steps", "x_motor.full_steps"},
                          {"x_driver.direction", "x_motor.direction"},
                          {"x_driver.enabled", "x_motor.enabled"},
                          {"x_driver.supply", "x_motor.supply"}};
    config.rails = {RailSpec{"motor_power", "motor_supply.voltage", {}, {"x_driver.vmot"}},
                    RailSpec{"logic", "logic_supply.voltage", {}, {}}};
    config.controller_rail = "logic";
    config.actuators = {ActuatorOutput{"x_motor.torque", 0, "N*m"}};
    config.velocity_dofs = 1;
    auto layer = std::make_unique<SignalLayer>(std::move(config));
    layer->add_component(std::make_unique<PowerSupply>("motor_supply", 24.0), Stage::power);
    layer->add_component(std::make_unique<PowerSupply>("logic_supply", 3.3), Stage::power);
    EncoderParams encoder;
    encoder.coordinate = JointCoordinate{CoordinateKind::angular, 0, {}, {0.0, 0.0, 1.0}};
    encoder.resolution = 2.0 * std::numbers::pi / 2400.0;
    layer->add_component(std::make_unique<IncrementalEncoder>("x_encoder", encoder), Stage::sensor);
    LimitSwitchParams limit;
    limit.coordinate = JointCoordinate{CoordinateKind::linear, 0, {}, {1.0, 0.0, 0.0}};
    limit.position = 0.5;
    layer->add_component(std::make_unique<LimitSwitch>("x_max_switch", limit), Stage::sensor);
    layer->add_component(std::make_unique<StepDirDriver>("x_driver", StepDirDriverParams{16, true}), Stage::driver);
    BehavioralStepperParams motor;
    motor.step_angle = std::numbers::pi / 100.0;
    motor.torque_speed = {{0.0, 0.40}, {100.0, 0.30}};
    layer->add_component(std::make_unique<BehavioralStepper>("x_motor", motor), Stage::actuator);
    return layer;
}

}  // namespace

// Physics -> signals -> controller -> signals -> physics with no setter call: every value below crossed the kernel's
// coupling table (KER-008, KER-020, invariant 1).
TEST_CASE("Physics, signal layer and controller parties exchange every crossing through the kernel",
          "[KER-008][KER-009][KER-020][SIG-024][CTL-014]") {
    auto seen = std::make_shared<Seen>();
    kernel::CoSimKernel kernel{kernel::RunConfig{}};
    kernel.register_party(
        std::make_unique<kernel::PhysicsParty>(std::make_unique<ShaftAdapter>(seen), kernel::PhysicsPartyConfig{}));
    auto layer = chain_layer();
    const signals::SignalLayer* layer_view = layer.get();
    kernel.register_party(std::move(layer));
    controller::ControllerPartyConfig config;
    config.rail = "logic";
    kernel.register_party(std::make_unique<controller::ControllerParty>(std::make_unique<PulsingRuntime>(seen), config));
    kernel::InMemoryRecorderSink sink;
    const kernel::RunReport report = kernel.run(kernel::EndCondition{kernel::Seconds(5, 1000), {}}, sink);
    REQUIRE(report.outcome == kernel::RunOutcome::end_condition_met);

    SECTION("controller party: reset on the logic rail, pin inputs from the layer") {
        CHECK(seen->resets == std::vector<double>{3.3});
        REQUIRE(seen->pin_inputs.size() == 5);
        CHECK(seen->pin_inputs[0].digital_in[1] == 1);
        int counts = 0;
        for (const pins::PinInputs& inputs : seen->pin_inputs) {
            counts += inputs.counter_delta[0];
        }
        // The layer first samples after step 1 (0.01 rad, count floor(0.01 / (2 pi / 2400)) = 3, the reference) and
        // last after step 5 (0.05 rad, count 19): 16 counts crossed.
        CHECK(counts == 16);
        CHECK(sink.rows().back().controller_state == "3");
    }
    SECTION("signal layer: pin outputs from the controller, physics state for the stepper") {
        CHECK(layer_view->signal_value("x_driver.full_steps") == 1.0);
        CHECK(layer_view->signal_value("x_motor.stalled") == 0.0);
    }
    SECTION("physics party: the layer's actuator torque, demand = J alpha + load with J = 0") {
        REQUIRE(seen->physics_inputs.size() == 5);
        CHECK(seen->physics_inputs[0].generalized_forces.empty());
        // Tick 1 applies the layer's tick 0 output, before any pin output: driver disabled, no torque available.
        // Tick 2 applies tick 1's, driven by the controller's tick 0 pulses: the stepper demands the load torque.
        CHECK(seen->physics_inputs[1].generalized_forces == std::vector<double>{0.0});
        CHECK(seen->physics_inputs[2].generalized_forces == std::vector<double>{load_torque});
    }
    SECTION("the coupling table holds every crossing, publisher by name") {
        const kernel::CouplingTable& table = kernel.coupling();
        CHECK(table.publisher("physics.outputs") == "physics");
        CHECK(table.publisher("signals.pin_inputs") == "signals");
        CHECK(table.publisher("signals.physics_inputs") == "signals");
        CHECK(table.publisher("signals.rails") == "signals");
        CHECK(table.publisher("signals.operator_status") == "signals");
        CHECK(table.publisher("controller.pin_outputs") == "controller");
        CHECK(sink.rows().back().coupling.size() == 6);
    }
}

TEST_CASE("Controller party consumes pin inputs only without a rail and refuses a missing rail", "[CTL-013][KER-020]") {
    CHECK_THROWS_AS(controller::ControllerParty(nullptr, controller::ControllerPartyConfig{}), std::invalid_argument);
    controller::ControllerPartyConfig config;
    const controller::ControllerParty plain(std::make_unique<controller::NullControllerRuntime>(kernel::Rate(1000)),
                                            config);
    CHECK(plain.consumed().size() == 1);
    CHECK(plain.role() == kernel::PartyRole::controller);
    auto seen = std::make_shared<Seen>();
    kernel::CoSimKernel kernel{kernel::RunConfig{}};
    kernel.register_party(
        std::make_unique<kernel::PhysicsParty>(std::make_unique<ShaftAdapter>(seen), kernel::PhysicsPartyConfig{}));
    kernel.register_party(chain_layer());
    config.rail = "no_such_rail";
    kernel.register_party(std::make_unique<controller::ControllerParty>(std::make_unique<PulsingRuntime>(seen), config));
    kernel::InMemoryRecorderSink sink;
    const kernel::RunReport report = kernel.run(kernel::EndCondition{kernel::Seconds(5, 1000), {}}, sink);
    CHECK(report.outcome == kernel::RunOutcome::run_error);
    CHECK(seen->resets.empty());
}
