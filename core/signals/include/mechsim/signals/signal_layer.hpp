#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/pins/pins.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/operator_link.hpp"
#include "mechsim/signals/pin_binding.hpp"
#include "mechsim/signals/power_rail.hpp"
#include "mechsim/signals/signal_component.hpp"

namespace mechsim::signals {

// Evaluation order inside one layer step: safety and supplies, then rails, then sensors, front ends (amplifiers
// between a sensor and its pin), drivers, actuators.
enum class Stage : std::uint8_t {
    power,
    sensor,
    driver,
    actuator,
    front_end,
};

// A PinInputs or PinOutputs field.
enum class PinField : std::uint8_t {
    digital_in,     // digital_input
    counter_delta,  // counter_input; several signals may share a channel and sum (A and B phases)
    adc_sample,     // adc_input
    uart_rx,        // uart_rx, channel 0, an OperatorLink's rx
    step_count,     // pulse_output
    direction,      // digital_output
    enable,         // digital_output
    digital_out,    // digital_output
    uart_tx,        // uart_tx, channel 0, an OperatorLink's tx
};

// Places a bound signal in a channel of the controller interface. The pin table says which pin and mode; the slot
// says which array element (the chip data carries no channel numbers; the builder assigns them).
struct ChannelSlot {
    std::string signal;
    PinField field = PinField::digital_in;
    std::size_t channel = 0;
};

struct Connection {
    std::string from;  // component output
    std::string to;    // component input
};

// A component input set from outside (the test runner through the kernel's coupling table), e.g. an E-stop loop.
struct ExternalInput {
    std::string signal;
    double initial = 0.0;
};

// A rail is on when its supply signal is above 0 V and every gate signal is 1 (contacts closed). Each consumer
// input receives the rail voltage, 0 when off (SIG-015, SIG-016).
struct RailSpec {
    std::string name;
    std::string supply;
    std::vector<std::string> gates;
    std::vector<std::string> consumers;
};

// A component output recorded as an actuator effort and, when applied, added to PhysicsInputs::generalized_forces[dof].
// A stepper's torque is a read-back of its joint drive (SIG-033), recorded only.
struct ActuatorOutput {
    std::string signal;
    std::size_t dof = 0;
    std::string unit = "N*m";
    bool applied = true;
};

// Coupling payloads the layer consumes from the test runner (KER-008): external input values by signal, and operator
// commands posted once per new sequence number (a held value is not re-posted, KER-011).
using ExternalInputs = std::map<std::string, double, std::less<>>;
struct OperatorCommands {
    std::int64_t sequence = 0;
    std::vector<OperatorMessage> messages;
};
inline constexpr const char* external_inputs_kind = "ExternalInputs";
inline constexpr const char* operator_commands_kind = "OperatorCommands";
inline constexpr const char* operator_status_kind = "OperatorStatus";

struct SignalLayerConfig {
    std::string name = "signals";
    std::string physics_outputs = "physics.outputs";         // PhysicsOutputs payload
    std::string pin_outputs = "controller.pin_outputs";      // PinOutputs payload
    kernel::Rate rate{1000};
    // The controller's step rate (default: the layer's). Per-tick pin buffers cross once per controller step
    // (SIG-038): pin inputs' counter deltas and UART bytes accumulate over the layer's steps until the controller
    // has stepped, and pin outputs' step counts and UART bytes are taken only at the first layer step after it did.
    std::optional<kernel::Rate> controller_rate;
    std::uint64_t seed = 0;
    PinBindingTable bindings;
    std::vector<ChannelSlot> slots;
    std::vector<Connection> connections;
    std::vector<ExternalInput> externals;
    std::vector<RailSpec> rails;
    std::string controller_rail;  // empty: none
    std::vector<ActuatorOutput> actuators;
    std::size_t velocity_dofs = 0;
};

// The signal layer party (KER-009, SIG-024). It owns the components, the pin tables and the rails. Per step: pin
// outputs and external inputs onto the signal bus, power stage, rails, then sensor, front_end, driver and actuator
// stages, each component begin_tick / set_inputs / sample / latch_outputs in registration order, its take_notes()
// recorded as `approximation` events; then pin inputs from the bus
// through the slots only, and applied actuator torques and every component's joint drives (registration order) into
// PhysicsInputs (KER-027), drives on one joint combined as parallel springs (ruling 23). Each component samples the
// state with its own share of the combined drive's read-back torque in joint_drive_torques (SIG-039): a disabled drive
// 0, the joint's only enabled drive the whole, else tau_i = K_i (r_i - theta) - D_i omega of its last published drive
// (theta the joint coordinate, omega its backward difference over the layer period) plus (T - sum tau) |tau_i| /
// sum |tau| (stiffness, then damping, then equal shares when every tau_i is 0), so the shares sum to T.
// Kernel coupling (KER-008, invariant 1): consumes config.physics_outputs, config.pin_outputs, `<name>.external_inputs`
// (only with externals configured) and `<name>.operator_commands` (only with an operator link); publishes
// `<name>.pin_inputs` (PinInputs), `.physics_inputs` (PhysicsInputs), `.rails` (PowerRails: std::vector<PowerRail>)
// and `.operator_status` (this step's std::vector<StatusMessage>). The setters and getters below are the same data
// for unit tests that drive the layer without a kernel.
class SignalLayer final : public kernel::Party {
public:
    explicit SignalLayer(SignalLayerConfig config);

    // Throws SignalNameError on a repeated signal; std::invalid_argument on a null component or after start().
    void add_component(std::unique_ptr<SignalComponent> component, Stage stage);
    // The link runs in the sensor stage; at most one.
    void add_operator_link(std::unique_ptr<OperatorLink> link);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] kernel::Rate nominal_rate() const override;
    [[nodiscard]] std::span<const std::string> published() const override;
    [[nodiscard]] std::span<const std::string> consumed() const override;
    // Validates the wiring (SignalConfigError naming the fault), then starts every component.
    void start(const kernel::Rate& base_rate) override;
    void read_inputs(const kernel::CouplingReader& inputs) override;
    void step(kernel::SimTime t, kernel::Duration dt) override;
    void write_outputs(kernel::CouplingWriter& outputs) override;
    void on_fault(const kernel::PartyFault& fault) noexcept override;

    // Coupling surface.
    void set_physical_state(const PhysicalState& state);
    void set_pin_outputs(const pins::PinOutputs& outputs);
    // Throws SignalConfigError on a signal not declared external.
    void set_external_input(std::string_view signal, double value);
    // Throws SignalConfigError without an operator link; the bytes leave on the following steps.
    void post_operator_message(const OperatorMessage& message);
    // Status messages decoded since the last call, in arrival order.
    [[nodiscard]] std::vector<StatusMessage> take_operator_status();
    [[nodiscard]] const pins::PinInputs& pin_inputs() const noexcept;
    [[nodiscard]] const kernel::PhysicsInputs& physics_inputs() const noexcept;
    [[nodiscard]] std::span<const PowerRail> rails() const noexcept;
    // The controller's rail, for ControllerRuntime::brownout() (SIG-022).
    [[nodiscard]] std::optional<PowerRail> controller_rail() const;
    // Throws SignalConfigError on an unknown signal.
    [[nodiscard]] double signal_value(std::string_view signal) const;
    // Appends every bound pin signal, every rail, every actuator effort and this step's events (SIG-028).
    void record(kernel::TickRecord& row) const override;

private:
    struct Entry {
        std::unique_ptr<SignalComponent> component;
        Stage stage = Stage::sensor;
        std::vector<std::size_t> input_sources;  // bus index per input
        std::vector<std::size_t> output_bus;     // bus index per output
        std::vector<double> input_values;
        std::vector<kernel::JointDrive> published;     // its joint drives at the last step
        std::vector<kernel::JointDriveTorque> shares;  // its part of the read-back torques this step
    };
    struct ResolvedSlot {
        PinField field = PinField::digital_in;
        std::size_t channel = 0;
        std::size_t bus = 0;
    };
    struct ResolvedRail {
        std::size_t supply = 0;
        std::vector<std::size_t> gates;
        std::vector<std::size_t> consumers;
    };

    [[nodiscard]] std::size_t bus_index(const std::string& signal) const;
    [[nodiscard]] std::size_t intern(const std::string& signal);
    void resolve();
    void apportion_drive_torques();
    void run_stage(Stage stage, kernel::SimTime t);
    void update_rails(kernel::SimTime t);
    void read_pin_outputs(bool fresh);
    void write_pin_inputs(bool consumed);
    [[nodiscard]] bool controller_stepped_before(kernel::SimTime t) const;
    void event(kernel::SimTime t, std::string source, std::string kind, std::string detail);

    SignalLayerConfig config_;
    std::vector<std::string> published_;
    std::vector<std::string> consumed_;
    std::vector<Entry> entries_;
    OperatorLink* link_ = nullptr;  // non-owning: the link is owned by entries_
    std::map<std::string, std::size_t, std::less<>> bus_by_name_;
    std::vector<std::string> bus_names_;
    std::vector<double> bus_;
    std::map<std::string, std::size_t, std::less<>> externals_;
    std::vector<ResolvedSlot> slots_;
    std::vector<ResolvedRail> rail_wiring_;
    std::vector<PowerRail> rails_;
    std::vector<std::pair<std::size_t, std::size_t>> actuators_;  // bus, dof
    std::vector<bool> applied_;
    std::int64_t controller_period_ = 1;
    std::optional<kernel::SimTime> last_step_;
    std::vector<std::size_t> stall_outputs_;
    std::vector<double> last_stall_;
    bool started_ = false;
    PhysicalState state_;
    std::vector<kernel::JointDriveTorque> read_back_;
    std::vector<double> last_positions_;
    bool positions_primed_ = false;
    double period_seconds_ = 0.0;
    pins::PinOutputs pin_outputs_;
    pins::PinInputs pin_inputs_;
    kernel::PhysicsInputs physics_inputs_;
    std::vector<kernel::RecordedEvent> events_;
    std::vector<StatusMessage> status_;
    std::vector<StatusMessage> step_status_;
    std::optional<std::int64_t> command_sequence_;
};

}  // namespace mechsim::signals
