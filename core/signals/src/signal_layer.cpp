#include "mechsim/signals/signal_layer.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/kernel/coupling.hpp"
#include "mechsim/signals/component_base.hpp"

namespace mechsim::signals {

// ===== Helpers =====================================================================================================
namespace {

constexpr std::size_t no_source = std::numeric_limits<std::size_t>::max();
constexpr double adc_max = 65535.0;

bool level(double value) noexcept {
    return value >= 0.5;
}

bool is_input_side(PinField field) noexcept {
    return field == PinField::digital_in || field == PinField::counter_delta || field == PinField::adc_sample ||
           field == PinField::uart_rx;
}

bool mode_fits(PinField field, PeripheralMode mode) noexcept {
    switch (field) {
        case PinField::digital_in:
            return mode == PeripheralMode::digital_input;
        case PinField::counter_delta:
            return mode == PeripheralMode::counter_input;
        case PinField::adc_sample:
            return mode == PeripheralMode::adc_input;
        case PinField::uart_rx:
            return mode == PeripheralMode::uart_rx;
        case PinField::step_count:
            return mode == PeripheralMode::pulse_output;
        case PinField::direction:
        case PinField::enable:
        case PinField::digital_out:
            return mode == PeripheralMode::digital_output;
        case PinField::uart_tx:
            return mode == PeripheralMode::uart_tx;
    }
    return false;
}

std::size_t channels(PinField field) noexcept {
    switch (field) {
        case PinField::digital_in:
            return pins::digital_in_channels;
        case PinField::counter_delta:
            return pins::counter_channels;
        case PinField::adc_sample:
            return pins::adc_channels;
        case PinField::step_count:
        case PinField::direction:
        case PinField::enable:
            return pins::stepper_channels;
        case PinField::digital_out:
            return pins::digital_out_channels;
        case PinField::uart_rx:
        case PinField::uart_tx:
            return 1;
    }
    return 0;
}

std::string unit_of(PeripheralMode mode) {
    switch (mode) {
        case PeripheralMode::digital_input:
        case PeripheralMode::digital_output:
            return "1";
        case PeripheralMode::counter_input:
        case PeripheralMode::pulse_output:
        case PeripheralMode::adc_input:
            return "count";
        case PeripheralMode::uart_rx:
        case PeripheralMode::uart_tx:
            return "byte";
    }
    return "1";
}

std::string format_number(double value) {
    std::array<char, 32> buffer{};
    const std::to_chars_result result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    return {buffer.data(), result.ptr};
}

bool ends_with_stalled(std::string_view name) noexcept {
    return name.ends_with(".stalled");
}

struct DriveSum {
    kernel::JointDrive first;
    double stiffness = 0.0;
    double weighted_rest = 0.0;
    double damping = 0.0;
    std::size_t enabled_count = 0;
};

// Drives on one joint act as parallel springs (ruling 23): K = sum K_i, rest = sum K_i r_i / K, D = sum D_i over the
// enabled drives, exact for linear springs, so KER-027's one drive per joint holds at the kernel. Joints keep the order
// of their first drive. A joint with at most one enabled drive publishes that drive (else its first) unchanged; enabled
// drives with K = 0 keep the first enabled drive's rest.
std::vector<kernel::JointDrive> combine_drives(const std::vector<kernel::JointDrive>& drives) {
    std::vector<DriveSum> sums;
    for (const kernel::JointDrive& drive : drives) {
        const auto found = std::find_if(sums.begin(), sums.end(),
                                        [&drive](const DriveSum& sum) { return sum.first.joint == drive.joint; });
        const auto index = static_cast<std::size_t>(std::distance(sums.begin(), found));
        if (found == sums.end()) {
            DriveSum fresh;
            fresh.first = drive;
            sums.push_back(fresh);
        }
        DriveSum& sum = sums[index];
        if (!drive.enabled) {
            continue;
        }
        if (sum.enabled_count == 0) {
            sum.first = drive;
        }
        sum.enabled_count += 1;
        sum.stiffness += drive.stiffness;
        sum.weighted_rest += drive.stiffness * drive.rest;
        sum.damping += drive.damping;
    }
    std::vector<kernel::JointDrive> combined;
    combined.reserve(sums.size());
    for (const DriveSum& sum : sums) {
        if (sum.enabled_count <= 1) {
            combined.push_back(sum.first);
            continue;
        }
        kernel::JointDrive drive = sum.first;
        drive.stiffness = sum.stiffness;
        drive.damping = sum.damping;
        if (sum.stiffness > 0.0) {
            drive.rest = sum.weighted_rest / sum.stiffness;
        }
        combined.push_back(drive);
    }
    return combined;
}

}  // namespace

// ===== Construction ================================================================================================
SignalLayer::SignalLayer(SignalLayerConfig config) : config_(std::move(config)) {
    published_ = {config_.name + ".pin_inputs", config_.name + ".physics_inputs", config_.name + ".rails",
                  config_.name + ".operator_status"};
    consumed_ = {config_.physics_outputs, config_.pin_outputs};
    if (!config_.externals.empty()) {
        consumed_.push_back(config_.name + ".external_inputs");
    }
}

std::size_t SignalLayer::intern(const std::string& signal) {
    const auto it = bus_by_name_.find(signal);
    if (it != bus_by_name_.end()) {
        return it->second;
    }
    const std::size_t index = bus_names_.size();
    bus_by_name_.emplace(signal, index);
    bus_names_.push_back(signal);
    bus_.push_back(0.0);
    return index;
}

std::size_t SignalLayer::bus_index(const std::string& signal) const {
    const auto it = bus_by_name_.find(signal);
    if (it == bus_by_name_.end()) {
        throw SignalConfigError("signal '" + signal + "' belongs to no component of the layer");
    }
    return it->second;
}

void SignalLayer::add_component(std::unique_ptr<SignalComponent> component, Stage stage) {
    if (component == nullptr) {
        throw std::invalid_argument("signal layer: null component");
    }
    if (started_) {
        throw std::invalid_argument("signal layer: component added after start");
    }
    SignalTable table;
    for (const Entry& entry : entries_) {
        table.add(*entry.component);
    }
    table.add(*component);
    Entry entry{std::move(component), stage, {}, {}, {}};
    for (const std::string& output : entry.component->outputs()) {
        entry.output_bus.push_back(intern(output));
    }
    for (const std::string& input : entry.component->inputs()) {
        entry.input_sources.push_back(intern(input));
    }
    entry.input_values.assign(entry.input_sources.size(), 0.0);
    entries_.push_back(std::move(entry));
}

void SignalLayer::add_operator_link(std::unique_ptr<OperatorLink> link) {
    if (link_ != nullptr) {
        throw std::invalid_argument("signal layer: one operator link at most");
    }
    OperatorLink* raw = link.get();
    add_component(std::move(link), Stage::sensor);
    link_ = raw;
    consumed_.push_back(config_.name + ".operator_commands");
}

std::string_view SignalLayer::name() const {
    return config_.name;
}

kernel::Rate SignalLayer::nominal_rate() const {
    return config_.rate;
}

std::span<const std::string> SignalLayer::published() const {
    return published_;
}

std::span<const std::string> SignalLayer::consumed() const {
    return consumed_;
}

// ===== Resolve =====================================================================================================
void SignalLayer::resolve() {
    std::vector<bool> is_output(bus_.size(), false);
    std::vector<bool> is_input(bus_.size(), false);
    for (const Entry& entry : entries_) {
        for (const std::size_t index : entry.output_bus) {
            is_output[index] = true;
        }
        for (const std::size_t index : entry.input_sources) {
            is_input[index] = true;
        }
    }
    std::vector<std::size_t> sources(bus_.size(), 0);
    std::vector<std::size_t> connected(bus_.size(), no_source);
    const auto require_output = [&](const std::string& signal) {
        const std::size_t index = bus_index(signal);
        if (!is_output[index]) {
            throw SignalConfigError("signal '" + signal + "' is not a component output");
        }
        return index;
    };
    const auto require_input = [&](const std::string& signal) {
        const std::size_t index = bus_index(signal);
        if (!is_input[index]) {
            throw SignalConfigError("signal '" + signal + "' is not a component input");
        }
        ++sources[index];
        return index;
    };

    std::set<std::pair<PinField, std::size_t>> used;
    for (const ChannelSlot& slot : config_.slots) {
        const std::optional<PinBinding> binding = config_.bindings.find_signal(slot.signal);
        if (!binding.has_value()) {
            throw SignalConfigError("slot signal '" + slot.signal + "' has no pin-binding entry");
        }
        if (!mode_fits(slot.field, binding->mode)) {
            throw SignalConfigError("slot signal '" + slot.signal + "' is bound in another peripheral mode");
        }
        if (slot.channel >= channels(slot.field)) {
            throw SignalConfigError("slot signal '" + slot.signal + "' names a channel outside the interface");
        }
        const bool fresh = used.emplace(slot.field, slot.channel).second;
        if (!fresh && slot.field != PinField::counter_delta) {
            throw SignalConfigError("slot signal '" + slot.signal + "' reuses a channel");
        }
        std::size_t index = 0;
        if (is_input_side(slot.field)) {
            index = require_output(slot.signal);
        } else {
            index = require_input(slot.signal);
        }
        const bool link_field = slot.field == PinField::uart_rx || slot.field == PinField::uart_tx;
        if (link_field) {
            const bool links_signal = link_ != nullptr && (slot.signal == link_->outputs().front() ||
                                                           slot.signal == link_->inputs().front());
            if (!links_signal) {
                throw SignalConfigError("UART slot signal '" + slot.signal + "' is not the operator link's");
            }
        }
        slots_.push_back(ResolvedSlot{slot.field, slot.channel, index});
    }
    for (const Connection& connection : config_.connections) {
        const std::size_t from = require_output(connection.from);
        const std::size_t to = require_input(connection.to);
        connected[to] = from;
    }
    for (const ExternalInput& external : config_.externals) {
        const std::size_t index = require_input(external.signal);
        externals_.emplace(external.signal, index);
        bus_[index] = external.initial;
    }
    for (const RailSpec& rail : config_.rails) {
        ResolvedRail wiring{require_output(rail.supply), {}, {}};
        for (const std::string& gate : rail.gates) {
            wiring.gates.push_back(require_output(gate));
        }
        for (const std::string& consumer : rail.consumers) {
            wiring.consumers.push_back(require_input(consumer));
        }
        rail_wiring_.push_back(std::move(wiring));
        rails_.push_back(PowerRail{rail.name, 0.0, false});
    }
    if (!config_.controller_rail.empty()) {
        const bool known = std::any_of(config_.rails.begin(), config_.rails.end(),
                                       [this](const RailSpec& r) { return r.name == config_.controller_rail; });
        if (!known) {
            throw SignalConfigError("controller rail '" + config_.controller_rail + "' is not a rail");
        }
    }
    for (const ActuatorOutput& actuator : config_.actuators) {
        if (actuator.dof >= config_.velocity_dofs) {
            throw SignalConfigError("actuator '" + actuator.signal + "' names a DOF outside the model");
        }
        actuators_.emplace_back(require_output(actuator.signal), actuator.dof);
        applied_.push_back(actuator.applied);
    }
    for (const Entry& entry : entries_) {
        for (const std::size_t index : entry.input_sources) {
            if (sources[index] != 1) {
                throw SignalConfigError("input '" + bus_names_[index] + "' needs exactly one source, has " +
                                        std::to_string(sources[index]));
            }
        }
        for (const std::size_t index : entry.output_bus) {
            if (ends_with_stalled(bus_names_[index])) {
                stall_outputs_.push_back(index);
            }
        }
    }
    last_stall_.assign(stall_outputs_.size(), 0.0);
    for (Entry& entry : entries_) {
        for (std::size_t& source : entry.input_sources) {
            if (connected[source] != no_source) {
                source = connected[source];
            }
        }
    }
}

// ===== Start =======================================================================================================
void SignalLayer::start(const kernel::Rate& base_rate) {
    if (started_) {
        throw std::invalid_argument("signal layer: started twice");
    }
    resolve();
    started_ = true;
    const kernel::Duration period = kernel::period_in_ticks(config_.rate, base_rate);
    for (Entry& entry : entries_) {
        const ComponentClock clock{base_rate, period,
                                   derive_child_seed(config_.seed, entry.component->instance_name())};
        entry.component->start(clock);
    }
    physics_inputs_.generalized_forces.assign(config_.velocity_dofs, 0.0);
    controller_period_ = kernel::period_in_ticks(config_.controller_rate.value_or(config_.rate), base_rate).count();
}

// The controller steps after the layer within a tick, so it has consumed what the layer published at its last step,
// and published new outputs, when one of its period boundaries lies in [last layer step, t).
bool SignalLayer::controller_stepped_before(kernel::SimTime t) const {
    if (!last_step_.has_value()) {
        return true;
    }
    const std::int64_t from = last_step_->count();
    const std::int64_t next = ((from + controller_period_ - 1) / controller_period_) * controller_period_;
    return next < t.count();
}

void SignalLayer::event(kernel::SimTime t, std::string source, std::string kind, std::string detail) {
    events_.push_back(kernel::RecordedEvent{t, std::move(source), std::move(kind), std::move(detail)});
}

// ===== ReadPinOutputs ==============================================================================================
void SignalLayer::read_pin_outputs(bool fresh) {
    for (const ResolvedSlot& slot : slots_) {
        switch (slot.field) {
            case PinField::step_count:
                bus_[slot.bus] = 0.0;
                if (fresh) {
                    bus_[slot.bus] = static_cast<double>(pin_outputs_.step_count.at(slot.channel));
                }
                break;
            case PinField::direction:
                bus_[slot.bus] = static_cast<double>(pin_outputs_.direction.at(slot.channel) != 0U);
                break;
            case PinField::enable:
                bus_[slot.bus] = static_cast<double>(pin_outputs_.enable.at(slot.channel) != 0U);
                break;
            case PinField::digital_out:
                bus_[slot.bus] = static_cast<double>(pin_outputs_.digital_out.at(slot.channel) != 0U);
                break;
            case PinField::uart_tx: {
                bus_[slot.bus] = 0.0;
                if (!fresh) {
                    break;
                }
                if (pin_outputs_.uart_tx_count > pins::uart_buffer_bytes) {
                    throw SignalRunError("controller sent " + std::to_string(pin_outputs_.uart_tx_count) +
                                         " UART bytes in one tick, over 64");
                }
                link_->accept_controller_bytes(
                    std::span<const std::uint8_t>(pin_outputs_.uart_tx.data(), pin_outputs_.uart_tx_count));
                bus_[slot.bus] = static_cast<double>(pin_outputs_.uart_tx_count);
                break;
            }
            case PinField::digital_in:
            case PinField::counter_delta:
            case PinField::adc_sample:
            case PinField::uart_rx:
                break;
        }
    }
}

// ===== RunStage ====================================================================================================
void SignalLayer::run_stage(Stage stage, kernel::SimTime t) {
    for (Entry& entry : entries_) {
        if (entry.stage != stage) {
            continue;
        }
        SignalComponent& component = *entry.component;
        component.begin_tick(t);
        for (std::size_t i = 0; i < entry.input_sources.size(); ++i) {
            entry.input_values[i] = bus_[entry.input_sources[i]];
        }
        component.set_inputs(entry.input_values);
        component.sample(state_);
        component.latch_outputs();
        for (std::string& line : component.take_notes()) {
            event(t, std::string(component.instance_name()), "approximation", std::move(line));
        }
        const std::span<const double> values = component.output_values();
        for (std::size_t i = 0; i < entry.output_bus.size(); ++i) {
            bus_[entry.output_bus[i]] = values[i];
        }
    }
}

// ===== UpdateRails =================================================================================================
void SignalLayer::update_rails(kernel::SimTime t) {
    for (std::size_t r = 0; r < rails_.size(); ++r) {
        const ResolvedRail& wiring = rail_wiring_[r];
        const double supply = bus_[wiring.supply];
        const bool gates_closed = std::all_of(wiring.gates.begin(), wiring.gates.end(),
                                              [this](std::size_t gate) { return level(bus_[gate]); });
        const bool on = supply > 0.0 && gates_closed;
        double voltage = 0.0;
        if (on) {
            voltage = supply;
        }
        if (on != rails_[r].on && t.count() > 0) {
            std::string kind = "rail_off";
            if (on) {
                kind = "rail_on";
            }
            event(t, rails_[r].name, std::move(kind), format_number(voltage) + " V");
        }
        rails_[r].on = on;
        rails_[r].voltage = voltage;
        for (const std::size_t consumer : wiring.consumers) {
            bus_[consumer] = voltage;
        }
    }
}

// ===== WritePinInputs ==============================================================================================
void SignalLayer::write_pin_inputs(bool consumed) {
    const pins::PinInputs held = pin_inputs_;
    pin_inputs_ = pins::PinInputs{};
    if (!consumed) {
        pin_inputs_.counter_delta = held.counter_delta;
        pin_inputs_.uart_rx = held.uart_rx;
        pin_inputs_.uart_rx_count = held.uart_rx_count;
    }
    for (const ResolvedSlot& slot : slots_) {
        const double value = bus_[slot.bus];
        switch (slot.field) {
            case PinField::digital_in:
                pin_inputs_.digital_in.at(slot.channel) = static_cast<std::uint8_t>(level(value));
                break;
            case PinField::counter_delta: {
                const std::int64_t sum = pin_inputs_.counter_delta.at(slot.channel) + std::llround(value);
                if (sum > std::numeric_limits<std::int32_t>::max() || sum < std::numeric_limits<std::int32_t>::min()) {
                    throw SignalRunError("counter channel " + std::to_string(slot.channel) + " overflows in a tick");
                }
                pin_inputs_.counter_delta.at(slot.channel) = static_cast<std::int32_t>(sum);
                break;
            }
            case PinField::adc_sample:
                pin_inputs_.adc_sample.at(slot.channel) =
                    static_cast<std::uint16_t>(std::lround(std::clamp(value, 0.0, adc_max)));
                break;
            case PinField::uart_rx: {
                const std::span<const std::uint8_t> bytes = link_->controller_rx();
                const std::size_t held_bytes = pin_inputs_.uart_rx_count;
                if (held_bytes + bytes.size() > pins::uart_buffer_bytes) {
                    throw SignalRunError("UART rx: " + std::to_string(held_bytes + bytes.size()) +
                                         " bytes for one controller step, over 64 (SIG-038)");
                }
                std::copy(bytes.begin(), bytes.end(),
                          pin_inputs_.uart_rx.begin() + static_cast<std::ptrdiff_t>(held_bytes));
                pin_inputs_.uart_rx_count = static_cast<std::uint16_t>(held_bytes + bytes.size());
                break;
            }
            case PinField::step_count:
            case PinField::direction:
            case PinField::enable:
            case PinField::digital_out:
            case PinField::uart_tx:
                break;
        }
    }
}

// ===== Step ========================================================================================================
void SignalLayer::step(kernel::SimTime t, kernel::Duration /*dt*/) {
    if (!started_) {
        throw SignalRunError("signal layer stepped before start");
    }
    events_.clear();
    step_status_.clear();
    const bool controller_stepped = controller_stepped_before(t);
    last_step_ = t;
    read_pin_outputs(controller_stepped);
    if (link_ != nullptr) {
        for (StatusMessage& status : link_->take_status()) {
            std::string detail = status.label;
            const MessageSpec& status_spec = link_->codec().spec(status.message.name);
            for (std::size_t f = 0; f < status.message.fields.size(); ++f) {
                detail += " " + status_spec.fields.at(f) + "=" + format_number(status.message.fields[f]);
            }
            event(t, std::string(link_->instance_name()), "operator_status", std::move(detail));
            step_status_.push_back(status);
            status_.push_back(std::move(status));
        }
        for (const OperatorMessage& command : link_->take_posted()) {
            const MessageSpec& command_spec = link_->codec().spec(command.name);
            std::string detail = command_spec.label;
            for (std::size_t f = 0; f < command.fields.size(); ++f) {
                detail += " " + command_spec.fields.at(f) + "=" + format_number(command.fields[f]);
            }
            event(t, std::string(link_->instance_name()), "operator_command", std::move(detail));
        }
    }
    run_stage(Stage::power, t);
    update_rails(t);
    run_stage(Stage::sensor, t);
    run_stage(Stage::front_end, t);
    run_stage(Stage::driver, t);
    run_stage(Stage::actuator, t);
    write_pin_inputs(controller_stepped);
    std::fill(physics_inputs_.generalized_forces.begin(), physics_inputs_.generalized_forces.end(), 0.0);
    for (std::size_t i = 0; i < actuators_.size(); ++i) {
        if (applied_[i]) {
            physics_inputs_.generalized_forces[actuators_[i].second] += bus_[actuators_[i].first];
        }
    }
    std::vector<kernel::JointDrive> published;
    for (const Entry& entry : entries_) {
        const std::span<const kernel::JointDrive> drives = entry.component->joint_drives();
        published.insert(published.end(), drives.begin(), drives.end());
    }
    physics_inputs_.joint_drives = combine_drives(published);
    for (std::size_t i = 0; i < stall_outputs_.size(); ++i) {
        const double stalled = bus_[stall_outputs_[i]];
        if (level(stalled) && !level(last_stall_[i])) {
            event(t, bus_names_[stall_outputs_[i]], "motor_stall", "demand above available torque");
        }
        last_stall_[i] = stalled;
    }
}

void SignalLayer::on_fault(const kernel::PartyFault& /*fault*/) noexcept {}

void SignalLayer::read_inputs(const kernel::CouplingReader& inputs) {
    if (inputs.has_value(config_.physics_outputs)) {
        state_ = inputs.payload<kernel::PhysicsOutputs>(config_.physics_outputs);
    }
    if (inputs.has_value(config_.pin_outputs)) {
        pin_outputs_ = inputs.payload<pins::PinOutputs>(config_.pin_outputs);
    }
    if (!config_.externals.empty()) {
        const std::string externals = config_.name + ".external_inputs";
        if (inputs.has_value(externals)) {
            for (const auto& [signal, value] : inputs.payload<ExternalInputs>(externals)) {
                set_external_input(signal, value);
            }
        }
    }
    if (link_ != nullptr) {
        const std::string commands = config_.name + ".operator_commands";
        if (inputs.has_value(commands)) {
            const auto& batch = inputs.payload<OperatorCommands>(commands);
            if (!command_sequence_.has_value() || *command_sequence_ != batch.sequence) {
                command_sequence_ = batch.sequence;
                for (const OperatorMessage& message : batch.messages) {
                    link_->post(message);
                }
            }
        }
    }
}

void SignalLayer::write_outputs(kernel::CouplingWriter& outputs) {
    outputs.set(published_[0], kernel::make_payload(pin_inputs_, pins::pin_inputs_kind));
    outputs.set(published_[1], kernel::make_payload(physics_inputs_, "PhysicsInputs"));
    outputs.set(published_[2], kernel::make_payload(rails_, pins::power_rails_kind));
    outputs.set(published_[3], kernel::make_payload(step_status_, operator_status_kind));
}

// ===== Coupling ====================================================================================================
void SignalLayer::set_physical_state(const PhysicalState& state) {
    state_ = state;
}

void SignalLayer::set_pin_outputs(const pins::PinOutputs& outputs) {
    pin_outputs_ = outputs;
}

void SignalLayer::set_external_input(std::string_view signal, double value) {
    const auto it = externals_.find(signal);
    if (it == externals_.end()) {
        throw SignalConfigError("signal '" + std::string(signal) + "' is not an external input");
    }
    bus_[it->second] = value;
}

void SignalLayer::post_operator_message(const OperatorMessage& message) {
    if (link_ == nullptr) {
        throw SignalConfigError("signal layer has no operator link");
    }
    link_->post(message);
}

std::vector<StatusMessage> SignalLayer::take_operator_status() {
    std::vector<StatusMessage> status = std::move(status_);
    status_.clear();
    return status;
}

const pins::PinInputs& SignalLayer::pin_inputs() const noexcept {
    return pin_inputs_;
}

const kernel::PhysicsInputs& SignalLayer::physics_inputs() const noexcept {
    return physics_inputs_;
}

std::span<const PowerRail> SignalLayer::rails() const noexcept {
    return rails_;
}

std::optional<PowerRail> SignalLayer::controller_rail() const {
    for (const PowerRail& rail : rails_) {
        if (rail.name == config_.controller_rail) {
            return rail;
        }
    }
    return std::nullopt;
}

double SignalLayer::signal_value(std::string_view signal) const {
    const auto it = bus_by_name_.find(signal);
    if (it == bus_by_name_.end()) {
        throw SignalConfigError("signal '" + std::string(signal) + "' is not on the layer");
    }
    return bus_[it->second];
}

// ===== Record ======================================================================================================
void SignalLayer::record(kernel::TickRecord& row) const {
    for (const PinBinding& binding : config_.bindings.entries()) {
        const auto it = bus_by_name_.find(binding.signal);
        if (it != bus_by_name_.end()) {
            row.pin_signals.push_back(kernel::SeriesSample{binding.signal, unit_of(binding.mode), bus_[it->second]});
        }
    }
    for (const PowerRail& rail : rails_) {
        row.rails.push_back(kernel::SeriesSample{rail.name + ".voltage", "V", rail.voltage});
        row.rails.push_back(kernel::SeriesSample{rail.name + ".on", "1", static_cast<double>(rail.on)});
    }
    for (std::size_t i = 0; i < actuators_.size(); ++i) {
        row.actuator_efforts.push_back(kernel::SeriesSample{config_.actuators[i].signal, config_.actuators[i].unit,
                                                            bus_[actuators_[i].first]});
    }
    row.events.insert(row.events.end(), events_.begin(), events_.end());
}

}  // namespace mechsim::signals
