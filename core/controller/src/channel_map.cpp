#include "mechsim/controller/channel_map.hpp"

#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/signals/pin_binding.hpp"

namespace mechsim::controller {

namespace {

using signals::PeripheralMode;

std::string slot_name(PinSlot slot) {
    switch (slot) {
        case PinSlot::digital_in:
            return "digital_in";
        case PinSlot::counter_delta:
            return "counter_delta";
        case PinSlot::adc_sample:
            return "adc_sample";
        case PinSlot::uart_rx:
            return "uart_rx";
        case PinSlot::step_count:
            return "step_count";
        case PinSlot::digital_out:
            return "digital_out";
        case PinSlot::uart_tx:
            return "uart_tx";
    }
    return "unknown";
}

}  // namespace

std::size_t slot_capacity(PinSlot slot) noexcept {
    switch (slot) {
        case PinSlot::digital_in:
            return digital_in_channels;
        case PinSlot::counter_delta:
            return counter_channels;
        case PinSlot::adc_sample:
            return adc_channels;
        case PinSlot::uart_rx:
            return 1;
        case PinSlot::step_count:
            return stepper_channels;
        case PinSlot::digital_out:
            return digital_out_channels;
        case PinSlot::uart_tx:
            return 1;
    }
    return 0;
}

bool slot_accepts(PinSlot slot, PeripheralMode mode) noexcept {
    switch (slot) {
        case PinSlot::digital_in:
            return mode == PeripheralMode::digital_input;
        case PinSlot::counter_delta:
            return mode == PeripheralMode::counter_input;
        case PinSlot::adc_sample:
            return mode == PeripheralMode::adc_input;
        case PinSlot::uart_rx:
            return mode == PeripheralMode::uart_rx;
        case PinSlot::step_count:
            return mode == PeripheralMode::pulse_output;
        case PinSlot::digital_out:
            return mode == PeripheralMode::digital_output;
        case PinSlot::uart_tx:
            return mode == PeripheralMode::uart_tx;
    }
    return false;
}

void ChannelMap::add(ChannelAssignment assignment) {
    if (assignment.signal.empty()) {
        throw ControllerConfigError("channel assignment has an empty signal name");
    }
    if (assignment.channel >= slot_capacity(assignment.slot)) {
        throw ControllerConfigError("signal " + assignment.signal + ": channel " + std::to_string(assignment.channel) +
                                    " is beyond " + slot_name(assignment.slot) + ", which holds " +
                                    std::to_string(slot_capacity(assignment.slot)));
    }
    for (const ChannelAssignment& existing : entries_) {
        if (existing.signal == assignment.signal) {
            throw ControllerConfigError("signal " + assignment.signal + " is assigned twice");
        }
        if (existing.slot == assignment.slot && existing.channel == assignment.channel) {
            throw ControllerConfigError("signals " + existing.signal + " and " + assignment.signal + " share " +
                                        slot_name(assignment.slot) + " channel " +
                                        std::to_string(assignment.channel));
        }
    }
    entries_.push_back(std::move(assignment));
}

std::optional<ChannelAssignment> ChannelMap::find(std::string_view signal) const {
    const auto found = std::ranges::find(entries_, signal, &ChannelAssignment::signal);
    if (found == entries_.end()) {
        return std::nullopt;
    }
    return *found;
}

std::span<const ChannelAssignment> ChannelMap::entries() const noexcept {
    return entries_;
}

void ChannelMap::check_bindings(const signals::PinBindingTable& bindings) const {
    for (const signals::PinBinding& binding : bindings.entries()) {
        const std::optional<ChannelAssignment> assignment = find(binding.signal);
        if (!assignment.has_value()) {
            throw ControllerConfigError("bound signal " + binding.signal + " on pin " + binding.pin +
                                        " has no channel assignment");
        }
        if (!slot_accepts(assignment->slot, binding.mode)) {
            throw ControllerConfigError("bound signal " + binding.signal + " on pin " + binding.pin +
                                        " is assigned to " + slot_name(assignment->slot) +
                                        ", which does not accept its peripheral mode");
        }
    }
    for (const ChannelAssignment& assignment : entries_) {
        if (!bindings.find_signal(assignment.signal).has_value()) {
            throw ControllerConfigError("channel assignment names " + assignment.signal +
                                        ", which no pin binding holds");
        }
    }
}

void ChannelMap::assign_axis(std::size_t axis, AxisChannels channels) {
    if (axis >= stepper_channels) {
        throw ControllerConfigError("stepper axis " + std::to_string(axis) + " is beyond the " +
                                    std::to_string(stepper_channels) + " axes");
    }
    if (axes_.at(axis).has_value()) {
        throw ControllerConfigError("stepper axis " + std::to_string(axis) + " is assigned twice");
    }
    if (channels.dir_channel == channels.enable_channel) {
        throw ControllerConfigError("stepper axis " + std::to_string(axis) +
                                    ": direction and enable share digital_out channel " +
                                    std::to_string(channels.dir_channel));
    }
    for (const std::size_t channel : {channels.dir_channel, channels.enable_channel}) {
        const bool bound = std::ranges::any_of(entries_, [channel](const ChannelAssignment& entry) {
            return entry.slot == PinSlot::digital_out && entry.channel == channel;
        });
        if (!bound) {
            throw ControllerConfigError("stepper axis " + std::to_string(axis) + ": digital_out channel " +
                                        std::to_string(channel) + " carries no bound signal");
        }
    }
    axes_.at(axis) = channels;
}

std::optional<AxisChannels> ChannelMap::axis(std::size_t axis) const {
    if (axis >= stepper_channels) {
        return std::nullopt;
    }
    return axes_.at(axis);
}

void derive_axis_levels(const ChannelMap& map, PinOutputs& outputs) {
    for (std::size_t index = 0; index < stepper_channels; ++index) {
        const std::optional<AxisChannels> channels = map.axis(index);
        std::uint8_t direction = 0;
        std::uint8_t enable = 0;
        if (channels.has_value()) {
            direction = outputs.digital_out.at(channels->dir_channel);
            enable = outputs.digital_out.at(channels->enable_channel);
        }
        outputs.direction.at(index) = direction;
        outputs.enable.at(index) = enable;
    }
}

}  // namespace mechsim::controller
