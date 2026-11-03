#include "mechsim/signals/signal_component.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/signals/signal_name.hpp"

namespace mechsim::signals {

void validate_injection(const FailureInjection& injection) {
    if (injection.end <= injection.start) {
        throw std::invalid_argument("failure window must end after it starts");
    }
    if (!std::isfinite(injection.noise_magnitude) || injection.noise_magnitude < 0.0) {
        throw std::invalid_argument("noise magnitude must be finite and non-negative");
    }
    if (!std::isfinite(injection.forced_level)) {
        throw std::invalid_argument("forced level must be finite");
    }
}

namespace {

void check_names(std::string_view instance, std::span<const std::string> names) {
    for (const std::string& name : names) {
        if (!is_dotted_signal_name(name)) {
            throw SignalNameError("signal '" + name + "' is not a dotted instance.signal name");
        }
        if (signal_instance(name) != instance) {
            throw SignalNameError("signal '" + name + "' does not belong to instance '" + std::string(instance) + "'");
        }
    }
}

}  // namespace

void SignalTable::add(const SignalComponent& component) {
    const std::string_view instance = component.instance_name();
    check_names(instance, component.inputs());
    check_names(instance, component.outputs());
    std::vector<std::string> added;
    for (const auto names : {component.inputs(), component.outputs()}) {
        for (const std::string& name : names) {
            const bool seen_here = std::find(added.begin(), added.end(), name) != added.end();
            if (seen_here || owner_by_signal_.contains(name)) {
                throw SignalNameError("signal '" + name + "' is not unique in the table");
            }
            added.push_back(name);
        }
    }
    for (std::string& name : added) {
        owner_by_signal_.emplace(std::move(name), std::string(instance));
    }
}

bool SignalTable::contains(std::string_view signal) const {
    return owner_by_signal_.find(signal) != owner_by_signal_.end();
}

std::size_t SignalTable::size() const noexcept {
    return owner_by_signal_.size();
}

NullSignalComponent::NullSignalComponent(std::string instance, std::vector<std::string> inputs,
                                         std::vector<std::string> outputs)
    : instance_(std::move(instance)), inputs_(std::move(inputs)), outputs_(std::move(outputs)) {
    if (!is_instance_name(instance_)) {
        throw SignalNameError("instance name '" + instance_ + "' is not a single identifier");
    }
    check_names(instance_, inputs_);
    check_names(instance_, outputs_);
    input_values_.assign(inputs_.size(), 0.0);
    output_values_.assign(outputs_.size(), 0.0);
}

std::string_view NullSignalComponent::instance_name() const {
    return instance_;
}

std::span<const std::string> NullSignalComponent::inputs() const {
    return inputs_;
}

std::span<const std::string> NullSignalComponent::outputs() const {
    return outputs_;
}

void NullSignalComponent::sample(const PhysicalState& state) {
    ++samples_;
    last_sample_step_ = state.steps;
}

void NullSignalComponent::set_inputs(std::span<const double> values) {
    if (values.size() != inputs_.size()) {
        throw std::invalid_argument("instance '" + instance_ + "' takes " + std::to_string(inputs_.size()) +
                                    " inputs, got " + std::to_string(values.size()));
    }
    input_values_.assign(values.begin(), values.end());
}

void NullSignalComponent::latch_outputs() {
    ++latches_;
}

std::span<const double> NullSignalComponent::output_values() const {
    return output_values_;
}

void NullSignalComponent::inject(const FailureInjection& injection) {
    validate_injection(injection);
    injections_.push_back(injection);
}

std::size_t NullSignalComponent::samples() const noexcept {
    return samples_;
}

std::size_t NullSignalComponent::latches() const noexcept {
    return latches_;
}

std::int64_t NullSignalComponent::last_sample_step() const noexcept {
    return last_sample_step_;
}

std::span<const FailureInjection> NullSignalComponent::injections() const noexcept {
    return injections_;
}

}  // namespace mechsim::signals
