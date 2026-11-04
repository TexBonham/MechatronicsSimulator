#include "mechsim/controller/controller_party.hpp"

#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/pins/pins.hpp"

namespace mechsim::controller {

ControllerParty::ControllerParty(std::unique_ptr<ControllerRuntime> runtime, ControllerPartyConfig config)
    : runtime_(std::move(runtime)), config_(std::move(config)) {
    if (runtime_ == nullptr) {
        throw std::invalid_argument("controller party: null runtime");
    }
    published_ = {config_.name + ".pin_outputs"};
    consumed_ = {config_.pin_inputs};
    if (!config_.rail.empty()) {
        consumed_.push_back(config_.rails);
    }
}

std::string_view ControllerParty::name() const {
    return config_.name;
}

kernel::Rate ControllerParty::nominal_rate() const {
    return config_.rate;
}

kernel::PartyRole ControllerParty::role() const {
    return kernel::PartyRole::controller;
}

std::span<const std::string> ControllerParty::published() const {
    return published_;
}

std::span<const std::string> ControllerParty::consumed() const {
    return consumed_;
}

void ControllerParty::start(const kernel::Rate& /*base_rate*/) {}

void ControllerParty::read_inputs(const kernel::CouplingReader& inputs) {
    if (inputs.has_value(config_.pin_inputs)) {
        inputs_ = inputs.payload<PinInputs>(config_.pin_inputs);
    }
    if (config_.rail.empty() || !inputs.has_value(config_.rails)) {
        return;
    }
    rail_.reset();
    for (const pins::PowerRail& rail : inputs.payload<std::vector<pins::PowerRail>>(config_.rails)) {
        if (rail.name == config_.rail) {
            rail_ = rail;
        }
    }
    if (!rail_.has_value()) {
        throw ControllerError("controller rail '" + config_.rail + "' is not among '" + config_.rails + "'");
    }
}

void ControllerParty::step(kernel::SimTime /*t*/, kernel::Duration /*dt*/) {
    if (rail_.has_value()) {
        double voltage = 0.0;
        if (rail_->on) {
            voltage = rail_->voltage;
        }
        if (!applied_voltage_.has_value()) {
            runtime_->reset(*rail_);
        } else if (*applied_voltage_ != voltage) {
            runtime_->brownout(voltage);
        }
        applied_voltage_ = voltage;
    }
    runtime_->tick(inputs_, outputs_);
}

void ControllerParty::write_outputs(kernel::CouplingWriter& outputs) {
    outputs.set(published_[0], kernel::make_payload(outputs_, pins::pin_outputs_kind));
}

void ControllerParty::on_fault(const kernel::PartyFault& /*fault*/) noexcept {}

void ControllerParty::record(kernel::TickRecord& row) const {
    row.controller_state = std::to_string(outputs_.state_code);
}

ControllerRuntime& ControllerParty::runtime() noexcept {
    return *runtime_;
}

const PinOutputs& ControllerParty::pin_outputs() const noexcept {
    return outputs_;
}

}  // namespace mechsim::controller
