#include "mechsim/controller/controller_party.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
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
#include "mechsim/kernel/time.hpp"
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

namespace {

const std::string fault_source = "kernel";
const std::string fault_kind = "fault";

void check_overlap(const std::vector<kernel::SimTime>& starts, const std::vector<kernel::SimTime>& ends,
                   const std::string& what) {
    for (std::size_t i = 0; i < starts.size(); ++i) {
        for (std::size_t j = i + 1; j < starts.size(); ++j) {
            if (starts[i] < ends[j] && starts[j] < ends[i]) {
                throw kernel::KernelConfigError(what + " windows [" + std::to_string(starts[i].count()) + ", " +
                                                std::to_string(ends[i].count()) + ") and [" +
                                                std::to_string(starts[j].count()) + ", " +
                                                std::to_string(ends[j].count()) + ") overlap");
            }
        }
    }
}

}  // namespace

void ControllerParty::check_schedulable(kernel::SimTime start, kernel::SimTime end) const {
    if (started_) {
        throw kernel::KernelConfigError("controller party '" + config_.name + "': faults are scheduled before start()");
    }
    if (start.count() < 0 || end <= start) {
        throw kernel::KernelConfigError("controller party '" + config_.name + "': fault window [" +
                                        std::to_string(start.count()) + ", " + std::to_string(end.count()) +
                                        ") must start at or after 0 and end after it starts");
    }
}

void ControllerParty::schedule_freeze(kernel::SimTime start, kernel::SimTime end) {
    check_schedulable(start, end);
    freezes_.push_back(Window{start, end});
}

void ControllerParty::schedule_slowdown(kernel::SimTime start, kernel::SimTime end, std::int64_t factor) {
    check_schedulable(start, end);
    if (factor < 2) {
        throw kernel::KernelConfigError("controller party '" + config_.name + "': slow-down factor " +
                                        std::to_string(factor) + " must be an integer of at least 2");
    }
    slowdowns_.push_back(Window{start, end, factor});
}

void ControllerParty::start(const kernel::Rate& /*base_rate*/) {
    const auto by_start = [](const Window& a, const Window& b) { return a.start < b.start; };
    std::sort(freezes_.begin(), freezes_.end(), by_start);
    std::sort(slowdowns_.begin(), slowdowns_.end(), by_start);
    const auto check = [this](const std::vector<Window>& windows, const std::string& what) {
        std::vector<kernel::SimTime> starts;
        std::vector<kernel::SimTime> ends;
        for (const Window& window : windows) {
            starts.push_back(window.start);
            ends.push_back(window.end);
        }
        check_overlap(starts, ends, "controller party '" + config_.name + "': " + what);
    };
    check(freezes_, "freeze");
    check(slowdowns_, "slow-down");
    started_ = true;
}

bool ControllerParty::advance(Window& window, kernel::SimTime t, const std::string& what) {
    if (!window.started && t >= window.start) {
        if (t >= window.end) {
            throw kernel::KernelConfigError("controller party '" + config_.name + "': " + what + " window [" +
                                            std::to_string(window.start.count()) + ", " +
                                            std::to_string(window.end.count()) + ") holds none of its steps");
        }
        window.started = true;
        std::string detail = what + " start " + config_.name;
        if (window.factor > 1) {
            detail += " factor " + std::to_string(window.factor);
        }
        pending_events_.push_back(FaultEvent{t, detail});
    }
    if (window.started && !window.ended && t >= window.end) {
        window.ended = true;
        pending_events_.push_back(FaultEvent{t, what + " end " + config_.name});
    }
    return window.started && !window.ended;
}

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

void ControllerParty::step(kernel::SimTime t, kernel::Duration /*dt*/) {
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
    bool frozen = false;
    for (Window& window : freezes_) {
        if (advance(window, t, "freeze")) {
            frozen = true;
        }
    }
    if (frozen != frozen_) {
        runtime_->freeze(frozen);
        frozen_ = frozen;
    }
    bool tick = true;
    for (Window& window : slowdowns_) {
        if (advance(window, t, "slowdown")) {
            tick = window.steps % window.factor == 0;
            ++window.steps;
        }
    }
    if (tick) {
        runtime_->tick(inputs_, produced_);
    } else {
        produced_ = held_levels(produced_);
    }
    publish(produced_);
}

void ControllerParty::publish(const PinOutputs& produced) {
    if (config_.latency_ticks == 0) {
        outputs_ = produced;
        return;
    }
    delayed_.push_back(produced);
    if (delayed_.size() > config_.latency_ticks) {
        outputs_ = delayed_.front();
        delayed_.pop_front();
    }
}

void ControllerParty::write_outputs(kernel::CouplingWriter& outputs) {
    outputs.set(published_[0], kernel::make_payload(outputs_, pins::pin_outputs_kind));
}

void ControllerParty::on_fault(const kernel::PartyFault& /*fault*/) noexcept {}

void ControllerParty::record(kernel::TickRecord& row) const {
    row.controller_state = std::to_string(outputs_.state_code);
    for (const FaultEvent& event : pending_events_) {
        row.events.push_back(kernel::RecordedEvent{event.time, fault_source, fault_kind, event.detail});
    }
    pending_events_.clear();
}

ControllerRuntime& ControllerParty::runtime() noexcept {
    return *runtime_;
}

const PinOutputs& ControllerParty::pin_outputs() const noexcept {
    return outputs_;
}

}  // namespace mechsim::controller
