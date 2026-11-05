#include "mechsim/signals/safety.hpp"

#include <cmath>
#include <string>
#include <utility>

namespace mechsim::signals {

namespace {

bool level(double value) noexcept {
    return value >= 0.5;
}

void require_time(double seconds, const char* what) {
    if (!std::isfinite(seconds) || seconds < 0.0) {
        throw SignalConfigError(std::string(what) + " must be finite and non-negative");
    }
}

}  // namespace

PowerSupply::PowerSupply(std::string instance, double output_voltage)
    : BuiltinComponent(std::move(instance), {}, {"voltage"}, {}), voltage_(output_voltage) {
    if (!std::isfinite(voltage_) || voltage_ < 0.0) {
        throw SignalConfigError("supply voltage must be finite and non-negative");
    }
}

void PowerSupply::sample(const PhysicalState& /*state*/) {}

void PowerSupply::latch_outputs() {
    set_output(0, voltage_);
}

EStopRelay::EStopRelay(std::string instance, EStopRelayParams params)
    : BuiltinComponent(std::move(instance), {"estop_loop", "reset"}, {"status", "contact_closed"}, {}),
      params_(params) {
    require_time(params_.cutoff_delay, "relay cutoff delay");
}

void EStopRelay::on_start() {
    cutoff_periods_ = periods_floor(params_.cutoff_delay, period_seconds());
    pressed_at_.reset();
    closed_ = true;
}

void EStopRelay::sample(const PhysicalState& /*state*/) {}

void EStopRelay::latch_outputs() {
    const bool loop_closed = level(input(0));
    const bool was_open = !closed_;
    if (!loop_closed && closed_ && !pressed_at_.has_value()) {
        pressed_at_ = sample_index();
    }
    if (pressed_at_.has_value() && sample_index() - *pressed_at_ >= cutoff_periods_) {
        closed_ = false;
        pressed_at_.reset();
    }
    if (was_open && !pressed_at_.has_value() && loop_closed) {
        if (!params_.manual_reset || level(input(1))) {
            closed_ = true;
        }
    }
    set_output(0, static_cast<double>(closed_));
    set_output(1, static_cast<double>(closed_));
}

Watchdog::Watchdog(std::string instance, WatchdogParams params)
    : BuiltinComponent(std::move(instance), {"heartbeat"}, {"tripped", "contact_closed"}, {}), params_(params) {
    require_time(params_.timeout, "watchdog timeout");
    require_time(params_.cutoff_delay, "watchdog cutoff delay");
}

void Watchdog::on_start() {
    timeout_periods_ = periods_floor(params_.timeout, period_seconds());
    cutoff_periods_ = periods_floor(params_.cutoff_delay, period_seconds());
    primed_ = false;
    tripped_at_.reset();
    closed_ = true;
}

void Watchdog::sample(const PhysicalState& /*state*/) {}

void Watchdog::latch_outputs() {
    const bool beat = level(input(0));
    const std::int64_t j = sample_index();
    bool edge = false;
    if (!primed_) {
        primed_ = true;
        last_edge_ = j;
    } else if (beat != last_level_) {
        edge = true;
        last_edge_ = j;
    }
    last_level_ = beat;
    if (tripped_at_.has_value() && edge && !params_.manual_reset) {
        tripped_at_.reset();
        closed_ = true;
    }
    if (!tripped_at_.has_value() && j - last_edge_ > timeout_periods_) {
        tripped_at_ = j;
    }
    if (tripped_at_.has_value() && j - *tripped_at_ >= cutoff_periods_) {
        closed_ = false;
    }
    set_output(0, static_cast<double>(tripped_at_.has_value()));
    set_output(1, static_cast<double>(closed_));
}

}  // namespace mechsim::signals
