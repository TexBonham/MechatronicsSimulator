#include "mechsim/signals/sensors.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace mechsim::signals {

namespace {

void require_non_negative(double value, const char* what) {
    if (!std::isfinite(value) || value < 0.0) {
        throw SignalConfigError(std::string(what) + " must be finite and non-negative");
    }
}

std::int64_t floor_div2(std::int64_t value) noexcept {
    std::int64_t quotient = value / 2;
    if (value % 2 != 0 && value < 0) {
        --quotient;
    }
    return quotient;
}

// Even integers k with low <= k <= high.
std::int64_t evens_between(std::int64_t low, std::int64_t high) noexcept {
    return floor_div2(high) - floor_div2(low - 1);
}

}  // namespace

LimitSwitch::LimitSwitch(std::string instance, LimitSwitchParams params)
    : BuiltinComponent(std::move(instance), {}, {"out"},
                       {FailureMode::stuck, FailureMode::noisy, FailureMode::dropped, FailureMode::forced_level}),
      params_(params), reader_(params_.coordinate) {
    if (!std::isfinite(params_.position)) {
        throw SignalConfigError("limit switch position must be finite");
    }
    require_non_negative(params_.hysteresis, "limit switch hysteresis");
    require_non_negative(params_.resolution, "limit switch resolution");
    require_non_negative(params_.debounce_time, "limit switch debounce time");
}

void LimitSwitch::inject(const FailureInjection& injection) {
    if (injection.mode == FailureMode::forced_level && injection.forced_level != 0.0 &&
        injection.forced_level != 1.0) {
        throw std::invalid_argument("a digital forced level must be 0 or 1");
    }
    BuiltinComponent::inject(injection);
}

void LimitSwitch::on_start() {
    debounce_periods_ = periods_ceil(params_.debounce_time, period_seconds());
    primed_ = false;
    pending_ = false;
}

void LimitSwitch::sample(const PhysicalState& state) {
    if (fault_active(FailureMode::dropped)) {
        return;
    }
    double x = reader_.read(state);
    const std::optional<FailureInjection> fault = active_fault();
    if (fault.has_value() && fault->mode == FailureMode::noisy) {
        x += fault->noise_magnitude * rng().gaussian();
    }
    if (params_.resolution > 0.0) {
        x = params_.resolution * std::floor(x / params_.resolution);
    }
    double toward = x - params_.position;
    if (!params_.actuate_above) {
        toward = -toward;
    }
    if (!primed_) {
        actuated_ = toward >= 0.0;
    } else if (!actuated_ && toward >= 0.0) {
        actuated_ = true;
    } else if (actuated_ && toward < -params_.hysteresis) {
        actuated_ = false;
    }
    if (!primed_) {
        primed_ = true;
        debounced_ = actuated_;
        return;
    }
    if (actuated_ == debounced_) {
        pending_ = false;
        return;
    }
    if (!pending_) {
        pending_ = true;
        pending_since_ = sample_index();
    }
    if (sample_index() - pending_since_ >= debounce_periods_) {
        debounced_ = actuated_;
        pending_ = false;
    }
}

void LimitSwitch::latch_outputs() {
    const std::optional<FailureInjection> fault = active_fault();
    if (fault.has_value() && fault->mode == FailureMode::stuck) {
        return;
    }
    if (fault.has_value() && fault->mode == FailureMode::forced_level) {
        set_output(0, fault->forced_level);
        return;
    }
    bool closed = debounced_;
    if (!params_.normally_open) {
        closed = !debounced_;
    }
    set_output(0, static_cast<double>(closed));
}

IncrementalEncoder::IncrementalEncoder(std::string instance, EncoderParams params)
    : BuiltinComponent(std::move(instance), {}, {"channel_a", "channel_b"},
                       {FailureMode::stuck, FailureMode::noisy, FailureMode::dropped, FailureMode::forced_level}),
      params_(params), reader_(params_.coordinate) {
    if (!std::isfinite(params_.resolution) || params_.resolution <= 0.0) {
        throw SignalConfigError("encoder resolution must be finite and positive");
    }
    if (params_.coordinate.kind != CoordinateKind::angular) {
        throw SignalConfigError("an incremental encoder reads an angular coordinate");
    }
}

void IncrementalEncoder::sample(const PhysicalState& state) {
    double angle = reader_.read(state);
    if (params_.reversed) {
        angle = -angle;
    }
    count_ = static_cast<std::int64_t>(std::floor(angle / params_.resolution));
    if (!primed_) {
        primed_ = true;
        last_count_ = count_;
    }
}

void IncrementalEncoder::latch_outputs() {
    const std::int64_t low = std::min(last_count_, count_);
    const std::int64_t high = std::max(last_count_, count_);
    std::int64_t a_edges = 0;
    std::int64_t b_edges = 0;
    if (high > low) {
        a_edges = evens_between(low, high - 1);
        b_edges = (high - low) - a_edges;
    }
    if (count_ < last_count_) {
        a_edges = -a_edges;
        b_edges = -b_edges;
    }
    last_count_ = count_;
    const std::optional<FailureInjection> fault = active_fault();
    if (fault.has_value()) {
        if (fault->mode == FailureMode::stuck || fault->mode == FailureMode::dropped) {
            a_edges = 0;
            b_edges = 0;
        } else if (fault->mode == FailureMode::noisy) {
            a_edges += std::llround(fault->noise_magnitude * rng().gaussian());
        } else if (fault->mode == FailureMode::forced_level) {
            a_edges = std::llround(fault->forced_level);
            b_edges = 0;
        }
    }
    set_output(0, static_cast<double>(a_edges));
    set_output(1, static_cast<double>(b_edges));
}

}  // namespace mechsim::signals
