#include "mechsim/signals/sensors.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mechsim::signals {

namespace {

constexpr double rate_slack = 1e-9;

void require_non_negative(double value, const std::string& what) {
    if (!std::isfinite(value) || value < 0.0) {
        throw SignalConfigError(what + " must be finite and non-negative");
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

std::int64_t wrap_index(std::int64_t index, std::int64_t depth) noexcept {
    return ((index % depth) + depth) % depth;
}

std::vector<FailureMode> sensor_modes(SensorOutputKind kind) {
    if (kind == SensorOutputKind::analog) {
        return {FailureMode::stuck, FailureMode::noisy, FailureMode::dropped};
    }
    return {FailureMode::stuck, FailureMode::noisy, FailureMode::dropped, FailureMode::forced_level};
}

}  // namespace

SampledSensor::SampledSensor(std::string instance, const std::vector<std::string>& outputs, SensorOutputKind kind,
                             std::size_t quantities, std::size_t frame, SensorFidelity fidelity)
    : BuiltinComponent(std::move(instance), {}, outputs, sensor_modes(kind)), kind_(kind), fidelity_(fidelity),
      quantities_(quantities, 0.0), frame_(frame, 0.0), frame_size_(frame) {
    const std::string name = "sensor '" + std::string(instance_name()) + "' ";
    require_non_negative(fidelity_.noise, name + "noise");
    require_non_negative(fidelity_.delay, name + "delay");
    require_non_negative(fidelity_.sample_rate, name + "sample_rate");
}

void SampledSensor::inject(const FailureInjection& injection) {
    if (injection.mode == FailureMode::forced_level) {
        if (kind_ == SensorOutputKind::analog) {
            throw SignalConfigError("sensor '" + std::string(instance_name()) +
                                    "' is analog: a forced_level fault has no level to present (SIG-036)");
        }
        if (kind_ == SensorOutputKind::digital && injection.forced_level != 0.0 && injection.forced_level != 1.0) {
            throw std::invalid_argument("a digital forced level must be 0 or 1");
        }
    }
    BuiltinComponent::inject(injection);
}

void SampledSensor::on_start() {
    const std::string name = "sensor '" + std::string(instance_name()) + "'";
    const double period = period_seconds();
    if (fidelity_.sample_rate * period > 1.0 + rate_slack) {
        throw SignalConfigError(name + " sample_rate " + std::to_string(fidelity_.sample_rate) +
                                " Hz exceeds the signal layer rate " + std::to_string(1.0 / period) + " Hz");
    }
    sample_period_ = 1;
    if (fidelity_.sample_rate > 0.0) {
        sample_period_ = std::max<std::int64_t>(1, periods_floor(1.0 / fidelity_.sample_rate, period));
    }
    delay_periods_ = periods_ceil(fidelity_.delay, period);
    if (delay_periods_ > max_delay_periods) {
        throw SignalConfigError(name + " delay " + std::to_string(fidelity_.delay) + " s is longer than " +
                                std::to_string(max_delay_periods) + " layer periods");
    }
    const std::span<const FailureInjection> faults = injections();
    for (std::size_t i = 0; i < faults.size(); ++i) {
        for (std::size_t j = i + 1; j < faults.size(); ++j) {
            if (faults[i].start < faults[j].end && faults[j].start < faults[i].end) {
                throw SignalConfigError(name + " has two fault injections with overlapping windows, [" +
                                        std::to_string(faults[i].start.count()) + ", " +
                                        std::to_string(faults[i].end.count()) + ") and [" +
                                        std::to_string(faults[j].start.count()) + ", " +
                                        std::to_string(faults[j].end.count()) + ") ticks");
            }
        }
    }
    std::fill(frame_.begin(), frame_.end(), 0.0);
    ring_.assign(static_cast<std::size_t>(delay_periods_ + 1) * frame_size_, 0.0);
    pushed_ = 0;
    on_sensor_start();
}

void SampledSensor::sample(const PhysicalState& state) {
    measure(state, quantities_);
    const std::int64_t step = sample_index();
    const std::optional<FailureInjection> fault = active_fault();
    bool due = step % sample_period_ == 0;
    if (fault.has_value() && fault->mode == FailureMode::dropped && kind_ != SensorOutputKind::counter) {
        due = false;
    }
    if (due) {
        for (double& quantity : quantities_) {
            if (fidelity_.noise > 0.0) {
                quantity += fidelity_.noise * rng().gaussian();
            }
            if (fault.has_value() && fault->mode == FailureMode::noisy && fault->noise_magnitude > 0.0) {
                quantity += fault->noise_magnitude * rng().gaussian();
            }
        }
        convert(quantities_, step, frame_);
    }
    const std::int64_t depth = delay_periods_ + 1;
    const auto width = static_cast<std::ptrdiff_t>(frame_size_);
    if (pushed_ == 0) {
        for (std::int64_t slot = 0; slot < depth; ++slot) {
            std::copy(frame_.begin(), frame_.end(), ring_.begin() + (static_cast<std::ptrdiff_t>(slot) * width));
        }
    }
    const auto slot = static_cast<std::ptrdiff_t>(wrap_index(step, depth));
    std::copy(frame_.begin(), frame_.end(), ring_.begin() + (slot * width));
    ++pushed_;
}

void SampledSensor::latch_outputs() {
    const std::int64_t depth = delay_periods_ + 1;
    const std::int64_t slot = wrap_index(sample_index() - delay_periods_, depth);
    const std::span<const double> delivered =
        std::span<const double>(ring_).subspan(static_cast<std::size_t>(slot) * frame_size_, frame_size_);
    present(delivered, active_fault());
}

void SampledSensor::present(std::span<const double> delivered, const std::optional<FailureInjection>& fault) {
    if (holds(fault)) {
        return;
    }
    if (fault.has_value() && fault->mode == FailureMode::forced_level) {
        for (std::size_t i = 0; i < outputs().size(); ++i) {
            set_output(i, fault->forced_level);
        }
        return;
    }
    for (std::size_t i = 0; i < delivered.size(); ++i) {
        set_output(i, delivered[i]);
    }
}

bool SampledSensor::holds(const std::optional<FailureInjection>& fault) noexcept {
    return fault.has_value() && (fault->mode == FailureMode::stuck || fault->mode == FailureMode::dropped);
}

std::int64_t SampledSensor::sample_period() const noexcept {
    return sample_period_;
}

std::int64_t SampledSensor::delay_periods() const noexcept {
    return delay_periods_;
}

LimitSwitch::LimitSwitch(std::string instance, LimitSwitchParams params)
    : SampledSensor(std::move(instance), {"out"}, SensorOutputKind::digital, 1, 1, params.fidelity),
      params_(params), reader_(params_.coordinate) {
    if (!std::isfinite(params_.position)) {
        throw SignalConfigError("limit switch position must be finite");
    }
    require_non_negative(params_.hysteresis, "limit switch hysteresis");
    require_non_negative(params_.resolution, "limit switch resolution");
    require_non_negative(params_.debounce_time, "limit switch debounce time");
}

void LimitSwitch::on_sensor_start() {
    debounce_periods_ = periods_ceil(params_.debounce_time, period_seconds());
    primed_ = false;
    pending_ = false;
}

void LimitSwitch::measure(const PhysicalState& state, std::span<double> quantities) {
    quantities[0] = reader_.read(state);
}

void LimitSwitch::convert(std::span<const double> quantities, std::int64_t step, std::span<double> frame) {
    double x = quantities[0];
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
    } else {
        if (pending_ && step - pending_since_ >= debounce_periods_) {
            debounced_ = !debounced_;
            pending_ = false;
        }
        if (actuated_ == debounced_) {
            pending_ = false;
        } else if (!pending_) {
            pending_ = true;
            pending_since_ = step;
        }
        if (pending_ && debounce_periods_ == 0) {
            debounced_ = actuated_;
            pending_ = false;
        }
    }
    bool closed = debounced_;
    if (!params_.normally_open) {
        closed = !debounced_;
    }
    frame[0] = static_cast<double>(closed);
}

IncrementalEncoder::IncrementalEncoder(std::string instance, EncoderParams params)
    : SampledSensor(std::move(instance), {"channel_a", "channel_b"}, SensorOutputKind::counter, 1, 1,
                    params.fidelity),
      params_(params), reader_(params_.coordinate) {
    if (!std::isfinite(params_.resolution) || params_.resolution <= 0.0) {
        throw SignalConfigError("encoder resolution must be finite and positive");
    }
    if (params_.coordinate.kind != CoordinateKind::angular) {
        throw SignalConfigError("an incremental encoder reads an angular coordinate");
    }
}

void IncrementalEncoder::on_sensor_start() {
    primed_ = false;
    last_count_ = 0;
}

void IncrementalEncoder::measure(const PhysicalState& state, std::span<double> quantities) {
    quantities[0] = reader_.read(state);
}

void IncrementalEncoder::convert(std::span<const double> quantities, std::int64_t /*step*/, std::span<double> frame) {
    double angle = quantities[0];
    if (params_.reversed) {
        angle = -angle;
    }
    frame[0] = std::floor(angle / params_.resolution);
}

void IncrementalEncoder::present(std::span<const double> delivered, const std::optional<FailureInjection>& fault) {
    const auto count = static_cast<std::int64_t>(delivered[0]);
    if (!primed_) {
        primed_ = true;
        last_count_ = count;
    }
    const std::int64_t low = std::min(last_count_, count);
    const std::int64_t high = std::max(last_count_, count);
    std::int64_t a_edges = 0;
    std::int64_t b_edges = 0;
    if (high > low) {
        a_edges = evens_between(low, high - 1);
        b_edges = (high - low) - a_edges;
    }
    if (count < last_count_) {
        a_edges = -a_edges;
        b_edges = -b_edges;
    }
    last_count_ = count;
    if (holds(fault)) {
        a_edges = 0;
        b_edges = 0;
    } else if (fault.has_value() && fault->mode == FailureMode::forced_level) {
        a_edges = std::llround(fault->forced_level);
        b_edges = 0;
    }
    set_output(0, static_cast<double>(a_edges));
    set_output(1, static_cast<double>(b_edges));
}

}  // namespace mechsim::signals
