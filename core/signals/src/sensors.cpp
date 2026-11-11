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

void require_positive(double value, const std::string& what) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw SignalConfigError(what + " must be finite and positive");
    }
}

double quantize(double value, double resolution) {
    if (resolution > 0.0) {
        return resolution * std::round(value / resolution);
    }
    return value;
}

// v in the body frame: R^T v with R the unit body-to-world quaternion (w first).
kernel::Vec3 to_body(const kernel::Quat& q, const kernel::Vec3& v) noexcept {
    const double w = q[0];
    const kernel::Vec3 u{-q[1], -q[2], -q[3]};
    const kernel::Vec3 uv{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
    const kernel::Vec3 t{2.0 * uv[0], 2.0 * uv[1], 2.0 * uv[2]};
    const kernel::Vec3 ut{u[1] * t[2] - u[2] * t[1], u[2] * t[0] - u[0] * t[2], u[0] * t[1] - u[1] * t[0]};
    return {v[0] + w * t[0] + ut[0], v[1] + w * t[1] + ut[1], v[2] + w * t[2] + ut[2]};
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
        for (std::size_t q = 0; q < quantities_.size(); ++q) {
            double& quantity = quantities_[q];
            const double sd = part_noise(q);
            if (sd > 0.0) {
                quantity += sd * rng().gaussian();
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

double SampledSensor::part_noise(std::size_t /*quantity*/) const {
    return fidelity_.noise;
}

const SensorFidelity& SampledSensor::fidelity() const noexcept {
    return fidelity_;
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

LoadCell::LoadCell(std::string instance, LoadCellParams params)
    : SampledSensor(std::move(instance), {"signal"}, SensorOutputKind::analog, 1, 1, params.fidelity),
      params_(params) {
    const std::string name = "load cell '" + std::string(instance_name()) + "' ";
    if (params_.payload.has_value() == params_.joint.has_value()) {
        throw SignalConfigError(name + "senses exactly one of a payload and a joint");
    }
    require_positive(params_.capacity, name + "capacity");
    require_positive(params_.sensitivity, name + "sensitivity");
    require_positive(params_.excitation_voltage, name + "excitation voltage");
    require_positive(params_.overload_limit, name + "overload limit");
    require_non_negative(params_.resolution, name + "resolution");
    require_non_negative(params_.nonlinearity, name + "nonlinearity");
    if (params_.nonlinearity > 1.0) {
        throw SignalConfigError(name + "nonlinearity must be at most 1");
    }
}

void LoadCell::on_sensor_start() {
    noted_ = false;
}

void LoadCell::measure(const PhysicalState& state, std::span<double> quantities) {
    if (params_.payload.has_value()) {
        const std::size_t index = *params_.payload;
        if (index >= state.payloads.size()) {
            throw SignalRunError("load cell '" + std::string(instance_name()) + "': payload index " +
                                 std::to_string(index) + " is outside the published state");
        }
        const kernel::Vec3& f = state.payloads[index].force;
        quantities[0] = std::sqrt((f[0] * f[0]) + (f[1] * f[1]) + (f[2] * f[2]));
        return;
    }
    if (!params_.joint.has_value()) {
        throw SignalRunError("load cell '" + std::string(instance_name()) + "': senses neither a payload nor a joint");
    }
    const std::size_t index = *params_.joint;
    if (index >= state.joints.size() || state.joints[index].generalized_force.empty()) {
        throw SignalRunError("load cell '" + std::string(instance_name()) + "': joint index " + std::to_string(index) +
                             " has no published generalized force");
    }
    quantities[0] = state.joints[index].generalized_force[0];
}

void LoadCell::convert(std::span<const double> quantities, std::int64_t step, std::span<double> frame) {
    double force = quantize(quantities[0], params_.resolution);
    if (std::fabs(quantities[0]) > params_.overload_limit) {
        force = std::copysign(params_.overload_limit, quantities[0]);
        if (!noted_) {
            noted_ = true;
            note("load cell '" + std::string(instance_name()) + "' saturated at its overload limit " +
                 std::to_string(params_.overload_limit) + " N (first at layer step " + std::to_string(step) +
                 "); the output is held at the limit, not modelled beyond it");
        }
    }
    const double full_scale = params_.sensitivity * params_.excitation_voltage;
    const double ratio = force / params_.capacity;
    frame[0] = (full_scale * ratio) + (params_.nonlinearity * ratio * ratio * full_scale);
}

Imu::Imu(std::string instance, ImuParams params)
    : SampledSensor(std::move(instance), {"accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"},
                    SensorOutputKind::analog, 6, 6, params.fidelity),
      params_(params) {
    const std::string name = "imu '" + std::string(instance_name()) + "' ";
    require_positive(params_.accel_range, name + "accel range");
    require_positive(params_.gyro_range, name + "gyro range");
    require_non_negative(params_.resolution, name + "resolution");
    require_non_negative(params_.gyro_resolution, name + "gyro resolution");
    require_non_negative(params_.gyro_noise, name + "gyro noise");
    for (const double g : params_.gravity) {
        if (!std::isfinite(g)) {
            throw SignalConfigError(name + "gravity must be finite");
        }
    }
}

void Imu::on_sensor_start() {
    primed_ = false;
    last_velocity_ = {};
}

double Imu::part_noise(std::size_t quantity) const {
    if (quantity < 3) {
        return fidelity().noise;
    }
    return params_.gyro_noise;
}

void Imu::measure(const PhysicalState& state, std::span<double> quantities) {
    if (params_.body >= state.bodies.size()) {
        throw SignalRunError("imu '" + std::string(instance_name()) + "': body index " + std::to_string(params_.body) +
                             " is outside the published state");
    }
    const kernel::BodyState& body = state.bodies[params_.body];
    if (!primed_) {
        primed_ = true;
        last_velocity_ = body.linear_velocity;
    }
    const double h = period_seconds();
    kernel::Vec3 relative{};
    for (std::size_t i = 0; i < 3; ++i) {
        const double a = (body.linear_velocity[i] - last_velocity_[i]) / h;
        relative[i] = a - params_.gravity[i];
    }
    last_velocity_ = body.linear_velocity;
    const kernel::Vec3 specific = to_body(body.orientation, relative);
    const kernel::Vec3 rate = to_body(body.orientation, body.angular_velocity);
    for (std::size_t i = 0; i < 3; ++i) {
        quantities[i] = specific[i];
        quantities[i + 3] = rate[i];
    }
}

void Imu::convert(std::span<const double> quantities, std::int64_t /*step*/, std::span<double> frame) {
    for (std::size_t i = 0; i < 3; ++i) {
        frame[i] = quantize(std::clamp(quantities[i], -params_.accel_range, params_.accel_range), params_.resolution);
        frame[i + 3] = quantize(std::clamp(quantities[i + 3], -params_.gyro_range, params_.gyro_range),
                                params_.gyro_resolution);
    }
}

}  // namespace mechsim::signals
