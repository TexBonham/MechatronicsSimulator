#include "mechsim/signals/safety.hpp"

#include <cmath>
#include <cstddef>
#include <span>
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
    : BuiltinComponent(std::move(instance), {}, {"voltage"}, {FailureMode::power_loss}), voltage_(output_voltage) {
    if (!std::isfinite(voltage_) || voltage_ < 0.0) {
        throw SignalConfigError("supply voltage must be finite and non-negative");
    }
}

void PowerSupply::sample(const PhysicalState& /*state*/) {}

void PowerSupply::latch_outputs() {
    if (fault_active(FailureMode::power_loss)) {
        set_output(0, 0.0);
        return;
    }
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

Brake::Brake(std::string instance, BrakeParams params)
    : BuiltinComponent(std::move(instance), {"release", "vcc"}, {"engaged", "torque"}, {}), params_(params) {
    if (!std::isfinite(params_.brake_torque) || params_.brake_torque <= 0.0) {
        throw SignalConfigError("brake torque must be finite and positive");
    }
    require_time(params_.engage_delay, "brake engage delay");
    require_time(params_.release_delay, "brake release delay");
    if (!std::isfinite(params_.damping) || params_.damping < 0.0) {
        throw SignalConfigError("brake damping must be finite and non-negative");
    }
    if (params_.inverse_inertia.has_value()) {
        const double a0 = *params_.inverse_inertia;
        if (!std::isfinite(a0) || a0 <= 0.0) {
            throw SignalConfigError("brake joint inverse inertia must be finite and positive");
        }
    }
}

void Brake::on_start() {
    const double h = period_seconds();
    stiffness_ = params_.brake_torque / brake_default_compliance;
    if (params_.inverse_inertia.has_value()) {
        stiffness_ = brake_envelope_product / (*params_.inverse_inertia * h * h);
    }
    engage_periods_ = periods_floor(params_.engage_delay, h);
    release_periods_ = periods_floor(params_.release_delay, h);
    primed_ = false;
    engaged_ = false;
    slipping_ = false;
    pending_since_.reset();
    readings_ = 0;
    torque_ = 0.0;
    drive_.clear();
}

void Brake::sample(const PhysicalState& state) {
    if (params_.joint >= state.joint_positions.size()) {
        throw SignalRunError("brake '" + std::string(instance_name()) + "': joint index " +
                             std::to_string(params_.joint) + " is outside the published joint positions");
    }
    const double h = period_seconds();
    const double theta = state.joint_positions[params_.joint];
    double omega = 0.0;
    if (readings_ >= 1) {
        omega = (theta - theta_) / h;
    }
    alpha_ = 0.0;
    if (readings_ >= 2) {
        alpha_ = (omega - omega_) / h;
    }
    theta_ = theta;
    omega_ = omega;
    ++readings_;
    torque_ = 0.0;
    for (const kernel::JointDriveTorque& applied : state.joint_drive_torques) {
        if (applied.joint == params_.joint) {
            torque_ = applied.torque;
        }
    }
}

void Brake::latch_outputs() {
    const double h = period_seconds();
    const bool powered = input(1) > 0.0;
    const bool coil = powered && level(input(0));
    bool demand = !coil;
    if (!params_.power_off_engaged) {
        demand = powered && !level(input(0));
    }
    const bool was_engaged = engaged_ && primed_;
    if (!primed_) {
        primed_ = true;
        engaged_ = demand;
    } else if (demand == engaged_) {
        pending_since_.reset();
    } else {
        if (!pending_since_.has_value()) {
            pending_since_ = sample_index();
        }
        std::int64_t delay = release_periods_;
        if (demand) {
            delay = engage_periods_;
        }
        if (sample_index() - *pending_since_ >= delay) {
            engaged_ = demand;
            pending_since_.reset();
        }
    }
    const double theta_p = theta_ + (omega_ * h) + (alpha_ * h * h);
    drive_.clear();
    if (!engaged_) {
        slipping_ = false;
        set_output(0, 0.0);
        set_output(1, 0.0);
        return;
    }
    double rest = 0.0;
    if (!was_engaged) {
        slipping_ = false;
        rest = theta_;
        torque_ = 0.0;
    } else {
        rest = drive_rest_;
        if (!slipping_ && std::fabs(torque_) > params_.brake_torque) {
            slipping_ = true;
            slip_sign_ = std::copysign(1.0, torque_);
        }
        if (slipping_) {
            const double velocity = omega_ + (alpha_ * h);
            if (velocity * slip_sign_ >= 0.0) {
                slipping_ = false;
                rest = theta_p;
            } else {
                rest = theta_p + (slip_sign_ * params_.brake_torque / stiffness_);
            }
        }
    }
    drive_rest_ = rest;
    drive_.push_back(kernel::JointDrive{params_.joint, rest, stiffness_, params_.damping, true});
    set_output(0, 1.0);
    set_output(1, torque_);
}

std::span<const kernel::JointDrive> Brake::joint_drives() const {
    return drive_;
}

double Brake::stiffness() const noexcept {
    return stiffness_;
}

}  // namespace mechsim::signals
