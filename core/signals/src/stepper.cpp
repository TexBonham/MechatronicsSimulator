#include "mechsim/signals/stepper.hpp"

#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mechsim::signals {

namespace {

constexpr std::int64_t max_microsteps = 256;

bool level(double value) noexcept {
    return value >= 0.5;
}

}  // namespace

StepDirDriver::StepDirDriver(std::string instance, StepDirDriverParams params)
    : BuiltinComponent(std::move(instance), {"step", "dir", "enable", "vmot"},
                       {"full_steps", "direction", "enabled", "supply"}, {}),
      params_(params) {
    if (params_.microsteps < 1 || params_.microsteps > max_microsteps) {
        throw SignalConfigError("driver microsteps must lie in [1, 256]");
    }
}

void StepDirDriver::sample(const PhysicalState& /*state*/) {}

void StepDirDriver::latch_outputs() {
    bool enabled = level(input(2));
    if (!params_.enable_active_high) {
        enabled = !enabled;
    }
    set_output(0, input(0) / static_cast<double>(params_.microsteps));
    set_output(1, static_cast<double>(level(input(1))));
    set_output(2, static_cast<double>(enabled));
    set_output(3, input(3));
}

TorqueSpeedTable::TorqueSpeedTable(std::vector<TorqueSpeedPoint> points) : points_(std::move(points)) {
    if (points_.empty() || points_.front().speed != 0.0) {
        throw SignalConfigError("a torque-speed table starts at speed 0");
    }
    for (std::size_t i = 0; i < points_.size(); ++i) {
        const TorqueSpeedPoint& p = points_[i];
        if (!std::isfinite(p.speed) || !std::isfinite(p.torque) || p.torque < 0.0) {
            throw SignalConfigError("torque-speed points must be finite with non-negative torque");
        }
        if (i > 0 && !(p.speed > points_[i - 1].speed)) {
            throw SignalConfigError("torque-speed speeds must increase strictly");
        }
    }
}

double TorqueSpeedTable::available(double speed) const {
    const double s = std::fabs(speed);
    if (s > points_.back().speed) {
        return 0.0;
    }
    for (std::size_t i = 1; i < points_.size(); ++i) {
        const TorqueSpeedPoint& lo = points_[i - 1];
        const TorqueSpeedPoint& hi = points_[i];
        if (s <= hi.speed) {
            const double w = (s - lo.speed) / (hi.speed - lo.speed);
            return lo.torque + (w * (hi.torque - lo.torque));
        }
    }
    return points_.back().torque;
}

BehavioralStepper::BehavioralStepper(std::string instance, BehavioralStepperParams params)
    : BuiltinComponent(std::move(instance), {"full_steps", "direction", "enabled", "supply"},
                       {"torque", "stalled", "missed_steps"}, {FailureMode::stall, FailureMode::slip}),
      params_(std::move(params)), table_(params_.torque_speed) {
    if (!std::isfinite(params_.step_angle) || params_.step_angle <= 0.0) {
        throw SignalConfigError("stepper step angle must be finite and positive");
    }
    if (!std::isfinite(params_.rotor_inertia) || params_.rotor_inertia < 0.0) {
        throw SignalConfigError("stepper rotor inertia must be finite and non-negative");
    }
}

void BehavioralStepper::sample(const PhysicalState& state) {
    if (params_.shaft_joint >= state.joints.size()) {
        throw SignalRunError("stepper '" + std::string(instance_name()) + "': shaft joint outside the published state");
    }
    const std::vector<double>& force = state.joints[params_.shaft_joint].generalized_force;
    if (params_.shaft_joint_dof >= force.size()) {
        throw SignalRunError("stepper '" + std::string(instance_name()) + "': shaft DOF outside the joint");
    }
    load_ = force[params_.shaft_joint_dof];
}

void BehavioralStepper::latch_outputs() {
    const double dt = period_seconds();
    double steps = input(0);
    if (!level(input(1))) {
        steps = -steps;
    }
    double missed = 0.0;
    double available_scale = 1.0;
    const std::optional<FailureInjection> fault = active_fault();
    if (fault.has_value() && fault->mode == FailureMode::slip) {
        missed = std::fabs(steps) * fault->fraction;
        steps *= 1.0 - fault->fraction;
    }
    if (fault.has_value() && fault->mode == FailureMode::stall) {
        available_scale = fault->fraction;
    }
    const double speed = steps * params_.step_angle / dt;
    const double demand = (params_.rotor_inertia * (speed - last_speed_) / dt) + load_;
    double available = 0.0;
    if (level(input(2)) && input(3) > 0.0) {
        available = available_scale * table_.available(speed);
    }
    if (std::fabs(demand) > available) {
        set_output(0, std::copysign(available, demand));
        set_output(1, 1.0);
        set_output(2, missed + std::fabs(steps));
        last_speed_ = 0.0;
        return;
    }
    set_output(0, demand);
    set_output(1, 0.0);
    set_output(2, missed);
    last_speed_ = speed;
}

}  // namespace mechsim::signals
