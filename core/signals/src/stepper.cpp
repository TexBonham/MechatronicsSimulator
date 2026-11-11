#include "mechsim/signals/stepper.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <span>
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

DetentTangent detent_tangent(double holding_torque, double step_angle, double delta) {
    const double phase = std::numbers::pi * delta / (2.0 * step_angle);
    const double k_hold = std::numbers::pi * holding_torque / (2.0 * step_angle);
    const double stiffness = std::max(k_hold * std::cos(phase), detent_stiffness_floor * k_hold);
    DetentTangent tangent;
    tangent.torque = holding_torque * std::sin(phase);
    tangent.stiffness = stiffness;
    if (stiffness > 0.0) {
        tangent.offset = tangent.torque / stiffness;
    }
    return tangent;
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
      params_(std::move(params)), table_(params_.torque_speed), drive_(1) {
    if (!std::isfinite(params_.holding_torque) || params_.holding_torque < 0.0) {
        throw SignalConfigError("stepper holding torque must be finite and non-negative");
    }
    if (!std::isfinite(params_.damping) || params_.damping < 0.0) {
        throw SignalConfigError("stepper damping must be finite and non-negative");
    }
    if (!std::isfinite(params_.step_angle) || params_.step_angle <= 0.0) {
        throw SignalConfigError("stepper step angle must be finite and positive");
    }
    if (!std::isfinite(params_.rotor_inertia) || params_.rotor_inertia < 0.0) {
        throw SignalConfigError("stepper rotor inertia must be finite and non-negative");
    }
    if (params_.shaft_axial_inertia.has_value()) {
        const double shaft = *params_.shaft_axial_inertia;
        const double scale = std::max(std::fabs(params_.rotor_inertia), std::fabs(shaft));
        if (!std::isfinite(shaft) || std::fabs(shaft - params_.rotor_inertia) > rotor_inertia_tolerance * scale) {
            throw SignalConfigError("stepper '" + std::string(instance_name()) + "' rotor_inertia " +
                                    std::to_string(params_.rotor_inertia) + " kg m^2 differs from shaft body '" +
                                    params_.shaft_body + "' axial inertia " + std::to_string(shaft) +
                                    " kg m^2; the builder copies rotor_inertia into the shaft body (SIG-037)");
        }
    }
    drive_[0].joint = params_.joint;
}

void BehavioralStepper::sample(const PhysicalState& state) {
    if (params_.transmission >= state.transmissions.size()) {
        throw SignalRunError("stepper '" + std::string(instance_name()) +
                             "': transmission outside the published state");
    }
    if (params_.joint >= state.joint_positions.size()) {
        throw SignalRunError("stepper '" + std::string(instance_name()) + "': shaft joint " +
                             std::to_string(params_.joint) + " outside the published joint positions");
    }
    theta_ = state.joint_positions[params_.joint];
    drive_torque_ = 0.0;
    for (const kernel::JointDriveTorque& applied : state.joint_drive_torques) {
        if (applied.joint == params_.joint) {
            drive_torque_ = applied.torque;
        }
    }
    if (!primed_) {
        theta_last_ = theta_;
        theta_cmd_ = theta_;
        primed_ = true;
    }
}

void BehavioralStepper::latch_outputs() {
    const double dt = period_seconds();
    const double step = params_.step_angle;
    double steps = input(0);
    if (!level(input(1))) {
        steps = -steps;
    }
    double missed = 0.0;
    double scale = 1.0;
    const std::optional<FailureInjection> fault = active_fault();
    if (fault.has_value() && fault->mode == FailureMode::slip) {
        missed = std::fabs(steps) * fault->fraction;
        steps *= 1.0 - fault->fraction;
    }
    if (fault.has_value() && fault->mode == FailureMode::stall) {
        scale = fault->fraction;
    }
    const double omega = (theta_ - theta_last_) / dt;
    theta_last_ = theta_;
    kernel::JointDrive& drive = drive_[0];
    drive.joint = params_.joint;
    set_output(0, drive_torque_);
    if (!level(input(2)) || !(input(3) > 0.0)) {
        theta_cmd_ = theta_;
        drive = kernel::JointDrive{params_.joint, theta_cmd_, 0.0, 0.0, false};
        set_output(1, static_cast<double>(steps != 0.0));
        set_output(2, missed + std::fabs(steps));
        return;
    }
    theta_cmd_ += steps * step;
    const double delta = theta_cmd_ - theta_;
    const double available = scale * table_.available(omega);
    bool stalled = false;
    const bool above_curve = std::fabs(drive_torque_) > available;
    if (std::fabs(delta) > pull_out_steps * step || above_curve) {
        double lost = std::round(delta / step);
        if (above_curve && std::fabs(lost) < 1.0) {
            double toward = delta;
            if (delta == 0.0) {
                toward = drive_torque_;
            }
            lost = std::copysign(1.0, toward);
        }
        theta_cmd_ -= lost * step;
        missed += std::fabs(lost);
        stalled = steps == 0.0;
    }
    const DetentTangent tangent = detent_tangent(scale * params_.holding_torque, step, theta_cmd_ - theta_);
    drive = kernel::JointDrive{params_.joint, theta_ + tangent.offset, tangent.stiffness, params_.damping, true};
    set_output(1, static_cast<double>(stalled));
    set_output(2, missed);
}

std::span<const kernel::JointDrive> BehavioralStepper::joint_drives() const {
    return drive_;
}

}  // namespace mechsim::signals
