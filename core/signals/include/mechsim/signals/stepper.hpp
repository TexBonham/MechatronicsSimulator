#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/signals/component_base.hpp"

namespace mechsim::signals {

// Catalog type step_dir_driver.
struct StepDirDriverParams {
    std::int64_t microsteps = 1;
    bool enable_active_high = true;
};

// Inputs `step` (pulses this tick), `dir`, `enable` (levels, from pin outputs), `vmot` (V, motor rail).
// Outputs `full_steps` = pulses / microsteps, `direction` and `enabled` latched levels, `supply` = vmot passed to
// the coils (SIG-012, SIG-013).
class StepDirDriver final : public BuiltinComponent {
public:
    // Throws SignalConfigError unless 1 <= microsteps <= 256.
    StepDirDriver(std::string instance, StepDirDriverParams params);

    void sample(const PhysicalState& state) override;
    void latch_outputs() override;

private:
    StepDirDriverParams params_;
};

struct TorqueSpeedPoint {
    double speed = 0.0;   // rad/s
    double torque = 0.0;  // N m
};

// Piecewise-linear pull-out curve. Speeds strictly increasing from 0, torques finite and non-negative. Above the last
// speed no torque is available: the curve does not say the motor can follow there.
class TorqueSpeedTable {
public:
    // Throws SignalConfigError on an empty, unordered or non-finite table or one not starting at speed 0.
    explicit TorqueSpeedTable(std::vector<TorqueSpeedPoint> points);
    [[nodiscard]] double available(double speed) const;  // at |speed|

private:
    std::vector<TorqueSpeedPoint> points_;
};

// Catalog type stepper_behavioral, tier 1 (behavioral), a position-commanded actuator whose electromagnetic hold is a
// joint drive physics applies implicitly (SIG-033, PHY-034). joint is the shaft joint's neutral index: the stepper
// reads its coordinate (joint_positions) and the drive torque physics applied on it (joint_drive_torques), and drives
// it. transmission selects the shaft's transmission in the published state (checked to exist). rotor_inertia is the
// catalog instance's parameter, the one source the builder also copies into the shaft body; with shaft_axial_inertia
// given, the two must agree to rotor_inertia_tolerance relative, else construction throws SignalConfigError naming
// both (SIG-037).
struct BehavioralStepperParams {
    double step_angle = 0.0;     // rad per full step
    double rotor_inertia = 0.0;  // kg m^2
    double holding_torque = 0.0;  // N m
    double damping = 1e-4;        // N m s/rad, placeholder (sig.md open parameters)
    std::vector<TorqueSpeedPoint> torque_speed;
    std::size_t joint = 0;
    std::size_t transmission = 0;
    std::optional<double> shaft_axial_inertia;  // kg m^2
    std::string shaft_body;
};

inline constexpr double rotor_inertia_tolerance = 1e-9;
inline constexpr double detent_stiffness_floor = 0.05;  // K_t floor as a fraction of K_hold

// The sinusoidal detent tau(delta) = holding sin(pi delta / (2 step_angle)) on delta = theta_cmd - theta, linearized
// at delta for the implicit drive (SIG-033): stiffness K_t = K_hold cos(pi delta / (2 step_angle)), floored at
// detent_stiffness_floor K_hold, K_hold = pi holding / (2 step_angle); offset = tau / K_t, so the drive's rest theta +
// offset makes the linear spring's torque at theta equal tau. delta = 0 gives K_hold and offset 0. holding 0 gives
// stiffness 0 and offset 0.
struct DetentTangent {
    double torque = 0.0;     // N m
    double stiffness = 0.0;  // N m/rad
    double offset = 0.0;     // rad
};
[[nodiscard]] DetentTangent detent_tangent(double holding_torque, double step_angle, double delta);
inline constexpr double pull_out_steps = 2.0;  // |theta_cmd - theta| beyond which synchronism is lost

// Inputs `full_steps`, `direction`, `enabled`, `supply` (from the driver). Outputs `torque` (N m, the drive torque
// physics applied on the shaft in the step just taken, read back), `stalled` (level), `missed_steps` (full steps this
// tick) (SIG-032, SIG-033).
// Per tick: theta_cmd += n step_angle (n = +-full_steps), omega = (theta - theta_last) / dt. Pull-out when
// |theta_cmd - theta| > pull_out_steps step angles or |drive torque| > table(|omega|): theta_cmd re-synchronizes to
// the step nearest theta on its grid, the lost steps are missed, and stalled is set when no steps were commanded.
// Then the drive {joint, rest theta + offset, stiffness K_t, damping} is published, the tangent of the sinusoidal
// detent at delta = theta_cmd - theta (detent_tangent). Disabled or no supply: the drive is disabled, theta_cmd
// follows theta, commanded steps are missed and stalled is set. Faults (SIG-034): stall scales holding (so the detent)
// and the table by fraction; slip misses fraction of the commanded steps.
class BehavioralStepper final : public BuiltinComponent {
public:
    // Throws SignalConfigError on a non-positive step angle, a negative inertia, holding torque or damping, or a bad
    // table.
    BehavioralStepper(std::string instance, BehavioralStepperParams params);

    void sample(const PhysicalState& state) override;
    void latch_outputs() override;
    [[nodiscard]] std::span<const kernel::JointDrive> joint_drives() const override;

private:
    BehavioralStepperParams params_;
    TorqueSpeedTable table_;
    double drive_torque_ = 0.0;
    double theta_ = 0.0;
    double theta_last_ = 0.0;
    double theta_cmd_ = 0.0;
    bool primed_ = false;
    std::vector<kernel::JointDrive> drive_;
};

}  // namespace mechsim::signals
