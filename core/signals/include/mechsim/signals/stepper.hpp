#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

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

// Catalog type stepper_behavioral, tier 1 (behavioral). shaft_joint and shaft_joint_dof select the load torque in
// the published state: joints[shaft_joint].generalized_force[shaft_joint_dof], taken as the torque the shaft must
// supply to the load.
struct BehavioralStepperParams {
    double step_angle = 0.0;     // rad per full step
    double rotor_inertia = 0.0;  // kg m^2
    std::vector<TorqueSpeedPoint> torque_speed;
    std::size_t shaft_joint = 0;
    std::size_t shaft_joint_dof = 0;
};

// Inputs `full_steps`, `direction`, `enabled`, `supply` (from the driver). Outputs `torque` (N m, the actuator input
// for the shaft), `stalled` (level), `missed_steps` (full steps this tick) (SIG-032, SIG-033).
// Per tick: n = +-full_steps, omega = n step_angle / dt, demand = J (omega - omega_last) / dt + load. Available is
// the table at |omega| when enabled with supply > 0, else 0. demand <= available: torque = demand, steps followed.
// demand > available: stalled, torque = available with the demand's sign, every commanded step missed, the rotor
// speed taken as 0. Faults (SIG-034): stall scales available by fraction; slip misses fraction of the steps.
class BehavioralStepper final : public BuiltinComponent {
public:
    // Throws SignalConfigError on a non-positive step angle, a negative inertia or a bad table.
    BehavioralStepper(std::string instance, BehavioralStepperParams params);

    void sample(const PhysicalState& state) override;
    void latch_outputs() override;

private:
    BehavioralStepperParams params_;
    TorqueSpeedTable table_;
    double load_ = 0.0;
    double last_speed_ = 0.0;
};

}  // namespace mechsim::signals
