#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/stepper.hpp"
#include "signal_test_support.hpp"

using mechsim::kernel::SimTime;
using mechsim::signals::BehavioralStepper;
using mechsim::signals::BehavioralStepperParams;
using mechsim::signals::FailureInjection;
using mechsim::signals::FailureMode;
using mechsim::signals::SignalConfigError;
using mechsim::signals::StepDirDriver;
using mechsim::signals::StepDirDriverParams;
using mechsim::signals::TorqueSpeedPoint;
using mechsim::signals::TorqueSpeedTable;
using signal_test::state_at;
using signal_test::Stepper;

namespace {

constexpr double pi = std::numbers::pi;
constexpr double rpm = pi / 30.0;  // rad/s per rpm
constexpr double tight = 1e-12;    // double rounding of the few operations below, far under any physical scale

// The catalog part nema17_class_generic: 1.8 deg (pi / 100 rad) per full step and its torque-speed curve.
std::vector<TorqueSpeedPoint> nema17_curve() {
    return {{0.0, 0.40}, {300.0 * rpm, 0.36}, {600.0 * rpm, 0.30}, {1200.0 * rpm, 0.18}, {1800.0 * rpm, 0.10}};
}

BehavioralStepperParams motor_params(double inertia) {
    BehavioralStepperParams params;
    params.step_angle = pi / 100.0;
    params.rotor_inertia = inertia;
    params.torque_speed = nema17_curve();
    return params;
}

// Inputs full_steps, direction, enabled, supply.
std::vector<double> drive(double steps, double direction = 1.0, double enabled = 1.0, double supply = 24.0) {
    return {steps, direction, enabled, supply};
}

bool near(double a, double b) {
    return std::fabs(a - b) < tight;
}

}  // namespace

TEST_CASE("The driver counts the tick's pulses in full steps and latches direction and enable", "[SIG-012][SIG-013]") {
    StepDirDriver driver("x_driver", StepDirDriverParams{16, true});
    Stepper run(driver);
    // 24 pulses at 16 microsteps = 1.5 full steps.
    const std::span<const double> out = run.step(state_at(0.0, 0.0), {24.0, 1.0, 1.0, 24.0});
    CHECK(out[0] == 1.5);
    CHECK(out[1] == 1.0);
    CHECK(out[2] == 1.0);
    CHECK(out[3] == 24.0);
    const std::span<const double> idle = run.step(state_at(0.0, 0.0), {0.0, 0.0, 0.0, 0.0});
    CHECK(idle[0] == 0.0);
    CHECK(idle[1] == 0.0);
    CHECK(idle[2] == 0.0);

    StepDirDriver low("y_driver", StepDirDriverParams{1, false});
    Stepper low_run(low);
    CHECK(low_run.step(state_at(0.0, 0.0), {1.0, 1.0, 0.0, 24.0})[2] == 1.0);
    CHECK_THROWS_AS(StepDirDriver("d", StepDirDriverParams{0, true}), SignalConfigError);
    CHECK_THROWS_AS(StepDirDriver("d", StepDirDriverParams{257, true}), SignalConfigError);
}

TEST_CASE("The torque-speed table interpolates linearly and gives nothing above its last speed", "[SIG-032]") {
    const TorqueSpeedTable table(nema17_curve());
    CHECK(table.available(0.0) == 0.40);
    // 450 rpm lies halfway between 300 and 600 rpm: (0.36 + 0.30) / 2 = 0.33 N m; the curve is symmetric in speed.
    CHECK(near(table.available(450.0 * rpm), 0.33));
    CHECK(near(table.available(-450.0 * rpm), 0.33));
    CHECK(near(table.available(1800.0 * rpm), 0.10));
    CHECK(table.available(1801.0 * rpm) == 0.0);
    CHECK_THROWS_AS(TorqueSpeedTable({}), SignalConfigError);
    CHECK_THROWS_AS(TorqueSpeedTable({{1.0, 0.4}}), SignalConfigError);
    CHECK_THROWS_AS(TorqueSpeedTable({{0.0, 0.4}, {0.0, 0.3}}), SignalConfigError);
    CHECK_THROWS_AS(TorqueSpeedTable({{0.0, -0.1}}), SignalConfigError);
}

TEST_CASE("The stepper delivers the demanded torque up to the curve and stalls just above it", "[SIG-032][SIG-033]") {
    // Rotor inertia 0 isolates the load: demand = load. 1.5 full steps per 1 ms tick is
    // 1.5 * (pi / 100) / 0.001 = 15 pi rad/s = 450 rpm, where the curve gives 0.33 N m.
    const double available = 0.33;
    {
        BehavioralStepper motor("x_motor", motor_params(0.0));
        Stepper run(motor);
        const std::span<const double> out = run.step(state_at(0.0, 0.0, available - 1e-6), drive(1.5));
        CHECK(near(out[0], available - 1e-6));
        CHECK(out[1] == 0.0);
        CHECK(out[2] == 0.0);
    }
    {
        BehavioralStepper motor("x_motor", motor_params(0.0));
        Stepper run(motor);
        const std::span<const double> out = run.step(state_at(0.0, 0.0, available + 1e-6), drive(1.5));
        CHECK(near(out[0], available));
        CHECK(out[1] == 1.0);
        CHECK(out[2] == 1.5);
    }
    {
        // Reverse direction against a negative load: the torque takes the demand's sign.
        BehavioralStepper motor("x_motor", motor_params(0.0));
        Stepper run(motor);
        const std::span<const double> out = run.step(state_at(0.0, 0.0, -0.2), drive(1.5, 0.0));
        CHECK(near(out[0], -0.2));
        CHECK(out[1] == 0.0);
    }
    {
        // 7 steps per tick = 2100 rpm, above the curve's last speed: any load stalls.
        BehavioralStepper motor("x_motor", motor_params(0.0));
        Stepper run(motor);
        const std::span<const double> out = run.step(state_at(0.0, 0.0, 0.01), drive(7.0));
        CHECK(out[0] == 0.0);
        CHECK(out[1] == 1.0);
        CHECK(out[2] == 7.0);
    }
}

TEST_CASE("The stepper demand includes the rotor inertia times the commanded acceleration", "[SIG-032]") {
    // J = 54 g cm^2 = 5.4e-6 kg m^2. One full step per tick from rest: omega = (pi / 100) / 0.001 = 10 pi rad/s,
    // alpha = 10 pi / 0.001 = 1e4 pi rad/s^2, demand = 5.4e-6 * 1e4 pi = 0.054 pi N m (0.170, below 0.36 at
    // 300 rpm). The next tick at the same speed has alpha = 0 and no load: demand 0.
    const double inertia = 5.4e-6;
    BehavioralStepper motor("x_motor", motor_params(inertia));
    Stepper run(motor);
    CHECK(near(run.step(state_at(0.0, 0.0), drive(1.0))[0], 0.054 * pi));
    CHECK(near(run.step(state_at(0.0, 0.0), drive(1.0))[0], 0.0));
    // Stopping from 10 pi rad/s in one tick: demand = -0.054 pi N m, within the 0.40 N m holding torque.
    CHECK(near(run.step(state_at(0.0, 0.0), drive(0.0))[0], -0.054 * pi));
}

TEST_CASE("No stepper torque reaches physics with the motor rail off or the driver disabled", "[SIG-016][SIG-032]") {
    for (const std::vector<double>& inputs : {drive(1.0, 1.0, 1.0, 0.0), drive(1.0, 1.0, 0.0, 24.0)}) {
        BehavioralStepper motor("x_motor", motor_params(0.0));
        Stepper run(motor);
        const std::span<const double> out = run.step(state_at(0.0, 0.0, 0.1), inputs);
        CHECK(out[0] == 0.0);
        CHECK(out[1] == 1.0);
        CHECK(out[2] == 1.0);
    }
    BehavioralStepper idle("x_motor", motor_params(0.0));
    Stepper idle_run(idle);
    const std::span<const double> out = idle_run.step(state_at(0.0, 0.0, 0.0), drive(0.0, 1.0, 1.0, 0.0));
    CHECK(out[0] == 0.0);
    CHECK(out[1] == 0.0);
}

TEST_CASE("Stall and slip faults scale available torque and miss steps for the window", "[SIG-034][SIG-033]") {
    // Stall at fraction 0.5: available at 450 rpm becomes 0.165 N m, so a 0.2 N m load stalls inside the window
    // and is carried outside it.
    BehavioralStepper motor("x_motor", motor_params(0.0));
    motor.inject(FailureInjection{FailureMode::stall, SimTime(0), SimTime(1), 0.0, 0.0, 0.5});
    Stepper run(motor);
    const std::span<const double> stalled = run.step(state_at(0.0, 0.0, 0.2), drive(1.5));
    CHECK(near(stalled[0], 0.165));
    CHECK(stalled[1] == 1.0);
    CHECK(run.step(state_at(0.0, 0.0, 0.2), drive(1.5))[1] == 0.0);

    // Slip at fraction 0.25 with 2 commanded steps: 0.5 steps missed, the rotor follows 1.5 steps (450 rpm).
    BehavioralStepper slipping("x_motor", motor_params(0.0));
    slipping.inject(FailureInjection{FailureMode::slip, SimTime(0), SimTime(1), 0.0, 0.0, 0.25});
    Stepper slip_run(slipping);
    const std::span<const double> slipped = slip_run.step(state_at(0.0, 0.0, 0.1), drive(2.0));
    CHECK(slipped[1] == 0.0);
    CHECK(slipped[2] == 0.5);

    CHECK_THROWS_AS(slipping.inject(FailureInjection{FailureMode::slip, SimTime(0), SimTime(1), 0.0, 0.0, 1.5}),
                    std::invalid_argument);
    CHECK_THROWS_AS(slipping.inject(FailureInjection{FailureMode::noisy, SimTime(0), SimTime(1), 0.1, 0.0, 0.0}),
                    std::invalid_argument);
}

TEST_CASE("The stepper rejects bad parameters and a shaft outside the published state", "[SIG-032]") {
    CHECK_THROWS_AS(BehavioralStepper("m", motor_params(-1.0)), SignalConfigError);
    BehavioralStepperParams no_angle = motor_params(0.0);
    no_angle.step_angle = 0.0;
    CHECK_THROWS_AS(BehavioralStepper("m", no_angle), SignalConfigError);
    BehavioralStepperParams far = motor_params(0.0);
    far.transmission = 3;
    BehavioralStepper motor("m", far);
    Stepper run(motor);
    CHECK_THROWS_AS(run.step(state_at(0.0, 0.0), drive(1.0)), mechsim::signals::SignalRunError);
}

// SIG-037: one rotor inertia source. 1e-9 relative is the tolerance in sig.md; a shaft body copied from the same
// catalog value is equal to rounding, so a 1e-12 relative difference passes and 1e-6 is refused.
TEST_CASE("The stepper refuses a shaft body whose axial inertia is not its rotor inertia", "[SIG-037]") {
    constexpr double rotor = 5.4e-6;
    BehavioralStepperParams same = motor_params(rotor);
    same.shaft_axial_inertia = rotor * (1.0 + 1e-12);
    same.shaft_body = "x_shaft";
    CHECK_NOTHROW(BehavioralStepper("m", same));
    BehavioralStepperParams off = motor_params(rotor);
    off.shaft_axial_inertia = rotor * (1.0 + 1e-6);
    off.shaft_body = "x_shaft";
    try {
        const BehavioralStepper motor("x_motor", off);
        FAIL("a mismatched shaft inertia was accepted");
    } catch (const mechsim::signals::SignalConfigError& error) {
        const std::string text = error.what();
        CHECK(text.find("x_motor") != std::string::npos);
        CHECK(text.find("x_shaft") != std::string::npos);
    }
    BehavioralStepperParams bad = motor_params(rotor);
    bad.shaft_axial_inertia = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(BehavioralStepper("m", bad), mechsim::signals::SignalConfigError);
    CHECK_NOTHROW(BehavioralStepper("m", motor_params(rotor)));
}
