#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <optional>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/stepper.hpp"
#include "signal_test_support.hpp"

using mechsim::kernel::JointDrive;
using mechsim::kernel::JointDriveTorque;
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
using mechsim::signals::PhysicalState;
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

namespace {

// The physics stub: a shaft on a frictionless bearing, J theta'' = K (rest - theta) - D omega - load, with the
// stepper's drive applied implicitly (backward Euler: the spring and damper at the step's end state, as the MuJoCo
// bridge's soft equality solves them), so no omega h limit. load is a constant external torque on the shaft (gravity
// through a transmission), positive against the + axis. It publishes the joint coordinate and the drive torque
// K (rest - theta) - D omega it applied, as physics does (PHY-034). For this linear scheme the energy
// J omega^2 / 2 + K (theta - theta_s)^2 / 2 about the static angle theta_s never grows, so from rest the shaft never
// swings more than |theta_s - theta_0| past theta_s.
struct Shaft {
    double inertia = 0.0;
    double load = 0.0;
    double theta = 0.0;
    double omega = 0.0;
    double torque = 0.0;
    bool driven = false;

    void advance(std::span<const JointDrive> drives) {
        const double dt = 0.001;
        REQUIRE(drives.size() == 1);
        const JointDrive& drive = drives[0];
        CHECK(drive.joint == 0);
        double k = 0.0;
        double d = 0.0;
        driven = drive.enabled;
        if (drive.enabled) {
            k = drive.stiffness;
            d = drive.damping;
        }
        omega = (omega + (dt / inertia) * ((k * (drive.rest - theta)) - load)) /
                (1.0 + (dt * d / inertia) + (dt * dt * k / inertia));
        theta += omega * dt;
        torque = (k * (drive.rest - theta)) - (d * omega);
    }

    [[nodiscard]] PhysicalState state() const {
        PhysicalState out = state_at(theta, 0.0, load);
        if (driven) {
            out.joint_drive_torques.push_back(JointDriveTorque{0, torque});
        }
        return out;
    }
};

struct Trace {
    double max_abs_delta = 0.0;
    double last_delta = 0.0;
    double last_torque = 0.0;
    double missed = 0.0;
    bool stalled = false;
    JointDrive last_drive;
};

// steps(k) is the full-step command of tick k; delta is the integrated command less the shaft angle after the step
// that applied the tick's drive, measured once k >= from.
template <typename Steps>
Trace run_shaft(BehavioralStepper& motor, Shaft& shaft, int ticks, Steps steps, int from, double enabled = 1.0) {
    Stepper run(motor);
    Trace trace;
    double command = shaft.theta;
    for (int k = 0; k < ticks; ++k) {
        const double n = steps(k);
        command += n * (pi / 100.0);
        const std::span<const double> out = run.step(shaft.state(), drive(n, 1.0, enabled));
        trace.last_torque = out[0];
        trace.missed += out[2];
        trace.stalled = trace.stalled || out[1] == 1.0;
        REQUIRE(motor.joint_drives().size() == 1);
        trace.last_drive = motor.joint_drives()[0];
        shaft.advance(motor.joint_drives());
        trace.last_delta = command - shaft.theta;
        if (k >= from) {
            trace.max_abs_delta = std::max(trace.max_abs_delta, std::fabs(trace.last_delta));
        }
    }
    return trace;
}

BehavioralStepperParams holding_params(double inertia) {
    BehavioralStepperParams params = motor_params(inertia);
    params.holding_torque = 0.40;
    return params;
}

// K_hold = pi tau_hold / (2 step) = pi 0.40 / (2 pi / 100) = 20 N m/rad.
constexpr double hold_stiffness = 20.0;

}  // namespace

TEST_CASE("An enabled stepper publishes its hold as a joint drive toward the commanded angle", "[SIG-033][KER-027]") {
    BehavioralStepperParams params = holding_params(5.4e-6);
    params.joint = 0;
    BehavioralStepper motor("x_motor", params);
    Stepper run(motor);
    run.step(state_at(0.25, 0.0), drive(1.0, 0.0));
    REQUIRE(motor.joint_drives().size() == 1);
    const JointDrive first = motor.joint_drives()[0];
    CHECK(first.joint == 0);
    CHECK(near(first.rest, 0.25 - (pi / 100.0)));
    CHECK(near(first.stiffness, hold_stiffness));
    CHECK(first.damping == 1e-4);
    CHECK(first.enabled);
    run.step(state_at(0.25, 0.0), drive(0.0, 1.0, 0.0));
    const JointDrive off = motor.joint_drives()[0];
    CHECK_FALSE(off.enabled);
    CHECK(off.rest == 0.25);
}

TEST_CASE("An enabled stepper at rest holds a load below its holding torque at the sag tau_load / K_hold",
          "[SIG-032][SIG-033]") {
    // Static: K_hold delta = load, delta = 0.2 / 20 = 0.01 rad, a third of a step; from rest the swing stays within
    // twice that (the stub's energy bound). J = 5.4e-6 + 2e-5 kg m^2.
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    Shaft shaft{2.54e-5, 0.2};
    const Trace trace = run_shaft(motor, shaft, 3000, [](int) { return 0.0; }, 0);
    const double closed = 0.2 / hold_stiffness;
    INFO("closed-form sag " << closed << " rad, measured " << trace.last_delta << " rad, max " << trace.max_abs_delta);
    CHECK(std::fabs(trace.last_delta - closed) < 0.1 * closed);
    CHECK(trace.max_abs_delta <= 2.0 * closed * (1.0 + 1e-9));
    CHECK(std::fabs(trace.last_torque - 0.2) < 0.1 * 0.2);
    CHECK(trace.missed == 0.0);
    CHECK_FALSE(trace.stalled);
}

TEST_CASE("A load above the holding torque pulls the stepper out: missed steps and stall", "[SIG-033]") {
    // 0.5 N m > tau_avail(0) = 0.40 N m: the drive torque that would balance it exceeds what the motor has, so the
    // pull-out re-synchronizes, counts lost steps and, with zero commanded speed, reports a stall; the shaft falls.
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    Shaft shaft{2.54e-5, 0.5};
    const Trace trace = run_shaft(motor, shaft, 500, [](int) { return 0.0; }, 0);
    CHECK(trace.missed >= 1.0);
    CHECK(trace.stalled);
    CHECK(shaft.theta < -2.0 * (pi / 100.0));
}

TEST_CASE("A load above the curve at a sub-half-step lead loses a step every tick it persists", "[SIG-033]") {
    // The shaft lags the commanded angle by a tenth of a step and the drive reads back 0.5 N m, above the curve at any
    // speed (tau_avail <= 0.40 N m): round(delta / step) = 0, yet the motor has lost synchronism, so each tick it
    // re-synchronizes one step toward the load (sign of delta, or of the drive torque at delta = 0), counts it
    // missed and, with no steps commanded, stalls. After a priming tick that puts the commanded angle on the shaft at 0,
    // rests move by exactly one step angle each tick.
    const double step = pi / 100.0;
    struct Lead {
        double fraction;
        double torque;
        double direction;
    };
    for (const Lead lead : {Lead{0.1, 0.5, -1.0}, Lead{-0.1, -0.5, 1.0}, Lead{0.0, 0.5, -1.0}, Lead{0.0, -0.5, 1.0},
                            Lead{0.4, 0.5, -1.0}}) {
        BehavioralStepper motor("x_motor", holding_params(5.4e-6));
        Stepper run(motor);
        run.step(state_at(0.0, 0.0), drive(0.0));
        double rest = motor.joint_drives()[0].rest;
        CHECK(rest == 0.0);
        for (int k = 0; k < 5; ++k) {
            PhysicalState state = state_at(rest - (lead.fraction * step), 0.0);
            state.joint_drive_torques.push_back(JointDriveTorque{0, lead.torque});
            const std::span<const double> out = run.step(state, drive(0.0));
            INFO("lead " << lead.fraction << " step, torque " << lead.torque << ", tick " << k);
            CHECK(out[2] == 1.0);
            CHECK(out[1] == 1.0);
            const double next = motor.joint_drives()[0].rest;
            CHECK(std::fabs((next - rest) - (lead.direction * step)) < tight);
            rest = next;
        }
    }
}

TEST_CASE("A commanded jump past the pull-out angle re-synchronizes to the shaft's step and misses it all",
          "[SIG-033]") {
    // 3 steps in one tick: |delta| = 3 step angles > 2, round(delta / step) = 3 steps lost, theta_cmd back on the
    // shaft; steps were commanded, so no stall.
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    Stepper run(motor);
    const std::span<const double> out = run.step(state_at(0.0, 0.0), drive(3.0));
    CHECK(out[2] == 3.0);
    CHECK(out[1] == 0.0);
    CHECK(near(motor.joint_drives()[0].rest, 0.0));
}

TEST_CASE("At a constant commanded speed under the curve the stepper tracks with the spring-damper lag",
          "[SIG-032][SIG-033]") {
    // Command ramps from 0 to 1 full step per tick over 200 ticks (to 10 pi rad/s = 300 rpm, table 0.36 N m), then
    // holds for 800. Steady state of the implicit stub at omega = w: K (rest - theta) = load + D w, so the lag is
    // (0.1 + 1e-4 * 10 pi) / 20 = 0.005157 rad, under a step; no torque above the curve, so nothing is missed.
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    Shaft shaft{2.54e-5, 0.1};
    const Trace trace = run_shaft(
        motor, shaft, 1000, [](int k) { return std::min(1.0, static_cast<double>(k) / 200.0); }, 400);
    const double lag = (0.1 + (1e-4 * 10.0 * pi)) / hold_stiffness;
    INFO("closed-form lag " << lag << " rad, measured " << trace.last_delta << " rad");
    CHECK(std::fabs(trace.last_delta - lag) < 0.01 * lag);
    CHECK(trace.max_abs_delta < pi / 100.0);
    CHECK(trace.missed == 0.0);
}

TEST_CASE("The lift's 10 kg on an 8 mm lead screw holds within 1 mm over 2 s with zero steps", "[SIG-033]") {
    // Lead 8 mm: c = 0.008 / (2 pi) m/rad. Load torque m g c = 0.12490 N m; inertia J + m c^2 = 2.161e-5 kg m^2.
    // Sag 0.1249 / 20 = 6.245e-3 rad, at most twice that from rest: 0.0125 rad = 16 um of lift, far inside 1 mm,
    // with K / J h^2 = 0.93, where the old explicit coupling was past its limit once back-driven.
    const double c = 0.008 / (2.0 * pi);
    BehavioralStepper motor("lift_motor", holding_params(5.4e-6));
    Shaft shaft{5.4e-6 + (10.0 * c * c), 10.0 * 9.81 * c};
    const Trace trace = run_shaft(motor, shaft, 2000, [](int) { return 0.0; }, 0);
    const double sag = shaft.load / hold_stiffness;
    INFO("closed-form sag " << sag << " rad, measured " << trace.last_delta << " rad, max " << trace.max_abs_delta);
    CHECK(trace.max_abs_delta * c < 0.001);
    CHECK(trace.max_abs_delta <= 2.0 * sag * (1.0 + 1e-9));
    CHECK(std::fabs(trace.last_delta - sag) < 0.1 * sag);
    CHECK(trace.missed == 0.0);
}

TEST_CASE("A disabled stepper or a dead rail drives nothing and the loaded shaft falls", "[SIG-016][SIG-032]") {
    // Free fall of the shaft under the stub: theta_n = -(load / J) h^2 n (n + 1) / 2, -39.8 rad after 100 ticks.
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    Shaft shaft{2.54e-5, 0.2};
    const Trace trace = run_shaft(motor, shaft, 100, [](int) { return 0.0; }, 0, 0.0);
    CHECK(trace.last_torque == 0.0);
    CHECK_FALSE(trace.last_drive.enabled);
    const double closed = -(0.2 / 2.54e-5) * 1e-6 * 100.0 * 101.0 / 2.0;
    CHECK(std::fabs(shaft.theta - closed) < 1e-9 * std::fabs(closed));
    BehavioralStepper dead("x_motor", holding_params(5.4e-6));
    Stepper run(dead);
    const std::span<const double> out = run.step(state_at(0.0, 0.0, 0.1), drive(1.0, 1.0, 1.0, 0.0));
    CHECK(out[0] == 0.0);
    CHECK(out[1] == 1.0);
    CHECK(out[2] == 1.0);
    CHECK_FALSE(dead.joint_drives()[0].enabled);
}

TEST_CASE("Stall and slip faults scale the hold and miss steps for the window", "[SIG-034][SIG-033]") {
    // Stall at fraction 0.25: tau_avail(0) = 0.10 N m and K_hold = 5 N m/rad. A 0.15 N m load needs 0.15 N m of drive
    // torque to balance, above 0.10, so it pulls out. Healthy, the swing from rest peaks at twice the static drive
    // torque, 0.30 N m, at |omega| <= (0.15 / 20) sqrt(20 / J) = 6.7 rad/s, where the curve still gives 0.39 N m.
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    motor.inject(FailureInjection{FailureMode::stall, SimTime(0), SimTime(1000), 0.0, 0.0, 0.25});
    Shaft shaft{2.54e-5, 0.15};
    const Trace trace = run_shaft(motor, shaft, 300, [](int) { return 0.0; }, 0);
    CHECK(trace.stalled);
    CHECK(trace.missed >= 1.0);
    CHECK(near(trace.last_drive.stiffness, 0.25 * hold_stiffness));
    BehavioralStepper healthy("x_motor", holding_params(5.4e-6));
    Shaft held{2.54e-5, 0.15};
    const Trace firm = run_shaft(healthy, held, 300, [](int) { return 0.0; }, 0);
    CHECK_FALSE(firm.stalled);
    CHECK(firm.missed == 0.0);

    // Slip at fraction 0.25 with 2 commanded steps: 0.5 steps missed at once.
    BehavioralStepper slipping("x_motor", holding_params(5.4e-6));
    slipping.inject(FailureInjection{FailureMode::slip, SimTime(0), SimTime(1), 0.0, 0.0, 0.25});
    Stepper slip_run(slipping);
    const std::span<const double> slipped = slip_run.step(state_at(0.0, 0.0, 0.0), drive(2.0));
    CHECK(slipped[2] == 0.5);
    CHECK(near(slipping.joint_drives()[0].rest, 1.5 * pi / 100.0));

    CHECK_THROWS_AS(slipping.inject(FailureInjection{FailureMode::slip, SimTime(0), SimTime(1), 0.0, 0.0, 1.5}),
                    std::invalid_argument);
    CHECK_THROWS_AS(slipping.inject(FailureInjection{FailureMode::noisy, SimTime(0), SimTime(1), 0.1, 0.0, 0.0}),
                    std::invalid_argument);
}

TEST_CASE("The stepper rejects bad parameters and a shaft outside the published state", "[SIG-032]") {
    CHECK_THROWS_AS(BehavioralStepper("m", motor_params(-1.0)), SignalConfigError);
    BehavioralStepperParams weak = holding_params(0.0);
    weak.holding_torque = -0.1;
    CHECK_THROWS_AS(BehavioralStepper("m", weak), SignalConfigError);
    BehavioralStepperParams sticky = holding_params(0.0);
    sticky.damping = -1.0;
    CHECK_THROWS_AS(BehavioralStepper("m", sticky), SignalConfigError);
    BehavioralStepperParams no_angle = motor_params(0.0);
    no_angle.step_angle = 0.0;
    CHECK_THROWS_AS(BehavioralStepper("m", no_angle), SignalConfigError);
    BehavioralStepperParams far = motor_params(0.0);
    far.transmission = 3;
    BehavioralStepper motor("m", far);
    Stepper run(motor);
    CHECK_THROWS_AS(run.step(state_at(0.0, 0.0), drive(1.0)), mechsim::signals::SignalRunError);
    BehavioralStepperParams off_joint = motor_params(0.0);
    off_joint.joint = 1;
    BehavioralStepper lost("m", off_joint);
    Stepper lost_run(lost);
    CHECK_THROWS_AS(lost_run.step(state_at(0.0, 0.0), drive(1.0)), mechsim::signals::SignalRunError);
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
