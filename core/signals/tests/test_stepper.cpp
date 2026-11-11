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
using mechsim::signals::DetentTangent;
using mechsim::signals::detent_tangent;
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

// Root of f on [low, high] with f(low) and f(high) of opposite sign, by bisection to rounding.
template <typename F>
double bisect(F f, double low, double high) {
    const bool rising = f(low) < 0.0;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (low + high);
        if ((f(mid) < 0.0) == rising) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return 0.5 * (low + high);
}

// Static lag of the sinusoidal detent under load: tau_hold sin(pi delta / (2 step)) = load.
double sinusoid_sag(double load) {
    return (2.0 * (pi / 100.0) / pi) * std::asin(load / 0.40);
}

// Largest lag a shaft released from rest at delta = 0 under a constant load reaches without dissipation: the work of
// the load equals the detent's stored energy, load delta = (2 step tau_hold / pi) (1 - cos(pi delta / (2 step))). With
// x = pi delta / (2 step): (load / tau_hold) x = 1 - cos x, the root in (0, pi] (load < 2 tau_hold / pi).
double swing_bound(double load) {
    const double ratio = load / 0.40;
    const double x = bisect([ratio](double v) { return (ratio * v) - (1.0 - std::cos(v)); }, pi, 1e-6);
    return 2.0 * (pi / 100.0) * x / pi;
}

}  // namespace

TEST_CASE("An enabled stepper publishes its hold as a joint drive toward the commanded angle", "[SIG-033][KER-027]") {
    // At delta = 0 the tangent is the milestone 1 drive: rest theta_cmd, K_hold = 20 N m/rad. One step back gives
    // delta = -step, phase -pi/2: K_t = K_hold cos(-pi/2) = 0 is floored at 0.05 K_hold = 1 N m/rad, and the rest sits
    // tau / K_t = -0.40 / 1 rad from the shaft, so the spring's torque at the shaft is the sinusoid's -0.40 N m.
    BehavioralStepperParams params = holding_params(5.4e-6);
    params.joint = 0;
    BehavioralStepper motor("x_motor", params);
    Stepper run(motor);
    run.step(state_at(0.25, 0.0), drive(0.0));
    REQUIRE(motor.joint_drives().size() == 1);
    const JointDrive held = motor.joint_drives()[0];
    CHECK(held.joint == 0);
    CHECK(held.rest == 0.25);
    CHECK(near(held.stiffness, hold_stiffness));
    CHECK(held.damping == 1e-4);
    CHECK(held.enabled);
    run.step(state_at(0.25, 0.0), drive(1.0, 0.0));
    const JointDrive back = motor.joint_drives()[0];
    CHECK(near(back.stiffness, 0.05 * hold_stiffness));
    CHECK(near(back.rest, 0.25 - 0.40));
    CHECK(near(back.stiffness * (back.rest - 0.25), -0.40));
    run.step(state_at(0.25, 0.0), drive(0.0, 1.0, 0.0));
    const JointDrive off = motor.joint_drives()[0];
    CHECK_FALSE(off.enabled);
    CHECK(off.rest == 0.25);
}

TEST_CASE("The detent tangent matches the sinusoid's torque and slope, floored past one step", "[SIG-033]") {
    // tau(delta) = 0.40 sin(pi delta / (2 step)), K_hold = 20 N m/rad. At delta = step / 2: K_t = K_hold cos(pi / 4),
    // tau = 0.40 sin(pi / 4), offset tau / K_t = (2 step / pi) tan(pi / 4) = 2 step / pi. Past one step the cosine is
    // negative and K_t sits on its floor 0.05 K_hold; at 2 steps tau = 0. The tangent is odd in delta.
    const double step = pi / 100.0;
    const DetentTangent zero = detent_tangent(0.40, step, 0.0);
    CHECK(zero.stiffness == hold_stiffness);
    CHECK(zero.torque == 0.0);
    CHECK(zero.offset == 0.0);
    const DetentTangent half = detent_tangent(0.40, step, step / 2.0);
    CHECK(near(half.stiffness, hold_stiffness * std::cos(pi / 4.0)));
    CHECK(near(half.torque, 0.40 * std::sin(pi / 4.0)));
    CHECK(near(half.offset, 2.0 * step / pi));
    const DetentTangent mirror = detent_tangent(0.40, step, -step / 2.0);
    CHECK(near(mirror.stiffness, half.stiffness));
    CHECK(near(mirror.offset, -half.offset));
    const DetentTangent past = detent_tangent(0.40, step, 1.5 * step);
    CHECK(near(past.stiffness, 0.05 * hold_stiffness));
    CHECK(near(past.torque, 0.40 * std::sin(0.75 * pi)));
    CHECK(near(past.offset, past.torque / past.stiffness));
    const DetentTangent pull_out = detent_tangent(0.40, step, 2.0 * step);
    CHECK(std::fabs(pull_out.torque) < tight);
    CHECK(near(pull_out.stiffness, 0.05 * hold_stiffness));
    const DetentTangent unheld = detent_tangent(0.0, step, step / 2.0);
    CHECK(unheld.stiffness == 0.0);
    CHECK(unheld.offset == 0.0);

    // Through the stepper: shaft held still at -step / 2 below a command of 0, so omega = 0, the predicted angle is
    // the shaft's and delta = step / 2.
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    Stepper run(motor);
    run.step(state_at(0.0, 0.0), drive(0.0));
    run.step(state_at(-step / 2.0, 0.0), drive(0.0));
    run.step(state_at(-step / 2.0, 0.0), drive(0.0));
    const JointDrive drive_half = motor.joint_drives()[0];
    CHECK(near(drive_half.stiffness, hold_stiffness * std::cos(pi / 4.0)));
    CHECK(near(drive_half.rest, (-step / 2.0) + (2.0 * step / pi)));
    CHECK(near(drive_half.stiffness * (drive_half.rest + (step / 2.0)), 0.40 * std::sin(pi / 4.0)));
}

TEST_CASE("Under a static load of half the holding torque the drive settles at a lag of a third of a step",
          "[SIG-033][PHY-034]") {
    // tau(delta*) = T with T = tau_hold / 2: delta* = (2 step / pi) asin(1 / 2) = step / 3. The implicit linear drive's
    // static solution under T is theta_(n+1) = r_n - T / K_t,n, that is delta_(n+1) = delta_n - (tau(delta_n) - T) /
    // tau'(delta_n): one Newton step on tau(delta) = T per tick. From delta = 0 the iterates rise monotonically
    // (tau is concave on (0, step), so Newton from below undershoots and never passes the root) and converge
    // quadratically, far inside 200 ticks to 1e-6 rad. At convergence the spring's torque at the shaft,
    // K_t (r - theta), is tau(delta*) = T, to rounding (1e-9 relative).
    const double step = pi / 100.0;
    const double load = 0.20;
    const double target = step / 3.0;
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    Stepper run(motor);
    double theta = 0.0;
    int converged_at = -1;
    for (int k = 0; k < 200; ++k) {
        run.step(state_at(theta, 0.0, load), drive(0.0));
        const JointDrive d = motor.joint_drives()[0];
        theta = d.rest - (load / d.stiffness);
        if (converged_at < 0 && std::fabs(-theta - target) < 1e-6) {
            converged_at = k;
        }
    }
    run.step(state_at(theta, 0.0, load), drive(0.0));
    const JointDrive settled = motor.joint_drives()[0];
    const double spring = settled.stiffness * (settled.rest - theta);
    INFO("closed-form lag " << target << " rad, measured " << -theta << " rad after " << converged_at
                            << " ticks; spring torque " << spring << " N m against " << load);
    CHECK(converged_at >= 0);
    CHECK(std::fabs(-theta - target) < 1e-6);
    CHECK(std::fabs(spring - load) < 1e-9 * load);
    // The linearized drive versus the sinusoid at delta = step / 2: K_t = K_hold cos(pi / 4), the slope of tau there.
    const double slope = (0.40 * (pi / (2.0 * step))) * std::cos(pi / 4.0);
    CHECK(near(detent_tangent(0.40, step, step / 2.0).stiffness, slope));
}

TEST_CASE("An enabled stepper at rest holds a load below its holding torque at the sag tau_load / K_hold",
          "[SIG-032][SIG-033]") {
    // Static: tau(delta) = load, delta = (2 step / pi) asin(0.2 / 0.4) = step / 3 = 0.01047 rad; from rest the swing
    // stays within the sinusoid's energy bound (swing_bound), which the stub's implicit step can only lower.
    // J = 5.4e-6 + 2e-5 kg m^2.
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    Shaft shaft{2.54e-5, 0.2};
    const Trace trace = run_shaft(motor, shaft, 3000, [](int) { return 0.0; }, 0);
    const double closed = sinusoid_sag(0.2);
    INFO("closed-form sag " << closed << " rad, measured " << trace.last_delta << " rad, max " << trace.max_abs_delta
                            << " rad, swing bound " << swing_bound(0.2) << " rad");
    CHECK(std::fabs(trace.last_delta - closed) < 0.1 * closed);
    CHECK(trace.max_abs_delta <= swing_bound(0.2) * (1.0 + 1e-9));
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
    // the command moves by exactly one step angle each tick; the drive is the detent's tangent at the new lead from
    // the predicted shaft angle theta + omega h, omega the shaft's motion over the last tick.
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
        CHECK(motor.joint_drives()[0].rest == 0.0);
        double command = 0.0;
        double previous = 0.0;
        for (int k = 0; k < 5; ++k) {
            const double theta = command - (lead.fraction * step);
            const double predicted = theta + (theta - previous);
            previous = theta;
            PhysicalState state = state_at(theta, 0.0);
            state.joint_drive_torques.push_back(JointDriveTorque{0, lead.torque});
            const std::span<const double> out = run.step(state, drive(0.0));
            INFO("lead " << lead.fraction << " step, torque " << lead.torque << ", tick " << k);
            CHECK(out[2] == 1.0);
            CHECK(out[1] == 1.0);
            command += lead.direction * step;
            const DetentTangent tangent = detent_tangent(0.40, step, command - predicted);
            CHECK(std::fabs(motor.joint_drives()[0].rest - (predicted + tangent.offset)) < tight);
            CHECK(near(motor.joint_drives()[0].stiffness, tangent.stiffness));
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
    // Command ramps from 0 to 1 full step per tick over 200 ticks (to omega = step / h = (pi / 100) / 0.001 = 10 pi
    // rad/s = 300 rpm, table 0.36 N m), then holds for 800. In steady tracking the shaft moves omega h per tick, so the
    // predicted angle theta + omega h is the end-of-tick angle and the drive's torque there is the sinusoid's: the
    // stub's balance tau(delta) - D omega = load gives the closed form
    // delta = (2 step / pi) asin((load + D omega) / tau_hold) = (2 pi / 100 / pi) asin((0.1 + 1e-4 * 10 pi) / 0.40)
    // = 0.02 asin(0.10314 / 0.40) = 5.216e-3 rad, under a step; no torque above the curve, so nothing is missed.
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    Shaft shaft{2.54e-5, 0.1};
    const Trace trace = run_shaft(
        motor, shaft, 1000, [](int k) { return std::min(1.0, static_cast<double>(k) / 200.0); }, 400);
    const double lag = sinusoid_sag(0.1 + (1e-4 * 10.0 * pi));
    INFO("closed-form lag " << lag << " rad, measured " << trace.last_delta << " rad");
    CHECK(std::fabs(trace.last_delta - lag) < 0.01 * std::fabs(lag));
    CHECK(trace.max_abs_delta < pi / 100.0);
    CHECK(trace.missed == 0.0);
}

TEST_CASE("The lift's 10 kg on an 8 mm lead screw holds within 1 mm over 2 s with zero steps", "[SIG-033]") {
    // Lead 8 mm: c = 0.008 / (2 pi) m/rad. Load torque m g c = 0.12490 N m; inertia J + m c^2 = 2.161e-5 kg m^2.
    // Sag (2 step / pi) asin(0.1249 / 0.4) = 6.352e-3 rad; from rest the swing stays within the sinusoid's energy
    // bound, 0.0130 rad = 17 um of lift, far inside 1 mm, with K / J h^2 = 0.93, where the old explicit coupling was
    // past its limit once back-driven.
    const double c = 0.008 / (2.0 * pi);
    BehavioralStepper motor("lift_motor", holding_params(5.4e-6));
    Shaft shaft{5.4e-6 + (10.0 * c * c), 10.0 * 9.81 * c};
    const Trace trace = run_shaft(motor, shaft, 2000, [](int) { return 0.0; }, 0);
    const double sag = sinusoid_sag(shaft.load);
    INFO("closed-form sag " << sag << " rad, measured " << trace.last_delta << " rad, max " << trace.max_abs_delta
                            << " rad, swing bound " << swing_bound(shaft.load) << " rad");
    CHECK(trace.max_abs_delta * c < 0.001);
    CHECK(trace.max_abs_delta <= swing_bound(shaft.load) * (1.0 + 1e-9));
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
    BehavioralStepper weak("x_motor", holding_params(5.4e-6));
    weak.inject(FailureInjection{FailureMode::stall, SimTime(0), SimTime(1), 0.0, 0.0, 0.25});
    Stepper weak_run(weak);
    weak_run.step(state_at(0.0, 0.0), drive(0.0));
    CHECK(near(weak.joint_drives()[0].stiffness, 0.25 * hold_stiffness));
    weak_run.step(state_at(0.0, 0.0), drive(0.0));
    CHECK(near(weak.joint_drives()[0].stiffness, hold_stiffness));
    BehavioralStepper healthy("x_motor", holding_params(5.4e-6));
    Shaft held{2.54e-5, 0.15};
    const Trace firm = run_shaft(healthy, held, 300, [](int) { return 0.0; }, 0);
    CHECK_FALSE(firm.stalled);
    CHECK(firm.missed == 0.0);

    // Slip at fraction 0.25 with 2 commanded steps: 0.5 steps missed at once, so the command is 1.5 steps ahead of
    // the shaft at 0 and the drive is the detent's tangent there.
    BehavioralStepper slipping("x_motor", holding_params(5.4e-6));
    slipping.inject(FailureInjection{FailureMode::slip, SimTime(0), SimTime(1), 0.0, 0.0, 0.25});
    Stepper slip_run(slipping);
    const std::span<const double> slipped = slip_run.step(state_at(0.0, 0.0, 0.0), drive(2.0));
    CHECK(slipped[2] == 0.5);
    const DetentTangent ahead = detent_tangent(0.40, pi / 100.0, 1.5 * pi / 100.0);
    CHECK(near(slipping.joint_drives()[0].rest, ahead.offset));
    CHECK(near(slipping.joint_drives()[0].stiffness, ahead.stiffness));

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

// ===== Loss of synchronism (SIG-033, brief M2-A3) ====================================================================

TEST_CASE("A free shaft spinning 50 steps per tick past an enabled idle stepper brakes to rest, never gaining energy",
          "[SIG-033][SIG-032]") {
    // Out of synchronism (|omega h| > one step, nothing commanded) the drive is rest r = theta + omega h - s step
    // (s = sign omega), K = tau_avail(|omega|) / step, damping D. In the stub's implicit step from theta, omega:
    //   omega' = omega + h tau / J,  theta' = theta + h omega',  tau = K (r - theta') - D omega'
    // so r - theta' = h (omega - omega') - s step = -h^2 tau / J - s step and
    //   tau (1 + K h^2 / J) = -s K step - D omega' = -s tau_avail - D omega'.
    // With omega > step / h: omega' (1 + h D / (J + K h^2)) = omega - h K step / (J + K h^2) > 0, because
    // step / h >= h K step / (J + K h^2) whenever J >= 0. So omega' keeps the sign of omega, tau opposes it, the kinetic
    // energy J omega^2 / 2 falls strictly every tick, and J |omega - omega'| / h = |tau| <= tau_avail(|omega|) +
    // D |omega'|: the torque-speed table's envelope plus the motor's own damper. Above the table's last speed
    // (1800 rpm = 188.5 rad/s) tau_avail is 0 and the damper alone brakes (omega' = omega / (1 + h D / J)). Once
    // |omega h| <= one step the detent tangent catches the shaft. The catch at about one step per tick is not
    // energy-exact (the tangent's error is second order in the change of shaft speed over a tick, and here K_hold h^2 /
    // J = 3.7), so the shaft may ring once past a step per tick and brake again; it never outruns its starting speed.
    // At rest on the detent, the linear hold's amplitude decays at least as exp(-D t / (2 J)) = exp(-9.26 t): exp(-26)
    // of a step per tick after 2.8 s, so the asserted 1e-3 step per tick is loose against that decay. The command
    // follows the shaft forward step by step, so the missed steps equal its displacement in steps, and it rests on a
    // step of the grid the command started on.
    const double step = pi / 100.0;
    const double h = 0.001;
    const double damping = 1e-4;
    const double inertia = 5.4e-6;
    const TorqueSpeedTable table(nema17_curve());
    BehavioralStepper motor("x_motor", holding_params(inertia));
    Stepper run(motor);
    Shaft shaft{inertia, 0.0};
    shaft.omega = 50.0 * step / h;
    const double start = shaft.omega;
    const double origin = shaft.theta;
    run.step(shaft.state(), drive(0.0, 1.0, 0.0));
    shaft.advance(motor.joint_drives());
    bool caught = false;
    int braking_ticks = 0;
    double missed = 0.0;
    for (int k = 1; k < 3000; ++k) {
        const double before = shaft.omega;
        const std::span<const double> out = run.step(shaft.state(), drive(0.0));
        missed += out[2];
        REQUIRE(motor.joint_drives()[0].enabled);
        shaft.advance(motor.joint_drives());
        INFO("tick " << k << " omega " << before << " -> " << shaft.omega << " rad/s");
        CHECK(std::fabs(shaft.omega) < start);
        if (std::fabs(before * h) > step) {
            CHECK(shaft.omega >= 0.0);
            CHECK(shaft.omega < before);
            const double envelope = table.available(before) + (damping * std::fabs(shaft.omega));
            CHECK(inertia * (before - shaft.omega) / h <= envelope * (1.0 + 1e-9));
            ++braking_ticks;
        } else {
            caught = true;
        }
    }
    INFO("braking ticks " << braking_ticks << ", final omega " << shaft.omega << " rad/s");
    CHECK(braking_ticks > 0);
    CHECK(caught);
    CHECK(std::fabs(shaft.omega) * h < 1e-3 * step);
    CHECK(std::fabs(motor.joint_drives()[0].rest - shaft.theta) < step);
    const double displacement = (shaft.theta - origin) / step;
    CHECK(missed == std::round(displacement));
    CHECK(std::fabs(displacement - std::round(displacement)) < 1e-3);
}

TEST_CASE("A shaft released from a brake hold with the command at the held position stays within one step",
          "[SIG-033][SIG-020]") {
    // The brake is a second linear drive on the shaft (K_b = 200 N m/rad at the held angle 0, the brake_torque / 0.01
    // rad default for 2 N m); the stub applies the parallel sum implicitly and hands the stepper its own spring law at
    // the step's end, K_s (r_s - theta') - D omega', which is its exact share (ruling 23, SIG-039). A 0.2 N m load
    // (half the holding torque) hangs on the shaft; after 300 ticks the brake releases. From any lag between 0 and the
    // static sag the swing past the sag is bounded by the sinusoid's energy, swing_bound(0.2) = 0.0222 rad, under one
    // step (0.0314 rad), and the stepper's share never exceeds the curve, so no step is missed.
    const double step = pi / 100.0;
    const double h = 0.001;
    const double inertia = 2.54e-5;
    const double load = 0.2;
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    Stepper run(motor);
    double theta = 0.0;
    double omega = 0.0;
    double own = 0.0;
    bool driven = false;
    double missed = 0.0;
    double worst = 0.0;
    for (int k = 0; k < 1500; ++k) {
        const bool held = k < 300;
        PhysicalState state = state_at(theta, 0.0, load);
        if (driven) {
            state.joint_drive_torques.push_back(JointDriveTorque{0, own});
        }
        const std::span<const double> out = run.step(state, drive(0.0));
        missed += out[2];
        const JointDrive motor_drive = motor.joint_drives()[0];
        REQUIRE(motor_drive.enabled);
        double k_sum = motor_drive.stiffness;
        double weighted = motor_drive.stiffness * motor_drive.rest;
        if (held) {
            k_sum += 200.0;
        }
        omega = (omega + (h / inertia) * (weighted - (k_sum * theta) - load)) /
                (1.0 + (h * motor_drive.damping / inertia) + (h * h * k_sum / inertia));
        theta += omega * h;
        own = (motor_drive.stiffness * (motor_drive.rest - theta)) - (motor_drive.damping * omega);
        driven = true;
        worst = std::max(worst, std::fabs(theta));
    }
    INFO("largest excursion " << worst << " rad, swing bound " << swing_bound(load) << " rad");
    CHECK(worst <= swing_bound(load) * (1.0 + 1e-9));
    CHECK(worst < step);
    CHECK(missed == 0.0);
}

TEST_CASE("Re-enabled after a power cut mid-move, the stepper re-synchronizes, settles and misses the coasted steps",
          "[SIG-033][SIG-016]") {
    // The command ramps to 3 full steps per tick (94.2 rad/s, 900 rpm, tau_avail 0.24 N m) over 300 ticks and holds;
    // at tick 500 the supply drops for 50 ticks and the command freezes (zero steps from then on). The free shaft
    // (J = 2.54e-5 kg m^2, no load) coasts on undriven at about 3 steps per tick, so on re-enable the slip is 3 steps:
    // out of synchronism, the drive brakes at the curve's torque (about 9.4e3 rad/s^2, under 10 ticks to one step per
    // tick), the command re-synchronizes each tick to the step nearest the predicted shaft angle, and once the slip is
    // under a step the detent catches the shaft and holds it at a step of the grid the command started on at the last
    // disabled tick (theta_cmd = theta there). Every re-synchronization moves the command forward with the shaft, so
    // the steps counted missed after re-enable equal the shaft's displacement from that angle, rounded to steps.
    const double step = pi / 100.0;
    BehavioralStepper motor("x_motor", holding_params(5.4e-6));
    Stepper run(motor);
    Shaft shaft{2.54e-5, 0.0};
    double reference = 0.0;
    double missed_after = 0.0;
    double missed_before = 0.0;
    for (int k = 0; k < 3000; ++k) {
        double n = std::min(3.0, static_cast<double>(k) / 100.0);
        double supply = 24.0;
        if (k >= 500) {
            n = 0.0;
        }
        if (k >= 500 && k < 550) {
            supply = 0.0;
        }
        if (k == 549) {
            reference = shaft.theta;
        }
        const std::span<const double> out = run.step(shaft.state(), drive(n, 1.0, 1.0, supply));
        if (k >= 550) {
            missed_after += out[2];
        } else {
            missed_before += out[2];
        }
        shaft.advance(motor.joint_drives());
    }
    const double displacement = shaft.theta - reference;
    INFO("coasted displacement " << displacement / step << " steps, missed " << missed_after << ", final omega "
                                 << shaft.omega << " rad/s");
    CHECK(missed_before == 0.0);
    CHECK(displacement > 0.0);
    CHECK(missed_after == std::round(displacement / step));
    CHECK(std::fabs(shaft.omega) * 0.001 < 1e-3 * step);
}
