#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <complex>
#include <cstdio>
#include <numbers>
#include <vector>

#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/transmission.hpp"

using mechsim::engine::Body;
using mechsim::engine::JointType;
using mechsim::engine::MultibodyConfig;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::PowerFlow;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::SolverNonConvergenceError;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::Transmission;
using mechsim::engine::TransmissionCoupling;
using mechsim::engine::TransmissionOutput;
using mechsim::engine::TransmissionType;

// Regularised efficiency switch (ruling 47) and the self-locking screw row (ruling 48); the model and its derivation
// are in transmission.hpp. Models: shaft(s) on the world (Revolute about z, q[0] = theta), output on the world.
//
// Settling bound. Below omega_s the efficiency loss is the viscous shaft damping c = |alpha| n F / omega_s, frozen
// per step with F at the step start; with F at its static value the scheme is backward Euler on a linear system
// (mass M, stiffness K, damping D including c), and settling_steps returns N with rho^N <= 1e-20 over the decaying
// modes of its step matrix (as in test_transmissions.cpp). Where the side (forward or backward) may change, N is the
// larger of the two sides' counts. 1e-20 against the asserted 1e-9 leaves 1e11 for amplitude and conditioning.
//
// Tolerances (docs/spec/eng.md, transmission unit-test tolerance): 1e-9 relative settled values; 1e-12 relative
// published values against the hand formulas; exact (==) for every held shaft coordinate and velocity, since a
// binding lock row sets the shaft velocity to 0 exactly and theta + h 0 = theta.

namespace {

constexpr double gravity = 9.81;
constexpr double transient = 1e-20;
constexpr double rel = 1e-9;
constexpr double omega_s = MultibodyConfig::default_friction_slip_threshold;

Body shaft(double inertia) {
    Body body;
    body.parent = -1;
    body.joint.type = JointType::Revolute;
    body.joint.axis = Eigen::Vector3d::UnitZ();
    body.mass = 1.0;
    body.inertia = Eigen::Vector3d(inertia, inertia, inertia).asDiagonal();
    return body;
}

Body slider(double mass, const Eigen::Vector3d& axis) {
    Body body;
    body.parent = -1;
    body.joint.type = JointType::Prismatic;
    body.joint.axis = axis;
    body.mass = mass;
    return body;
}

Eigen::Matrix2d coupling_matrix(double gain, double n_eff) {
    const Eigen::Vector2d a(gain, -1.0);
    const Eigen::Vector2d b(n_eff, -1.0);
    return a * b.transpose();
}

long settling_steps(const Eigen::Matrix2d& m, const Eigen::Matrix2d& k, const Eigen::Matrix2d& d, double h) {
    const Eigen::Matrix2d s = m + h * d + h * h * k;
    const Eigen::Matrix2d si = s.inverse();
    Eigen::Matrix4d a;
    a.topLeftCorner<2, 2>() = Eigen::Matrix2d::Identity() - h * h * si * k;
    a.topRightCorner<2, 2>() = h * si * m;
    a.bottomLeftCorner<2, 2>() = -h * si * k;
    a.bottomRightCorner<2, 2>() = si * m;
    const Eigen::EigenSolver<Eigen::Matrix4d> solver(a);
    double rho = 0.0;
    for (Eigen::Index i = 0; i < 4; ++i) {
        const std::complex<double> lambda = solver.eigenvalues()[i];
        if (std::abs(lambda - 1.0) < 1e-6) {
            continue;
        }
        rho = std::max(rho, std::abs(lambda));
    }
    REQUIRE(rho < 1.0);
    const long n = static_cast<long>(std::ceil(std::log(transient) / std::log(rho)));
    REQUIRE(n < 20000);
    return n;
}

bool close(double measured, double target, double tolerance) {
    std::printf("  target %.15g measured %.15g\n", target, measured);
    return std::abs(measured - target) <= tolerance * std::abs(target);
}

Transmission lossy_belt() {
    Transmission t;
    t.ratio = 0.02;
    t.efficiency = 0.9;
    t.back_efficiency = 0.8;
    t.stiffness = 1e5;
    return t;
}

}  // namespace

TEST_CASE("belt efficiency gain passes linearly through n inside the threshold shaft speed", "[ENG-043][ENG-045]") {
    // Hand formulas (transmission.hpp, ruling 47) at a stretch E = 1e-4 m (F = k E = 10 N, L = -n F < 0): at and
    // above omega_s the explicit gain is n / eta_f (omega > 0, P < 0) or n eta_b (omega < 0, P > 0) bit for bit, the
    // loss there being the implicit shaft damping c = |alpha| n F / |omega| on a = [n, -1] (brief M3-E6: a gain g
    // applied to the end-of-step force put energy in whenever the flow reversed within a step); below it
    // g = n (1 + s_w alpha), a = [n, -1] and the loss equals the viscous torque -c omega, c = |alpha| n F / omega_s;
    // at omega = 0 g = n and c takes the larger loss.
    const Transmission t = lossy_belt();
    const double n = t.ratio;
    const double x = -1e-4;
    const double f = t.stiffness * 1e-4;
    for (const double factor : {1.0, 2.0, 1e3}) {
        const TransmissionCoupling fwd =
            mechsim::engine::couple(t, 0.0, 0.0, 0.0, factor * omega_s, x, 0.0, 0.0, omega_s);
        REQUIRE(fwd.output.power_flow == PowerFlow::Forward);
        REQUIRE(fwd.gain == n / t.efficiency);
        REQUIRE(fwd.stiffness_gain == n);
        REQUIRE(close(fwd.shaft_damping, (1.0 / t.efficiency - 1.0) * n * f / (factor * omega_s), 1e-12));
        const TransmissionCoupling back =
            mechsim::engine::couple(t, 0.0, 0.0, 0.0, -factor * omega_s, x, 0.0, 0.0, omega_s);
        REQUIRE(back.output.power_flow == PowerFlow::Backward);
        REQUIRE(back.gain == n * t.back_efficiency);
        REQUIRE(back.stiffness_gain == n);
        REQUIRE(close(back.shaft_damping, (1.0 - t.back_efficiency) * n * f / (factor * omega_s), 1e-12));
    }
    for (const double fraction : {0.25, -0.25, 1e-12, -1e-12}) {
        const double omega = fraction * omega_s;
        const TransmissionCoupling c = mechsim::engine::couple(t, 0.0, 0.0, 0.0, omega, x, 0.0, 0.0, omega_s);
        double alpha = 1.0 / t.efficiency - 1.0;
        PowerFlow flow = PowerFlow::Forward;
        if (omega < 0.0) {
            alpha = t.back_efficiency - 1.0;
            flow = PowerFlow::Backward;
        }
        REQUIRE(c.output.power_flow == flow);
        REQUIRE(c.stiffness_gain == n);
        REQUIRE(std::abs(c.gain - n * (1.0 + std::abs(fraction) * alpha)) <= 1e-12 * n);
        const double damping = std::abs(alpha) * n * f / omega_s;
        REQUIRE(std::abs(c.shaft_damping - damping) <= 1e-12 * damping);
        REQUIRE(std::abs(c.shaft_torque - (-n * f - c.shaft_damping * omega)) <= 1e-12 * n * f);
        REQUIRE(std::abs(c.gain - n) <= std::abs(fraction) * 0.2 * n + 1e-15 * n);
    }
    const TransmissionCoupling rest = mechsim::engine::couple(t, 0.0, 0.0, 0.0, 0.0, x, 0.0, 0.0, omega_s);
    REQUIRE(rest.output.power_flow == PowerFlow::Static);
    REQUIRE(rest.gain == n);
    REQUIRE(std::abs(rest.shaft_damping - 0.2 * n * f / omega_s) <= 1e-12 * 0.2 * n * f / omega_s);
}

namespace {

struct SagRun {
    double theta = 0.0;
    double target = 0.0;
    double load_torque = 0.0;
    long derived = 0;
    long observed = -1;
    PowerFlow first = PowerFlow::Static;
    std::vector<double> trace;
};

// Shaft J held by a drive spring K (damping D_d, rest 0) against a constant output force F0 through a lossy coupling
// (eta_f 0.9, eta_b 0.8). Static sag (i): theta = -n F0 / K. Start at rest at theta0 = factor theta_static with the
// coupling stretched to F0 (x0 = n theta0 - F0 / k), so the start error e0 = (factor - 1) theta_static is inside the
// old band. Creep (ii): the start speed is at most K |e0| / D_d < omega_s (asserted), the system is heavily
// overdamped and |e| decreases, so every later step is below omega_s, where c = |alpha| n F0 / omega_s frozen makes
// the scheme the linear backward Euler of settling_steps with D = c_k ab + diag(D_d + c, 0), K = k ab + diag(K, 0).
// N = 1 (the start step, c at the larger loss) + the larger of the forward and backward counts.
SagRun run_sag(bool gear, double factor) {
    Transmission t = lossy_belt();
    double n = t.ratio;
    double load = 10.0;
    double output_inertia = 1.0;
    Body output = slider(output_inertia, Eigen::Vector3d::UnitX());
    if (gear) {
        t.type = TransmissionType::GearPair;
        t.ratio = 5.0;
        t.stiffness = 1e3;
        n = 1.0 / t.ratio;
        load = 1.0;
        output_inertia = 2.5e-3;
        output = shaft(output_inertia);
    }
    const double j = 1e-4;
    const double k_drive = 1e4;
    const double d_drive = 200.0;
    const double mu = 1.0 / (1.0 / output_inertia + n * n / j);
    t.damping = 2.0 * std::sqrt(t.stiffness * mu);
    MultibodyModel model;
    model.bodies.push_back(shaft(j));
    model.bodies.push_back(output);
    model.transmissions.push_back(t);
    MultibodyProvider provider(model);
    provider.set_joint_drive(0, 0, 0.0, k_drive, d_drive);
    SemiImplicitEuler integrator{StepConfig{}};
    const double h = integrator.config().h;
    SagRun run;
    run.target = -n * load / k_drive;
    State state;
    state.q = provider.neutral_positions();
    state.q[0] = factor * run.target;
    state.q[1] = n * state.q[0] - load / t.stiffness;
    state.v = Eigen::VectorXd::Zero(2);
    const double e0 = std::abs(state.q[0] - run.target);
    REQUIRE(k_drive * e0 / d_drive < omega_s);
    const Eigen::Matrix2d mass = Eigen::Vector2d(j, output_inertia).asDiagonal();
    const Eigen::Matrix2d ab = coupling_matrix(n, n);
    long longest = 0;
    for (const double alpha : {1.0 / t.efficiency - 1.0, 1.0 - t.back_efficiency}) {
        const double c = alpha * n * load / omega_s;
        Eigen::Matrix2d stiffness = t.stiffness * ab;
        stiffness(0, 0) += k_drive;
        Eigen::Matrix2d damping = t.damping * ab;
        damping(0, 0) += d_drive + c;
        longest = std::max(longest, settling_steps(mass, stiffness, damping, h));
    }
    run.derived = 1 + longest;
    const Eigen::VectorXd force = Eigen::Vector2d(0.0, -load);
    for (long i = 0; i < run.derived; ++i) {
        provider.set_generalized_forces(force);
        provider.step(integrator, state);
        REQUIRE(std::abs(state.v[0]) < omega_s);
        if (i == 0) {
            run.first = provider.outputs().transmissions.at(0).power_flow;
        }
        if (run.observed < 0 && std::abs(state.q[0] - run.target) <= rel * std::abs(run.target)) {
            run.observed = i + 1;
        }
        run.trace.push_back(state.q[0]);
        run.trace.push_back(state.v[1]);
    }
    run.theta = state.q[0];
    run.load_torque = provider.outputs().transmissions.at(0).shaft_load_torque;
    std::printf("  %s factor %.2f: derived %ld steps, observed %ld\n", gear ? "gear" : "belt", factor, run.derived,
                run.observed);
    return run;
}

}  // namespace

TEST_CASE("belt and gear static sag is n F / K approached from above and from below", "[ENG-043][ENG-045]") {
    for (const bool gear : {false, true}) {
        const SagRun above = run_sag(gear, 1.05);
        REQUIRE(above.first == PowerFlow::Forward);
        REQUIRE(close(above.theta, above.target, rel));
        REQUIRE(close(above.load_torque, -above.target * 1e4, rel));
        const SagRun below = run_sag(gear, 0.95);
        REQUIRE(below.first == PowerFlow::Backward);
        REQUIRE(close(below.theta, below.target, rel));
        REQUIRE(close(below.load_torque, -below.target * 1e4, rel));
    }
}

TEST_CASE("belt and gear regularised efficiency run is bitwise identical when repeated",
          "[ENG-043][ENG-045][ENG-023]") {
    for (const bool gear : {false, true}) {
        REQUIRE(run_sag(gear, 0.95).trace == run_sag(gear, 0.95).trace);
    }
}

namespace {

// Vertical self-locking lead screw: lead 4 mm (n = 4e-3 / 2 pi), eta_f 0.4, eta_b 0.3, k = 1e6 N/m, coupling damping
// critical on the carriage alone, carriage m = 5 kg under gravity: F = m g, L = -n m g < 0, so s = +1 (the shaft may
// not turn negative while the row is present).
constexpr double screw_mass = 5.0;
constexpr double screw_k = 1e6;

Transmission self_locking_screw(bool self_locking) {
    Transmission t;
    t.type = TransmissionType::LeadScrew;
    t.ratio = 4e-3;
    t.self_locking = self_locking;
    t.efficiency = 0.4;
    t.back_efficiency = 0.3;
    t.stiffness = screw_k;
    t.damping = 2.0 * std::sqrt(screw_k * screw_mass);
    return t;
}

MultibodyModel screw_model(double j, bool self_locking) {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0, 0, -gravity);
    model.bodies.push_back(shaft(j));
    model.bodies.push_back(slider(screw_mass, Eigen::Vector3d::UnitZ()));
    model.transmissions.push_back(self_locking_screw(self_locking));
    return model;
}

// Carriage at its static stretch, shaft at rest: F = m g from the first step.
State hanging(const MultibodyProvider& provider) {
    State state;
    state.q = provider.neutral_positions();
    state.q[1] = -screw_mass * gravity / screw_k;
    state.v = Eigen::VectorXd::Zero(provider.velocity_size());
    return state;
}

double screw_n() {
    return mechsim::engine::output_per_input(self_locking_screw(true));
}

}  // namespace

TEST_CASE("self-locking screw holds a hanging load with no motor torque for 10 s", "[ENG-044]") {
    // Zero drive, T L = 0: the row is present and the static gain n applies, so the load torque n m g back-drives
    // the shaft in the solve and the row binds, setting omega to 0 exactly: theta never moves, at J = 1e-5 (where the
    // old hold's pivot condition J > n (h^2 k + h c) fails) and at J = 1e-2. The first step starts at F = 0 with T = 0,
    // where the row is bilateral (the end-of-step force would otherwise turn the shaft through the implicit
    // coupling). Started at rest from x = 0 the carriage
    // is a critically damped oscillator on the held screw, sqrt(k / m) = 447 rad/s: x -> -m g / k long before 10 s,
    // and the row torque -> n m g (1e-9).
    for (const double j : {1e-5, 1e-2}) {
        MultibodyProvider provider(screw_model(j, true));
        SemiImplicitEuler integrator{StepConfig{}};
        const double h = integrator.config().h;
        const double n = screw_n();
        const Transmission t = self_locking_screw(true);
        if (j < 1e-4) {
            REQUIRE(j < n * (h * h * t.stiffness + h * t.damping));
        }
        State state;
        state.q = provider.neutral_positions();
        state.v = Eigen::VectorXd::Zero(2);
        for (int i = 0; i < 10000; ++i) {
            provider.step(integrator, state);
            REQUIRE(state.q[0] == 0.0);
            REQUIRE(state.v[0] == 0.0);
        }
        std::printf("  J %g: shaft drift %.3g rad after 10 s\n", j, state.q[0]);
        const TransmissionOutput& o = provider.outputs().transmissions.at(0);
        REQUIRE(o.power_flow == PowerFlow::Locked);
        REQUIRE(o.shaft_load_torque == 0.0);
        REQUIRE(close(o.lock_torque, n * screw_mass * gravity, rel));
        REQUIRE(close(state.q[1], -screw_mass * gravity / screw_k, rel));
        provider.set_shaft_torque(0, 1.0);
        const TransmissionOutput lift = provider.transmission_states(state.q, state.v).at(0);
        REQUIRE(lift.power_flow == PowerFlow::Static);
        REQUIRE(lift.shaft_load_torque == n * lift.force);
    }
}

TEST_CASE("self-locking screw holds after a lifting move stops", "[ENG-044]") {
    // Lift with T = 2 n m g / eta_f for 100 steps (forward, P < 0), then remove the drive: the forward gain
    // decelerates the shaft, the step that would reverse it binds the row (omega = 0 exactly), and from then on the
    // zero-drive hold of the previous case applies: theta stays exactly where it stopped for 2 s.
    for (const double j : {1e-5, 1e-2}) {
        MultibodyProvider provider(screw_model(j, true));
        SemiImplicitEuler integrator{StepConfig{}};
        const double n = screw_n();
        const double lift = 2.0 * n * screw_mass * gravity / 0.4;
        State state = hanging(provider);
        for (int i = 0; i < 100; ++i) {
            provider.set_shaft_torque(0, lift);
            provider.step(integrator, state);
        }
        REQUIRE(provider.outputs().transmissions.at(0).power_flow == PowerFlow::Forward);
        REQUIRE(state.v[0] > omega_s);
        int stop = -1;
        for (int i = 0; i < 20000 && stop < 0; ++i) {
            provider.step(integrator, state);
            REQUIRE(state.v[0] >= 0.0);
            if (state.v[0] == 0.0) {
                stop = i;
            }
        }
        REQUIRE(stop >= 0);
        const double held = state.q[0];
        REQUIRE(held > 0.0);
        for (int i = 0; i < 2000; ++i) {
            provider.step(integrator, state);
            REQUIRE(state.q[0] == held);
            REQUIRE(state.v[0] == 0.0);
        }
        std::printf("  J %g: stopped after %d steps at theta %.15g, drift 0 over 2 s\n", j, stop, held);
        REQUIRE(provider.outputs().transmissions.at(0).power_flow == PowerFlow::Locked);
    }
}

TEST_CASE("lowering a self-locking screw under a drive proceeds unresisted", "[ENG-044]") {
    // T = -n m g / 2 pushes the back-driving way (T L > 0): no row and, with P >= 0, gain 0, so the shaft row of f,
    // K and D carries T alone: J omega_k = k h T, theta_N = h^2 T N (N + 1) / (2 J) to rounding (1e-9). The
    // published load torque is 0 (Locked): unresisted lowering is the documented limit of ENG-044. The carriage
    // follows at n T / J < g, so F stays positive.
    for (const double j : {1e-5, 1e-2}) {
        MultibodyProvider provider(screw_model(j, true));
        SemiImplicitEuler integrator{StepConfig{}};
        const double h = integrator.config().h;
        const double n = screw_n();
        const double drive = -0.5 * n * screw_mass * gravity;
        REQUIRE(std::abs(n * drive / j) < gravity);
        State state = hanging(provider);
        const long steps = 100;
        for (long i = 0; i < steps; ++i) {
            provider.set_shaft_torque(0, drive);
            provider.step(integrator, state);
            const TransmissionOutput& o = provider.outputs().transmissions.at(0);
            REQUIRE(o.force > 0.0);
            REQUIRE(o.lock_torque == 0.0);
        }
        const double target = h * h * drive * static_cast<double>(steps * (steps + 1)) / (2.0 * j);
        REQUIRE(close(state.q[0], target, rel));
        provider.set_shaft_torque(0, drive);
        const TransmissionOutput o = provider.transmission_states(state.q, state.v).at(0);
        REQUIRE(o.power_flow == PowerFlow::Locked);
        REQUIRE(o.shaft_load_torque == 0.0);
    }
}

TEST_CASE("lifting a self-locking screw needs n F / eta_f", "[ENG-044][ENG-048]") {
    // (a) T = n m g / 2 < n F: the static gain n leaves n F - T back-driving and the row carries it: theta stays
    //     exactly 0, row torque n m g - T (1e-9).
    // (b) n F < T = n F (1 + (1 / eta_f - 1) / 2) < n F / eta_f: the steady state is the creep where g(s_w) F = T,
    //     s_w = 1 / 2, omega_c = omega_s / 2 exactly (1e-9), load torque T; settling from the linear system with
    //     c = (1 / eta_f - 1) n F / omega_s on the shaft (the rigid mode excluded).
    // (c) T = 1.5 n F / eta_f: the shaft passes omega_s and the load torque is (n / eta_f) F (1e-12).
    for (const double j : {1e-5, 1e-2}) {
        const double n = screw_n();
        const double f = screw_mass * gravity;
        const double eta = 0.4;
        SemiImplicitEuler integrator{StepConfig{}};
        const double h = integrator.config().h;
        {
            MultibodyProvider provider(screw_model(j, true));
            State state = hanging(provider);
            const double drive = 0.5 * n * f;
            for (int i = 0; i < 1000; ++i) {
                provider.set_shaft_torque(0, drive);
                provider.step(integrator, state);
                REQUIRE(state.q[0] == 0.0);
            }
            REQUIRE(close(provider.outputs().transmissions.at(0).lock_torque, n * f - drive, rel));
        }
        {
            MultibodyProvider provider(screw_model(j, true));
            State state = hanging(provider);
            const double drive = n * f * (1.0 + 0.5 * (1.0 / eta - 1.0));
            const Transmission t = self_locking_screw(true);
            const Eigen::Matrix2d ab = coupling_matrix(n, n);
            Eigen::Matrix2d damping = t.damping * ab;
            damping(0, 0) += (1.0 / eta - 1.0) * n * f / omega_s;
            const long settle =
                settling_steps(Eigen::Vector2d(j, screw_mass).asDiagonal(), t.stiffness * ab, damping, h);
            for (long i = 0; i < settle; ++i) {
                provider.set_shaft_torque(0, drive);
                provider.step(integrator, state);
                REQUIRE(state.v[0] >= 0.0);
            }
            REQUIRE(close(state.v[0], 0.5 * omega_s, rel));
            REQUIRE(close(provider.outputs().transmissions.at(0).shaft_load_torque, drive, rel));
        }
        {
            MultibodyProvider provider(screw_model(j, true));
            State state = hanging(provider);
            const double drive = 1.5 * n * f / eta;
            for (int i = 0; i < 200; ++i) {
                provider.set_shaft_torque(0, drive);
                provider.step(integrator, state);
            }
            REQUIRE(state.v[0] > omega_s);
            provider.set_shaft_torque(0, drive);
            const TransmissionOutput o = provider.transmission_states(state.q, state.v).at(0);
            REQUIRE(o.power_flow == PowerFlow::Forward);
            REQUIRE(std::abs(o.shaft_load_torque - n / eta * o.force) <= 1e-12 * std::abs(n / eta * o.force));
        }
    }
}

TEST_CASE("a lossy lead screw without self-locking is back-driven at both inertias", "[ENG-044]") {
    // Same screw with self_locking false: no row, so L = -n m g turns the shaft negative from rest (Backward).
    for (const double j : {1e-5, 1e-2}) {
        MultibodyProvider provider(screw_model(j, false));
        SemiImplicitEuler integrator{StepConfig{}};
        State state = hanging(provider);
        for (int i = 0; i < 100; ++i) {
            provider.step(integrator, state);
            REQUIRE(provider.outputs().transmissions.at(0).lock_torque == 0.0);
        }
        REQUIRE(state.q[0] < 0.0);
        REQUIRE(state.v[0] < -omega_s);
        REQUIRE(provider.outputs().transmissions.at(0).power_flow == PowerFlow::Backward);
    }
}

TEST_CASE("the self-locking row applies only while the screw is in contact", "[ENG-044][ENG-043]") {
    // Backlash b = 1e-4 m, shaft turning at omega0 = -1e-3 rad/s (the back-driving way), carriage released at x = 0
    // inside the dead band: out of contact there is no force, no row and no shaft torque, so omega stays -1e-3
    // exactly while the carriage falls. Once E = n theta - x reaches b / 2 the coupling is in contact, F > 0, the row
    // binds and the shaft stops dead; the carriage settles at x = n theta_hold - b / 2 - m g / k (1e-9).
    for (const double j : {1e-5, 1e-2}) {
        MultibodyModel model = screw_model(j, true);
        const double b = 1e-4;
        model.transmissions[0].backlash = b;
        MultibodyProvider provider(model);
        SemiImplicitEuler integrator{StepConfig{}};
        const double n = screw_n();
        State state;
        state.q = provider.neutral_positions();
        state.v = Eigen::VectorXd::Zero(2);
        state.v[0] = -1e-3;
        int free_steps = 0;
        while (!provider.transmission_states(state.q, state.v).at(0).in_contact) {
            provider.step(integrator, state);
            const TransmissionOutput& o = provider.outputs().transmissions.at(0);
            if (!o.in_contact) {
                REQUIRE(state.v[0] == -1e-3);
                REQUIRE(o.lock_torque == 0.0);
            }
            ++free_steps;
            REQUIRE(free_steps < 1000);
        }
        provider.step(integrator, state);
        REQUIRE(state.v[0] == 0.0);
        const double held = state.q[0];
        for (int i = 0; i < 5000; ++i) {
            provider.step(integrator, state);
            REQUIRE(state.q[0] == held);
        }
        std::printf("  J %g: %d free steps, held at theta %.15g\n", j, free_steps, held);
        REQUIRE(close(state.q[1], n * held - 0.5 * b - screw_mass * gravity / screw_k, rel));
    }
}

TEST_CASE("self-locking screw rows that do not converge within the iteration cap end the run", "[ENG-044][ENG-025]") {
    // Two self-locking screws on two shafts (J = 1e-5) drive one carriage. The two rows couple through S^-1 by the
    // carriage (n^2 m against J), so one Gauss-Seidel sweep leaves the first row off its target: at cap 1 the step
    // throws SolverNonConvergenceError (ENG-025); at the default cap both shafts hold exactly. Efficiencies 1 here:
    // a lossy screw's zero-speed damping c = |alpha| n F / omega_s (about 234 N m s against J) would shrink the
    // coupling below the velocity tolerance in one sweep.
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0, 0, -gravity);
    model.bodies.push_back(shaft(1e-5));
    model.bodies.push_back(shaft(1e-5));
    model.bodies.push_back(slider(screw_mass, Eigen::Vector3d::UnitZ()));
    Transmission first = self_locking_screw(true);
    first.efficiency = 1.0;
    first.back_efficiency = 1.0;
    first.input_body = 0;
    first.output_body = 2;
    Transmission second = first;
    second.input_body = 1;
    model.transmissions = {first, second};
    MultibodyConfig capped;
    capped.iteration_cap = 1;
    MultibodyProvider failing(model, capped);
    SemiImplicitEuler integrator{StepConfig{}};
    State start;
    start.q = failing.neutral_positions();
    start.q[2] = -screw_mass * gravity / (2.0 * screw_k);
    start.v = Eigen::VectorXd::Zero(3);
    State state = start;
    REQUIRE_THROWS_AS(failing.step(integrator, state), SolverNonConvergenceError);
    MultibodyProvider provider(model);
    state = start;
    for (int i = 0; i < 100; ++i) {
        provider.step(integrator, state);
        REQUIRE(state.q[0] == 0.0);
        REQUIRE(state.q[1] == 0.0);
    }
}

TEST_CASE("a self-locking screw lift, stop and hold is bitwise identical when repeated", "[ENG-044][ENG-023]") {
    auto run = []() {
        MultibodyProvider provider(screw_model(1e-5, true));
        SemiImplicitEuler integrator{StepConfig{}};
        State state = hanging(provider);
        std::vector<double> trace;
        for (int i = 0; i < 600; ++i) {
            if (i < 100) {
                provider.set_shaft_torque(0, 0.2);
            }
            provider.step(integrator, state);
            const TransmissionOutput& o = provider.outputs().transmissions.at(0);
            trace.insert(trace.end(), {state.q[0], state.q[1], state.v[0], state.v[1], o.force, o.lock_torque});
        }
        return trace;
    };
    REQUIRE(run() == run());
}

namespace {

// Unloaded screw for ruling 55: gravity off, lead 2 pi / 256 so that n = 1 / 256 exactly (a power of two: n theta
// and x accumulate the same roundings, so a matched motion keeps the stretch, and F, exactly 0), coupling undamped.
MultibodyModel unloaded_screw() {
    MultibodyModel model = screw_model(1e-5, true);
    model.gravity = Eigen::Vector3d::Zero();
    model.transmissions[0].ratio = 2.0 * std::numbers::pi / 256.0;
    model.transmissions[0].damping = 0.0;
    return model;
}

State at_rest(const MultibodyProvider& provider) {
    State state;
    state.q = provider.neutral_positions();
    state.v = Eigen::VectorXd::Zero(provider.velocity_size());
    return state;
}

}  // namespace

TEST_CASE("an unloaded self-locking screw that is moving coasts", "[ENG-044]") {
    // Ruling 55: the two-sided row needs omega = 0 exactly. Shaft at 1 rad/s and carriage at n rad/s: E and dE/dt stay
    // exactly 0, so F = 0, no row and no torque: omega stays 1 bitwise and theta advances h per step.
    MultibodyProvider provider(unloaded_screw());
    REQUIRE(mechsim::engine::output_per_input(provider.model().transmissions[0]) == 1.0 / 256.0);
    SemiImplicitEuler integrator{StepConfig{}};
    State state = at_rest(provider);
    state.v[0] = 1.0;
    state.v[1] = 1.0 / 256.0;
    for (int i = 0; i < 100; ++i) {
        provider.step(integrator, state);
        REQUIRE(provider.outputs().transmissions.at(0).force == 0.0);
        REQUIRE(state.v[0] == 1.0);
        REQUIRE(provider.outputs().transmissions.at(0).lock_torque == 0.0);
    }
    REQUIRE(state.q[0] > 0.09);
}

TEST_CASE("a body-wrench torque turns an unloaded self-locking screw either way", "[ENG-044]") {
    // Ruling 55: T includes apply_body_wrench reduced to the shaft DOF; with T != 0 and F = 0 there is no row, so the
    // shaft turns the way the torque pushes, in both directions.
    for (const double sign : {1.0, -1.0}) {
        MultibodyProvider provider(unloaded_screw());
        SemiImplicitEuler integrator{StepConfig{}};
        State state = at_rest(provider);
        provider.apply_body_wrench(0, Eigen::Vector3d(0.0, 0.0, sign * 1e-3), Eigen::Vector3d::Zero());
        provider.step(integrator, state);
        REQUIRE(sign * state.v[0] > 0.0);
        REQUIRE(provider.outputs().transmissions.at(0).lock_torque == 0.0);
    }
}

TEST_CASE("a model joint spring on the shaft counts as the screw's own torque", "[ENG-044]") {
    // Spring K_s = 1 N m/rad, rest 0.1 rad: own torque +0.1 N m at theta = 0, F = 0, so no row: the shaft turns
    // positive on the first step.
    MultibodyModel model = unloaded_screw();
    model.bodies[0].joint.stiffness = Eigen::VectorXd::Constant(1, 1.0);
    model.bodies[0].joint.damping = Eigen::VectorXd::Zero(1);
    model.bodies[0].joint.rest = Eigen::VectorXd::Constant(1, 0.1);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    State state = at_rest(provider);
    provider.step(integrator, state);
    REQUIRE(state.v[0] > 0.0);
}

TEST_CASE("an unloaded self-locking screw at rest holds when the load arrives later", "[ENG-044]") {
    // 10 steps unloaded at rest (two-sided row, nothing to hold), then a force -m g on the carriage from step 10:
    // that step starts at F = 0, T = 0, omega = 0, so the two-sided row holds the shaft against the end-of-step
    // coupling force; afterwards F > 0 and the one-sided row holds. theta stays exactly 0 throughout.
    MultibodyProvider provider(unloaded_screw());
    SemiImplicitEuler integrator{StepConfig{}};
    State state = at_rest(provider);
    for (int i = 0; i < 1000; ++i) {
        if (i >= 10) {
            provider.set_generalized_forces(Eigen::Vector2d(0.0, -screw_mass * gravity));
        }
        provider.step(integrator, state);
        REQUIRE(state.q[0] == 0.0);
        REQUIRE(state.v[0] == 0.0);
    }
    REQUIRE(state.q[1] < 0.0);
}

TEST_CASE("a self-locking screw lifted between n F and n F / eta_f creeps without ratcheting", "[ENG-044]") {
    // The larger-loss damping at omega = 0 in isolation. Leaving rest with c = max(1 / eta_f - 1, 1 - eta_b) n F /
    // omega_s (1.5 here, the forward side) the first step reaches omega = (T - n F) / (c + J_eff / h) < omega_s, and
    // from then on the forward creep holds omega in (0, omega_s): omega never returns to 0 (the row never re-binds)
    // and theta rises every step. With c = 0 at rest the lossless first step would reach h (T - n F) / J_eff, above
    // omega_s, reverse under n F / eta_f into the row and repeat.
    for (const double j : {1e-5, 1e-2}) {
        MultibodyProvider provider(screw_model(j, true));
        SemiImplicitEuler integrator{StepConfig{}};
        const double n = screw_n();
        const double f = screw_mass * gravity;
        const double drive = n * f * (1.0 + 0.5 * (1.0 / 0.4 - 1.0));
        State state = hanging(provider);
        double previous = state.q[0];
        for (int i = 0; i < 500; ++i) {
            provider.set_shaft_torque(0, drive);
            provider.step(integrator, state);
            REQUIRE(state.v[0] > 0.0);
            REQUIRE(state.v[0] < omega_s);
            REQUIRE(state.q[0] > previous);
            previous = state.q[0];
        }
    }
}
