#include <Eigen/Core>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <string>

#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "multibody_fixtures.hpp"

using mechsim::engine::BeamSpec;
using mechsim::engine::Body;
using mechsim::engine::InvalidModelError;
using mechsim::engine::JointType;
using mechsim::engine::MultibodyConfig;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::SolverNonConvergenceError;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::Transmission;
using mechsim::engine::add_lumped_beam;
using mechsim::engine::test::rod;

// Joint travel limits (ENG-013, ENG-021 hard stop), joint friction (ENG-014, ruling 38) and the per-step joint drive
// (ENG-051, ruling 37). Tolerances are the rows "Joint limit, friction and drive tests" in docs/spec/eng.md.

namespace {

constexpr double g = 9.81;
constexpr double h = 1e-3;
constexpr double inf = std::numeric_limits<double>::infinity();

Eigen::VectorXd one(double x) { return Eigen::VectorXd::Constant(1, x); }

Eigen::VectorXd force(double x) { return Eigen::VectorXd::Constant(1, x); }

Body slider(const Eigen::Vector3d& axis, double mass) {
    Body body;
    body.joint.type = JointType::Prismatic;
    body.joint.axis = axis;
    body.mass = mass;
    return body;
}

MultibodyModel single(const Body& body) {
    MultibodyModel model;
    model.bodies = {body};
    return model;
}

State start(const MultibodyProvider& provider, double q, double v) {
    State state;
    state.q = provider.neutral_positions();
    state.v = Eigen::VectorXd::Zero(provider.velocity_size());
    state.q[0] = q;
    state.v[0] = v;
    return state;
}

bool relative(double measured, double target, double tolerance) {
    std::printf("  target %.15g measured %.15g\n", target, measured);
    return std::abs(measured - target) <= tolerance * std::abs(target);
}

}  // namespace

// ===== Travel limits =====================================================

TEST_CASE("joint limit rest: a mass on its lower limit carries m g and sits on the bound", "[ENG-013]") {
    // Anticipatory row (ruling 43): s v >= -gap / h. The falling step that would pass the bound lands on it
    // (q + h (-gap / h), round-off only), the next starts with gap 0 and the row returns the step's -g h to zero
    // with impulse m g h, so the reaction is m g. No position projection is involved.
    const double m = 2.0;
    Body body = slider(Eigen::Vector3d::UnitZ(), m);
    body.joint.lower = one(0.0);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.05, 0.0);
    for (int i = 0; i < 1000; ++i) {
        provider.step(integrator, state);
        REQUIRE(state.q[0] >= -1e-15);
        REQUIRE(provider.outputs().joints[0].limit_overshoot[0] <= 1e-15);
    }
    REQUIRE(std::abs(state.q[0]) <= 1e-15);
    REQUIRE(std::abs(state.v[0]) <= 1e-12);
    REQUIRE(relative(provider.outputs().joints[0].limit_force[0], m * g, 1e-9));
}

TEST_CASE("joint limit impact lands on the bound, stops the next step, kinetic energy never rises",
          "[ENG-013][ENG-021]") {
    // Horizontal slider, no force along the axis, arriving at V = 0.3 m/s. Free steps keep v bitwise. The engaging
    // step is the first whose unconstrained end q_n - h V would pass the bound: the row takes impulse
    // m (V - q_n / h) and the step ends at v = -q_n / h, exactly on the bound (round-off). The next step starts with
    // gap 0: impulse m q_n / h, v = 0 (zero restitution). Kinetic energy falls at each of the two and never rises.
    const double m = 1.5;
    const double speed = 0.3;
    Body body = slider(Eigen::Vector3d::UnitX(), m);
    body.joint.lower = one(0.0);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.05, -speed);
    int engaged = -1;
    double landing_gap = 0.0;
    for (int i = 0; i < 400; ++i) {
        const double q_before = state.q[0];
        const double energy_before = 0.5 * m * state.v[0] * state.v[0];
        provider.step(integrator, state);
        const double energy_after = 0.5 * m * state.v[0] * state.v[0];
        REQUIRE(energy_after <= energy_before);
        REQUIRE(provider.outputs().joints[0].limit_overshoot[0] <= 1e-15);
        if (engaged < 0 && provider.outputs().joints[0].limit_force[0] > 0.0) {
            engaged = i;
            landing_gap = q_before;
            REQUIRE(q_before < h * speed);
            REQUIRE(std::abs(state.q[0]) <= 1e-15);
            REQUIRE(relative(state.v[0], -q_before / h, 1e-12));
            REQUIRE(relative(provider.outputs().joints[0].limit_force[0], m * (speed - q_before / h) / h, 1e-9));
        } else if (engaged >= 0 && i == engaged + 1) {
            REQUIRE(std::abs(state.v[0]) <= 1e-12);
            REQUIRE(relative(provider.outputs().joints[0].limit_force[0], m * landing_gap / (h * h), 1e-6));
        }
    }
    REQUIRE(engaged >= 0);
    REQUIRE(std::abs(state.q[0]) <= 1e-15);
    REQUIRE(std::abs(state.v[0]) <= 1e-12);
}

TEST_CASE("joint limit crossing with a spring on the joint never raises kinetic plus potential energy",
          "[ENG-013][ENG-051]") {
    // Backward Euler on K with the limit impulse: E_{n+1} - E_n = -(m (dv)^2 + K (dq)^2) / 2 + lambda s dq, and
    // lambda >= 0 acts only while s dq <= 0 (the row lands on the bound), so E never rises; round-off 1e-12 E_0.
    const double m = 1.0;
    const double k = 400.0;
    Body body = slider(Eigen::Vector3d::UnitX(), m);
    body.joint.stiffness = one(k);
    body.joint.rest = one(0.5);
    body.joint.upper = one(0.1);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, -0.2, 0.0);
    auto energy = [&](const State& s) {
        return 0.5 * m * s.v[0] * s.v[0] + 0.5 * k * (s.q[0] - 0.5) * (s.q[0] - 0.5);
    };
    const double e0 = energy(state);
    bool engaged = false;
    for (int i = 0; i < 500; ++i) {
        const double before = energy(state);
        provider.step(integrator, state);
        REQUIRE(energy(state) <= before + 1e-12 * e0);
        if (provider.outputs().joints[0].limit_force[0] < 0.0) {
            engaged = true;
        }
    }
    REQUIRE(engaged);
    REQUIRE(std::abs(state.q[0] - 0.1) <= 1e-15);
}

TEST_CASE("joint limit on a moving belt-coupled carriage never raises the coupled energy", "[ENG-013][ENG-043]") {
    // Shaft J, belt radius n, stiffness k, no damping, carriage m moving rigidly with the shaft (omega = v / n) into
    // an upper stop. Energy J w^2 / 2 + m v^2 / 2 + k (n theta - x)^2 / 2 is quadratic and the coupling enters K
    // whole (eta 1, no slip, a = b), so the step is backward Euler plus a landing impulse: non-increasing to
    // round-off on every step, the crossing step included (the overshoot-and-clamp scheme added up to
    // (omega h)^2 of the coupled inertia's kinetic energy there).
    const double n = 0.02;
    const double j = 1e-4;
    const double k = 1e5;
    const double m = 1.0;
    const double speed = 0.5;
    Body shaft;
    shaft.joint.type = JointType::Revolute;
    shaft.inertia = Eigen::Vector3d(j, j, j).asDiagonal();
    Body carriage = slider(Eigen::Vector3d::UnitX(), m);
    carriage.joint.upper = one(0.0101);
    MultibodyModel model;
    model.bodies = {shaft, carriage};
    Transmission belt;
    belt.input_body = 0;
    belt.output_body = 1;
    belt.ratio = n;
    belt.stiffness = k;
    model.transmissions = {belt};
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, speed / n);
    state.v[1] = speed;
    auto energy = [&](const State& s) {
        const double stretch = n * s.q[0] - s.q[1];
        return 0.5 * j * s.v[0] * s.v[0] + 0.5 * m * s.v[1] * s.v[1] + 0.5 * k * stretch * stretch;
    };
    const double e0 = energy(state);
    bool engaged = false;
    for (int i = 0; i < 100; ++i) {
        const double before = energy(state);
        provider.step(integrator, state);
        const double after = energy(state);
        if (!engaged && provider.outputs().joints[1].limit_force[0] < 0.0) {
            engaged = true;
            std::printf("  crossing step energy before %.15g after %.15g\n", before, after);
        }
        REQUIRE(after <= before + 1e-12 * e0);
        REQUIRE(state.q[1] <= 0.0101 + 1e-15);
    }
    REQUIRE(engaged);
}

TEST_CASE("joint limit start coordinate outside its limits is an invalid model error naming the joint",
          "[ENG-013][ENG-049]") {
    Body body = slider(Eigen::Vector3d::UnitZ(), 1.0);
    body.joint.lower = one(0.0);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, -1e-3, 0.0);
    try {
        provider.step(integrator, state);
        FAIL("accepted");
    } catch (const InvalidModelError& error) {
        REQUIRE(std::string(error.what()).find("body 0") != std::string::npos);
    }
    REQUIRE(state.q[0] == -1e-3);
}

TEST_CASE("joint limit release: a force away from the limit accelerates at (F - m g) / m from the first step",
          "[ENG-013]") {
    // On the bound at rest with F > m g up: the unconstrained increment h (F - m g) / m is positive, so the
    // one-sided row takes no impulse (a bilateral penalty spring would hold it back).
    const double m = 2.0;
    const double lift = 30.0;
    Body body = slider(Eigen::Vector3d::UnitZ(), m);
    body.joint.lower = one(0.0);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    provider.set_generalized_forces(force(lift));
    provider.step(integrator, state);
    REQUIRE(relative(state.v[0], h * (lift - m * g) / m, 1e-12));
    REQUIRE(provider.outputs().joints[0].limit_force[0] == 0.0);
    // A lift below m g leaves it on the bound, the limit carrying the rest: m g - F.
    const double weak = 5.0;
    State held = start(provider, 0.0, 0.0);
    provider.set_generalized_forces(force(weak));
    provider.step(integrator, held);
    REQUIRE(held.q[0] == 0.0);
    REQUIRE(held.v[0] == 0.0);
    REQUIRE(relative(provider.outputs().joints[0].limit_force[0], m * g - weak, 1e-9));
}

TEST_CASE("joint limit with both bounds holds a constant push at either bound", "[ENG-013]") {
    // Constant F along the axis: at rest on the bound it pushes into, the limit force is -F exactly (rounding).
    const double m = 1.0;
    const double push = 3.0;
    Body body = slider(Eigen::Vector3d::UnitX(), m);
    body.joint.lower = one(-0.1);
    body.joint.upper = one(0.1);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.5);
    for (int i = 0; i < 1000; ++i) {
        provider.set_generalized_forces(force(push));
        provider.step(integrator, state);
    }
    REQUIRE(std::abs(state.q[0] - 0.1) <= 1e-15);
    REQUIRE(std::abs(state.v[0]) <= 1e-12);
    REQUIRE(relative(provider.outputs().joints[0].limit_force[0], -push, 1e-9));
    for (int i = 0; i < 1000; ++i) {
        provider.set_generalized_forces(force(-push));
        provider.step(integrator, state);
    }
    REQUIRE(std::abs(state.q[0] + 0.1) <= 1e-15);
    REQUIRE(std::abs(state.v[0]) <= 1e-12);
    REQUIRE(relative(provider.outputs().joints[0].limit_force[0], push, 1e-9));
}

TEST_CASE("joint limit at the start coordinate holds from the first step", "[ENG-013]") {
    const double m = 3.0;
    Body body = slider(Eigen::Vector3d::UnitZ(), m);
    body.joint.lower = one(0.25);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.25, 0.0);
    provider.step(integrator, state);
    REQUIRE(state.q[0] == 0.25);
    REQUIRE(state.v[0] == 0.0);
    REQUIRE(relative(provider.outputs().joints[0].limit_force[0], m * g, 1e-9));
    REQUIRE(provider.outputs().joints[0].limit_overshoot[0] == 0.0);
}

TEST_CASE("joint limit with lower equal to upper locks the joint like a fixed joint", "[ENG-013]") {
    // Both rows active on one coordinate: v stays 0 whichever way the load acts; the limit force balances the
    // net load (m g down, then F - m g up).
    const double m = 1.0;
    Body body = slider(Eigen::Vector3d::UnitZ(), m);
    body.joint.lower = one(0.2);
    body.joint.upper = one(0.2);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.2, 0.0);
    for (int i = 0; i < 10; ++i) {
        provider.step(integrator, state);
    }
    REQUIRE(state.q[0] == 0.2);
    REQUIRE(state.v[0] == 0.0);
    REQUIRE(relative(provider.outputs().joints[0].limit_force[0], m * g, 1e-9));
    const double lift = 100.0;
    for (int i = 0; i < 10; ++i) {
        provider.set_generalized_forces(force(lift));
        provider.step(integrator, state);
    }
    REQUIRE(state.q[0] == 0.2);
    REQUIRE(state.v[0] == 0.0);
    REQUIRE(relative(provider.outputs().joints[0].limit_force[0], -(lift - m * g), 1e-9));
}

TEST_CASE("joint limit open side: -inf lower with a finite upper is one row", "[ENG-013]") {
    Body body = slider(Eigen::Vector3d::UnitZ(), 1.0);
    body.joint.lower = one(-inf);
    body.joint.upper = one(0.0);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    provider.step(integrator, state);
    REQUIRE(state.v[0] == -g * h);
    REQUIRE(provider.outputs().joints[0].limit_force[0] == 0.0);
}

TEST_CASE("joint limit parameters are validated before stepping, naming the joint", "[ENG-013][ENG-049]") {
    auto rejects = [](const Body& body, const std::string& cause) {
        try {
            const MultibodyProvider provider(single(body));
            FAIL("accepted");
        } catch (const InvalidModelError& error) {
            const std::string what = error.what();
            REQUIRE(what.find("body 0") != std::string::npos);
            REQUIRE(what.find(cause) != std::string::npos);
        }
    };
    Body above = slider(Eigen::Vector3d::UnitX(), 1.0);
    above.joint.lower = one(0.2);
    above.joint.upper = one(0.1);
    rejects(above, "lower limit is above the upper limit");
    Body nan = slider(Eigen::Vector3d::UnitX(), 1.0);
    nan.joint.lower = one(std::nan(""));
    rejects(nan, "lower limit");
    Body nan_upper = slider(Eigen::Vector3d::UnitX(), 1.0);
    nan_upper.joint.upper = one(std::nan(""));
    rejects(nan_upper, "upper limit");
    Body plus = slider(Eigen::Vector3d::UnitX(), 1.0);
    plus.joint.lower = one(inf);
    rejects(plus, "lower limit");
    Body minus = slider(Eigen::Vector3d::UnitX(), 1.0);
    minus.joint.upper = one(-inf);
    rejects(minus, "upper limit");
    Body two = slider(Eigen::Vector3d::UnitX(), 1.0);
    two.joint.lower = Eigen::VectorXd::Zero(2);
    rejects(two, "one entry per joint velocity");
    for (const JointType type : {JointType::Spherical, JointType::Planar, JointType::Free, JointType::Fixed,
                                 JointType::Translation}) {
        Body body = slider(Eigen::Vector3d::UnitZ(), 1.0);
        body.joint.type = type;
        body.joint.lower = one(0.0);
        rejects(body, "joint travel limits need a Revolute or Prismatic joint");
        Body upper = slider(Eigen::Vector3d::UnitZ(), 1.0);
        upper.joint.type = type;
        upper.joint.upper = one(0.0);
        rejects(upper, "joint travel limits need a Revolute or Prismatic joint");
    }
}

TEST_CASE("joint limit on a revolute pendulum: it stops at the stop and carries m g l sin(theta_s)",
          "[ENG-013][ENG-021]") {
    // Rod pivot about z, gravity -y, com at l = 0.5 m along the rod: gravity torque -m g l cos(theta). Released
    // horizontal (theta 0), it falls onto a lower stop at theta = -pi / 3, i.e. theta_s = pi / 6 from the downward
    // vertical; at rest (v = 0, no Coriolis term) the stop carries m g l sin(theta_s) = m g l cos(-pi / 3).
    const double m = 1.0;
    const double l = 0.5;
    const double stop = -std::numbers::pi / 3.0;
    Body arm = rod("arm", -1, Eigen::Vector3d::Zero(), 2.0 * l, m);
    arm.joint.lower = one(stop);
    MultibodyModel model = single(arm);
    model.gravity = Eigen::Vector3d(0.0, -g, 0.0);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    for (int i = 0; i < 2000; ++i) {
        provider.step(integrator, state);
    }
    REQUIRE(std::abs(state.q[0] - stop) <= 1e-15);
    REQUIRE(std::abs(state.v[0]) <= 1e-12);
    REQUIRE(relative(provider.outputs().joints[0].limit_force[0], m * g * l * std::sin(std::numbers::pi / 6.0),
                     1e-9));
}

TEST_CASE("joint limit with a joint spring on the same joint carries the spring force", "[ENG-013][ENG-051]") {
    // Spring K toward rest 0.3 past an upper stop at 0.1: at rest the stop carries -K (0.3 - 0.1).
    const double k = 100.0;
    Body body = slider(Eigen::Vector3d::UnitX(), 1.0);
    body.joint.stiffness = one(k);
    body.joint.damping = one(2.0);
    body.joint.rest = one(0.3);
    body.joint.upper = one(0.1);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    for (int i = 0; i < 2000; ++i) {
        provider.step(integrator, state);
    }
    REQUIRE(std::abs(state.q[0] - 0.1) <= 1e-15);
    REQUIRE(std::abs(state.v[0]) <= 1e-12);
    REQUIRE(relative(provider.outputs().joints[0].limit_force[0], -k * 0.2, 1e-9));
}

TEST_CASE("joint limit on a belt's output joint: the reflected row carries the shaft torque over the radius",
          "[ENG-013][ENG-043]") {
    // Shaft torque tau drives the carriage (belt radius n) into an upper stop. Static: shaft tau - n F = 0, carriage
    // F + limit = 0, so the stop carries -tau / n. The row works in the full system matrix (H + h D + h^2 K with the
    // coupling), so the shaft's inertia reaches it through the belt. The pinned-carriage mode is the shaft on the
    // belt spring, J / n^2 at the output: coupling damping 2 sqrt(k J) / n is critical, decay e^-632 per second.
    const double n = 0.02;
    const double j = 1e-4;
    const double k = 1e5;
    const double tau = 0.1;
    Body shaft;
    shaft.joint.type = JointType::Revolute;
    shaft.mass = 1.0;
    shaft.inertia = Eigen::Vector3d(j, j, j).asDiagonal();
    Body carriage = slider(Eigen::Vector3d::UnitX(), 1.0);
    carriage.joint.upper = one(0.02);
    MultibodyModel model;
    model.bodies = {shaft, carriage};
    Transmission belt;
    belt.input_body = 0;
    belt.output_body = 1;
    belt.ratio = n;
    belt.stiffness = k;
    belt.damping = 2.0 * std::sqrt(k * j) / n;
    model.transmissions = {belt};
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    for (int i = 0; i < 3000; ++i) {
        provider.set_shaft_torque(0, tau);
        provider.step(integrator, state);
    }
    REQUIRE(std::abs(state.q[1] - 0.02) <= 1e-15);
    REQUIRE(std::abs(state.v[1]) <= 1e-12);
    REQUIRE(relative(provider.outputs().joints[1].limit_force[0], -tau / n, 1e-9));
}

TEST_CASE("joint limit rows that do not converge within the iteration cap end the run", "[ENG-013][ENG-025]") {
    // Two prismatic joints in series along z, both on their lower stops. The rows couple through H^-1 (off-diagonal
    // -1 / m0), so one Gauss-Seidel sweep leaves a residual; at cap 1 the step throws. At the default cap the sweep
    // contracts by m1 / (m0 + m1) = 1/2 per iteration and converges; the stops then carry (m0 + m1) g and m1 g to
    // within the velocity tolerance: |dF| <= max row sum of H * tolerance / h = 3 m * 1e-9 / 1e-3.
    const double m = 1.0;
    Body lower = slider(Eigen::Vector3d::UnitZ(), m);
    lower.joint.lower = one(0.0);
    Body upper = slider(Eigen::Vector3d::UnitZ(), m);
    upper.parent = 0;
    upper.joint.lower = one(0.0);
    MultibodyModel model;
    model.bodies = {lower, upper};
    MultibodyConfig capped;
    capped.iteration_cap = 1;
    MultibodyProvider failing(model, capped);
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(failing, 0.0, 0.0);
    REQUIRE_THROWS_AS(failing.step(integrator, state), SolverNonConvergenceError);
    MultibodyProvider provider(model);
    State resting = start(provider, 0.0, 0.0);
    for (int i = 0; i < 10; ++i) {
        provider.step(integrator, resting);
    }
    const double bound = 3.0 * m * MultibodyConfig::default_velocity_tolerance / h;
    std::printf("  stops %.15g %.15g\n", provider.outputs().joints[0].limit_force[0],
                provider.outputs().joints[1].limit_force[0]);
    REQUIRE(std::abs(provider.outputs().joints[0].limit_force[0] - 2.0 * m * g) <= bound);
    REQUIRE(std::abs(provider.outputs().joints[1].limit_force[0] - m * g) <= bound);
    REQUIRE(resting.q[0] == 0.0);
    REQUIRE(resting.q[1] == 0.0);
}

TEST_CASE("joint limit runs are bitwise identical on a second run", "[ENG-013][ENG-023]") {
    auto run = []() {
        Body body = slider(Eigen::Vector3d::UnitZ(), 1.0);
        body.joint.lower = one(0.0);
        body.joint.upper = one(0.3);
        body.joint.friction = one(0.5);
        MultibodyProvider provider(single(body));
        SemiImplicitEuler integrator{StepConfig{}};
        State state = start(provider, 0.1, 2.0);
        for (int i = 0; i < 500; ++i) {
            provider.step(integrator, state);
        }
        return state;
    };
    const State a = run();
    const State b = run();
    REQUIRE(a.q == b.q);
    REQUIRE(a.v == b.v);
}

// ===== Joint friction ====================================================

TEST_CASE("joint friction sliding: stopping time and distance of the frozen-rate scheme", "[ENG-014]") {
    // c = F / |v_n| frozen at the step start, implicit: v_{n+1} = v_n^2 / (v_n + a h), a = F / m, so the
    // deceleration is a v_{n+1} / v_n (first order in h). With N the first step at or below v_s:
    //   time   t* <= N h <= t* + h (1 + ln(v0 (v_s + a h) / v_s^2)),  t* = (v0 - v_s) / a
    //   (sum of decrements v0 - v_N = a h sum(v_{n+1} / v_n), 1 - v_{n+1}/v_n <= ln(v_n / v_{n+1}),
    //    v_N > v_s^2 / (v_s + a h));
    //   distance x_N = h sum v_n in [D, D + h (v0 - v_N) / 2], D = (v0^2 - v_N^2) / (2 a)
    //   (v_n^2 - v_{n+1}^2 = a h v_{n+1} (1 + r_n), r_n = v_{n+1} / v_n, and v_{n+1} (1 - r_n) <= v_n - v_{n+1}),
    // so x_N is within h v0 / 2 + v_s^2 / (2 a) of the Coulomb distance v0^2 m / (2 F).
    const double m = 2.0;
    const double friction = 4.0;
    const double v0 = 1.0;
    const double a = friction / m;
    const double vs = MultibodyConfig::default_friction_slip_threshold;
    Body body = slider(Eigen::Vector3d::UnitX(), m);
    body.joint.friction = one(friction);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, v0);
    long steps = 0;
    while (state.v[0] > vs) {
        const double before = state.v[0];
        provider.step(integrator, state);
        ++steps;
        REQUIRE(std::abs(state.v[0] - before * before / (before + a * h)) <= 1e-14);
        REQUIRE(steps < 10000);
    }
    const double time = static_cast<double>(steps) * h;
    const double coulomb_time = (v0 - vs) / a;
    std::printf("  stopping time target %.15g measured %.15g\n", coulomb_time, time);
    REQUIRE(time >= coulomb_time);
    REQUIRE(time <= coulomb_time + h * (1.0 + std::log(v0 * (vs + a * h) / (vs * vs))));
    const double vn = state.v[0];
    const double distance = (v0 * v0 - vn * vn) / (2.0 * a);
    std::printf("  stopping distance target %.15g measured %.15g\n", v0 * v0 * m / (2.0 * friction), state.q[0]);
    REQUIRE(state.q[0] >= distance - 1e-12);
    REQUIRE(state.q[0] <= distance + h * (v0 - vn) / 2.0 + 1e-12);
    REQUIRE(std::abs(state.q[0] - v0 * v0 * m / (2.0 * friction)) <= h * v0 / 2.0 + vs * vs / (2.0 * a));
}

TEST_CASE("joint friction hold with creep: a load below F creeps at v_s L / F", "[ENG-014]") {
    // Below v_s the friction is the damper F / v_s; its fixed point under load L is v* = v_s L / F (ruling 38), and
    // the step contracts the error by m / (m + h F / v_s) = 1 / 21 here, so 40 steps leave 21^-40 of it.
    const double m = 1.0;
    const double friction = 2.0;
    const double vs = MultibodyConfig::default_friction_slip_threshold;
    for (const double load : {1.5, -0.5}) {
        Body body = slider(Eigen::Vector3d::UnitX(), m);
        body.joint.friction = one(friction);
        MultibodyProvider provider(single(body));
        SemiImplicitEuler integrator{StepConfig{}};
        State state = start(provider, 0.0, 0.0);
        for (int i = 0; i < 40; ++i) {
            provider.set_generalized_forces(force(load));
            provider.step(integrator, state);
        }
        const double creep = vs * load / friction;
        REQUIRE(relative(state.v[0], creep, 1e-9));
        REQUIRE(relative(provider.outputs().joints[0].friction_creep[0], std::abs(creep), 1e-9));
    }
}

TEST_CASE("joint friction under a load above F accelerates at (L - F) / m above v_s", "[ENG-014]") {
    // Above v_s: v_{n+1} = (m v_n + h L) / (m + h F / v_n), an acceleration (L - F) / m / (1 + h F / (m v_n)).
    // Over 1 s from v0 the speed therefore lies between v0 + (L - F) t / (m (1 + h F / (m v0))) and
    // v0 + (L - F) t / m; creep output is zero above v_s.
    const double m = 1.0;
    const double friction = 2.0;
    const double load = 3.0;
    const double v0 = 1.0;
    Body body = slider(Eigen::Vector3d::UnitX(), m);
    body.joint.friction = one(friction);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, v0);
    for (int i = 0; i < 1000; ++i) {
        const double before = state.v[0];
        provider.set_generalized_forces(force(load));
        provider.step(integrator, state);
        const double expected = (m * before + h * load) / (m + h * friction / before);
        REQUIRE(std::abs(state.v[0] - expected) <= 1e-14 * expected);
    }
    const double t = 1.0;
    const double gain = (load - friction) * t / m;
    std::printf("  v(1 s) Coulomb %.15g measured %.15g\n", v0 + gain, state.v[0]);
    REQUIRE(state.v[0] <= v0 + gain);
    REQUIRE(state.v[0] >= v0 + gain / (1.0 + h * friction / (m * v0)));
    REQUIRE(provider.outputs().joints[0].friction_creep[0] == 0.0);
}

TEST_CASE("joint friction zero equals the frictionless run bitwise", "[ENG-014]") {
    auto run = [](bool zero_entry) {
        Body body = slider(Eigen::Vector3d::UnitZ(), 1.0);
        body.joint.stiffness = one(50.0);
        if (zero_entry) {
            body.joint.friction = one(0.0);
        }
        MultibodyProvider provider(single(body));
        SemiImplicitEuler integrator{StepConfig{}};
        State state = start(provider, 0.1, 0.3);
        for (int i = 0; i < 300; ++i) {
            provider.set_generalized_forces(force(1.0));
            provider.step(integrator, state);
        }
        return state;
    };
    const State with = run(true);
    const State without = run(false);
    REQUIRE(with.q == without.q);
    REQUIRE(with.v == without.v);
}

TEST_CASE("joint friction values are validated, naming the joint", "[ENG-014][ENG-049]") {
    auto rejects = [](const Body& body, const std::string& cause) {
        try {
            const MultibodyProvider provider(single(body));
            FAIL("accepted");
        } catch (const InvalidModelError& error) {
            const std::string what = error.what();
            REQUIRE(what.find("body 0") != std::string::npos);
            REQUIRE(what.find(cause) != std::string::npos);
        }
    };
    Body negative = slider(Eigen::Vector3d::UnitX(), 1.0);
    negative.joint.friction = one(-1.0);
    rejects(negative, "joint friction must not be negative");
    Body nan = slider(Eigen::Vector3d::UnitX(), 1.0);
    nan.joint.friction = one(std::nan(""));
    rejects(nan, "joint friction is not finite");
    Body infinite = slider(Eigen::Vector3d::UnitX(), 1.0);
    infinite.joint.friction = one(inf);
    rejects(infinite, "joint friction is not finite");
    Body sized = slider(Eigen::Vector3d::UnitX(), 1.0);
    sized.joint.friction = Eigen::VectorXd::Zero(2);
    rejects(sized, "joint friction must be empty or have one entry per joint velocity");
    Body ball = slider(Eigen::Vector3d::UnitX(), 1.0);
    ball.joint.type = JointType::Spherical;
    ball.joint.friction = Eigen::VectorXd::Ones(3);
    rejects(ball, "joint friction needs a Revolute, Prismatic or Planar joint");
}

// ===== Joint drive =======================================================

TEST_CASE("joint drive adds to the model spring: static position balances both springs and the load", "[ENG-051]") {
    // Model spring K_m 50 toward r_m 0, drive K_d 400 toward r_d 0.1, load tau 2: at rest
    // K_m (q - r_m) + K_d (q - r_d) = tau, q = (tau + K_m r_m + K_d r_d) / (K_m + K_d). Damping D_m 2 + D_d 40 on
    // m 1: zeta = 42 / (2 sqrt(450)) = 0.99, so 3 s leave below e^-60 of the start error.
    const double km = 50.0;
    const double kd = 400.0;
    const double tau = 2.0;
    Body body = slider(Eigen::Vector3d::UnitX(), 1.0);
    body.joint.stiffness = one(km);
    body.joint.damping = one(2.0);
    MultibodyProvider provider(single(body));
    provider.set_joint_drive(0, 0, 0.1, kd, 40.0);
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    for (int i = 0; i < 3000; ++i) {
        provider.set_generalized_forces(force(tau));
        provider.step(integrator, state);
    }
    REQUIRE(relative(state.q[0], (tau + kd * 0.1) / (km + kd), 1e-9));
    REQUIRE(provider.outputs().events.empty());
}

TEST_CASE("joint drive damping adds to the model damping: decay rate with D_m + D_d", "[ENG-051]") {
    // No springs: (m + h (D_m + D_d)) v_{n+1} = m v_n, so each step multiplies v by m / (m + h (D_m + D_d)).
    const double m = 2.0;
    const double dm = 3.0;
    const double dd = 5.0;
    Body body = slider(Eigen::Vector3d::UnitX(), m);
    body.joint.damping = one(dm);
    MultibodyProvider provider(single(body));
    provider.set_joint_drive(0, 0, 0.0, 0.0, dd);
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 1.0);
    for (int i = 0; i < 10; ++i) {
        const double before = state.v[0];
        provider.step(integrator, state);
        REQUIRE(relative(state.v[0], before * m / (m + h * (dm + dd)), 1e-14));
    }
}

TEST_CASE("joint drive clear restores the undriven response bitwise", "[ENG-051]") {
    // Driven for 100 steps, then cleared: from that state the run must equal a never-driven provider's bitwise.
    Body body = slider(Eigen::Vector3d::UnitX(), 1.0);
    body.joint.stiffness = one(50.0);
    body.joint.damping = one(2.0);
    body.joint.rest = one(0.05);
    MultibodyProvider driven(single(body));
    MultibodyProvider plain(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State a = start(driven, 0.0, 0.0);
    driven.set_joint_drive(0, 0, 0.3, 400.0, 10.0);
    for (int i = 0; i < 100; ++i) {
        driven.step(integrator, a);
    }
    driven.clear_joint_drive(0, 0);
    State b = a;
    for (int i = 0; i < 300; ++i) {
        driven.set_generalized_forces(force(1.0));
        driven.step(integrator, a);
        plain.set_generalized_forces(force(1.0));
        plain.step(integrator, b);
    }
    REQUIRE(a.q == b.q);
    REQUIRE(a.v == b.v);
    REQUIRE(driven.outputs().joints[0].scheme_decay.size() == 0);
}

TEST_CASE("joint drive set, cleared and set again acts as the last drive only", "[ENG-051]") {
    // Static with the model spring K_m 50 at 0: drive 1 (K 400, rest 0.1), clear, drive 2 (K 200, rest -0.2):
    // q = K_2 r_2 / (K_m + K_2) = -0.16, not the sum of both drives.
    Body body = slider(Eigen::Vector3d::UnitX(), 1.0);
    body.joint.stiffness = one(50.0);
    body.joint.damping = one(40.0);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    provider.set_joint_drive(0, 0, 0.1, 400.0, 0.0);
    for (int i = 0; i < 500; ++i) {
        provider.step(integrator, state);
    }
    provider.clear_joint_drive(0, 0);
    provider.set_joint_drive(0, 0, -0.2, 200.0, 0.0);
    for (int i = 0; i < 5000; ++i) {
        provider.step(integrator, state);
    }
    REQUIRE(relative(state.q[0], 200.0 * -0.2 / 250.0, 1e-9));
    provider.set_joint_drive(0, 0, 0.4, 200.0, 0.0);
    for (int i = 0; i < 5000; ++i) {
        provider.step(integrator, state);
    }
    REQUIRE(relative(state.q[0], 200.0 * 0.4 / 250.0, 1e-9));
}

TEST_CASE("joint drive steady lag is D v / K behind a rest moving at constant speed", "[ENG-051]") {
    // Rest set each step to its end-of-step value r_{n+1} = v (t_n + h). At constant speed v the implicit balance
    // 0 = -K (q_{n+1} - r_{n+1}) - D v gives q - r = -D v / K exactly (a start-of-step rest would add -h v).
    const double k = 400.0;
    const double d = 40.0;
    const double speed = 0.2;
    const Body body = slider(Eigen::Vector3d::UnitX(), 1.0);
    MultibodyProvider provider(single(body));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    double rest = 0.0;
    for (int i = 0; i < 3000; ++i) {
        rest = speed * (state.t + h);
        provider.set_joint_drive(0, 0, rest, k, d);
        provider.step(integrator, state);
    }
    REQUIRE(relative(state.q[0] - rest, -d * speed / k, 1e-9));
    REQUIRE(relative(state.v[0], speed, 1e-9));
}

TEST_CASE("joint drive publishes the backward-Euler scheme decay pi omega h, driven DOFs only", "[ENG-051]") {
    // Ruling 44: omega^2 = (K_model + K_drive) (H^-1)_kk; a rotor of inertia I alone has (H^-1)_kk = 1 / I.
    const double inertia = 0.02;
    const double km = 10.0;
    const double k = 50.0;
    Body rotor;
    rotor.joint.type = JointType::Revolute;
    rotor.joint.stiffness = one(km);
    rotor.inertia = Eigen::Vector3d(inertia, inertia, inertia).asDiagonal();
    MultibodyProvider provider(single(rotor));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    provider.step(integrator, state);
    REQUIRE(provider.outputs().joints[0].scheme_decay.size() == 0);
    provider.set_joint_drive(0, 0, 0.0, k, 0.0);
    provider.step(integrator, state);
    REQUIRE(relative(provider.outputs().joints[0].scheme_decay[0],
                     std::numbers::pi * std::sqrt((km + k) / inertia) * h, 1e-12));
}

TEST_CASE("joint drive scheme decay on a belt's shaft uses the coupled pair's lower root", "[ENG-051][ENG-043]") {
    // Shaft J with drive K_d, belt (ratio n, stiffness k) to carriage m. Pair stiffness [[K_d + k n^2, -k n],
    // [-k n, k]], mass diag(J, m): J m w^2 - (J k + m (K_d + k n^2)) w + K_d k = 0, lower root w. As k grows
    // w -> K_d / (J + n^2 m): K_d / J alone overstates omega by sqrt(1 + n^2 m / J) = sqrt(5) = 2.24 here.
    const double n = 0.02;
    const double j = 1e-4;
    const double k = 1e5;
    const double m = 1.0;
    const double kd = 1.0;
    Body shaft;
    shaft.joint.type = JointType::Revolute;
    shaft.inertia = Eigen::Vector3d(j, j, j).asDiagonal();
    MultibodyModel model;
    model.bodies = {shaft, slider(Eigen::Vector3d::UnitX(), m)};
    Transmission belt;
    belt.input_body = 0;
    belt.output_body = 1;
    belt.ratio = n;
    belt.stiffness = k;
    model.transmissions = {belt};
    MultibodyProvider provider(model);
    provider.set_joint_drive(0, 0, 0.0, kd, 0.0);
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    provider.step(integrator, state);
    const double b = j * k + m * (kd + k * n * n);
    const double w = (b - std::sqrt(b * b - 4.0 * j * m * kd * k)) / (2.0 * j * m);
    const double target = std::numbers::pi * std::sqrt(w) * h;
    const double decay = provider.outputs().joints[0].scheme_decay[0];
    std::printf("  single-DOF omega over pair omega %.6g\n", std::sqrt(kd / j) / std::sqrt(w));
    REQUIRE(relative(decay, target, 1e-12));
    REQUIRE(provider.outputs().joints[1].scheme_decay.size() == 0);
}

TEST_CASE("joint drive scheme decay is absent on an undriven beam model", "[ENG-051][ENG-022]") {
    BeamSpec spec;
    spec.youngs_modulus = 2.0e11;
    spec.area_moment = 1e-8 / 12.0;
    spec.density = 7850.0;
    spec.area = 1e-4;
    spec.length = 1.5;
    MultibodyModel model;
    add_lumped_beam(model, spec, -1, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    State state;
    state.q = provider.neutral_positions();
    state.v = Eigen::VectorXd::Zero(provider.velocity_size());
    provider.step(integrator, state);
    for (const auto& joint : provider.outputs().joints) {
        REQUIRE(joint.scheme_decay.size() == 0);
    }
}

TEST_CASE("joint drive on a belt's shaft acts on the reflected coordinate", "[ENG-051][ENG-043]") {
    // Drive K_d on the shaft, load F_L on the carriage, belt radius n, stiffness k. Static: carriage F + F_L = 0;
    // shaft -K_d theta - n F = 0, so theta = n F_L / K_d and the carriage sits at n theta + F_L / k (the drive
    // reflected to the output is K_d / n^2 in series with the belt). Drive damping makes the slow mode about
    // critical (omega ~ 45 rad/s), the belt damping 2 sqrt(k mu) the fast one; 5 s leave below e^-150.
    const double n = 0.02;
    const double j = 1e-4;
    const double k = 1e5;
    const double kd = 1.0;
    const double load = 3.0;
    Body shaft;
    shaft.joint.type = JointType::Revolute;
    shaft.inertia = Eigen::Vector3d(j, j, j).asDiagonal();
    MultibodyModel model;
    model.bodies = {shaft, slider(Eigen::Vector3d::UnitX(), 1.0)};
    Transmission belt;
    belt.input_body = 0;
    belt.output_body = 1;
    belt.ratio = n;
    belt.stiffness = k;
    const double mu = 1.0 / (1.0 + n * n / j);
    belt.damping = 2.0 * std::sqrt(k * mu);
    model.transmissions = {belt};
    MultibodyProvider provider(model);
    provider.set_joint_drive(0, 0, 0.0, kd, 0.045);
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(2);
    tau[1] = load;
    for (int i = 0; i < 5000; ++i) {
        provider.set_generalized_forces(tau);
        provider.step(integrator, state);
    }
    REQUIRE(relative(state.q[0], n * load / kd, 1e-9));
    REQUIRE(relative(state.q[1], n * n * load / kd + load / k, 1e-9));
}

TEST_CASE("joint drive on a belt's driven output joint sags F / K_d and the shaft follows through the ratio",
          "[ENG-051][ENG-043]") {
    // Drive K_d toward r_d on the carriage, free shaft, load F_L on the carriage. The free shaft passes no force at
    // rest (F = 0, stretch 0), so x = r_d + F_L / K_d and theta = x / n. Slow mode: K_d on m + J / n^2 = 1.25 kg,
    // drive damping 2 sqrt(K_d 1.25) critical; belt damping as in the shaft case; 5 s leave below e^-100.
    const double n = 0.02;
    const double j = 1e-4;
    const double k = 1e5;
    const double kd = 400.0;
    const double rest = 0.05;
    const double load = 3.0;
    Body shaft;
    shaft.joint.type = JointType::Revolute;
    shaft.inertia = Eigen::Vector3d(j, j, j).asDiagonal();
    MultibodyModel model;
    model.bodies = {shaft, slider(Eigen::Vector3d::UnitX(), 1.0)};
    Transmission belt;
    belt.input_body = 0;
    belt.output_body = 1;
    belt.ratio = n;
    belt.stiffness = k;
    belt.damping = 2.0 * std::sqrt(k / (1.0 + n * n / j));
    model.transmissions = {belt};
    MultibodyProvider provider(model);
    provider.set_joint_drive(1, 0, rest, kd, 2.0 * std::sqrt(kd * 1.25));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = start(provider, 0.0, 0.0);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(2);
    tau[1] = load;
    for (int i = 0; i < 5000; ++i) {
        provider.set_generalized_forces(tau);
        provider.step(integrator, state);
    }
    REQUIRE(relative(state.q[1], rest + load / kd, 1e-9));
    REQUIRE(relative(state.q[0], (rest + load / kd) / n, 1e-9));
}

TEST_CASE("joint drive with zero stiffness and damping leaves the run bitwise unchanged", "[ENG-051]") {
    auto run = [](bool drive) {
        Body body = slider(Eigen::Vector3d::UnitX(), 2.0);
        body.joint.stiffness = one(100.0);
        MultibodyProvider provider(single(body));
        if (drive) {
            provider.set_joint_drive(0, 0, 0.3, 0.0, 0.0);
        }
        SemiImplicitEuler integrator{StepConfig{}};
        State state = start(provider, 0.5, 0.0);
        for (int i = 0; i < 100; ++i) {
            provider.set_generalized_forces(force(3.0));
            provider.step(integrator, state);
        }
        return state;
    };
    const State with = run(true);
    const State without = run(false);
    REQUIRE(with.q == without.q);
    REQUIRE(with.v == without.v);
}

TEST_CASE("joint drive arguments are validated with typed errors", "[ENG-051]") {
    MultibodyModel model;
    Body fixed;
    fixed.joint.type = JointType::Fixed;
    Body ball;
    ball.joint.type = JointType::Spherical;
    Body free;
    free.joint.type = JointType::Free;
    Body planar;
    planar.joint.type = JointType::Planar;
    model.bodies = {slider(Eigen::Vector3d::UnitX(), 1.0), fixed, ball, free, planar};
    MultibodyProvider provider(model);
    REQUIRE_THROWS_AS(provider.set_joint_drive(-1, 0, 0.0, 1.0, 1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_joint_drive(5, 0, 0.0, 1.0, 1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_joint_drive(0, 1, 0.0, 1.0, 1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_joint_drive(0, -1, 0.0, 1.0, 1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_joint_drive(4, 3, 0.0, 1.0, 1.0), InvalidModelError);
    REQUIRE_NOTHROW(provider.set_joint_drive(4, 2, 0.0, 1.0, 1.0));
    for (const Eigen::Index joint : {1, 2, 3}) {
        REQUIRE_THROWS_AS(provider.set_joint_drive(joint, 0, 0.0, 1.0, 1.0), InvalidModelError);
    }
    REQUIRE_THROWS_AS(provider.set_joint_drive(0, 0, 0.0, -1.0, 1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_joint_drive(0, 0, 0.0, 1.0, -1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_joint_drive(0, 0, std::nan(""), 1.0, 1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_joint_drive(0, 0, inf, 1.0, 1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_joint_drive(0, 0, 0.0, inf, 1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_joint_drive(0, 0, 0.0, 1.0, std::nan("")), InvalidModelError);
}
