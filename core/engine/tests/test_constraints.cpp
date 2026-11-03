#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <vector>

#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "multibody_fixtures.hpp"

using mechsim::engine::Body;
using mechsim::engine::ConstraintError;
using mechsim::engine::ConstraintType;
using mechsim::engine::DriftError;
using mechsim::engine::JointType;
using mechsim::engine::LoopConstraint;
using mechsim::engine::MultibodyConfig;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyOutputs;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::StepHealth;
using mechsim::engine::test::crank_rocker;
using mechsim::engine::test::four_bar_closure;
using mechsim::engine::test::four_bar_model;
using mechsim::engine::test::FourBar;
using mechsim::engine::test::rod;
using mechsim::engine::test::standard_gravity;

namespace {

State four_bar_state(const FourBar& bar, double crank_angle) {
    return {four_bar_closure(bar, crank_angle), Eigen::VectorXd::Zero(3), 0.0};
}

// Two rods of length 1 m pivoting at the world origin. Constraint 0 joins the pivots (always satisfied, no
// reachable motion: zero Jacobian rows); constraint 1 joins the tips, welding the rods together.
MultibodyModel welded_pair() {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, -standard_gravity, 0.0);
    model.bodies.push_back(rod("a", -1, Eigen::Vector3d::Zero(), 1.0, 1.0));
    model.bodies.push_back(rod("b", -1, Eigen::Vector3d::Zero(), 1.0, 1.0));
    LoopConstraint pivots;
    pivots.body_a = 0;
    pivots.body_b = 1;
    LoopConstraint tips = pivots;
    tips.point_a = Eigen::Vector3d::UnitX();
    tips.point_b = Eigen::Vector3d::UnitX();
    model.constraints = {pivots, tips};
    return model;
}

}  // namespace

TEST_CASE("four-bar closed by a point constraint stays below the drift threshold over 2 000 steps",
          "[ENG-012][ENG-024]") {
    const FourBar bar = crank_rocker();
    MultibodyProvider provider(four_bar_model(bar, Eigen::Vector3d::Zero()));
    SemiImplicitEuler integrator(StepConfig{});
    State state = four_bar_state(bar, 0.5 * std::numbers::pi);
    REQUIRE(provider.constraint_errors(state.q).front().translational <= 1e-15);
    const double coupler_start = state.q[0] + state.q[1];
    const Eigen::Vector3d tip_start = provider.body_states(state.q, state.v)[1].position;
    constexpr double torque = 2.5e-2;
    constexpr int steps = 2000;
    double worst = 0.0;
    double lowest = coupler_start;
    double highest = coupler_start;
    for (int n = 0; n < steps; ++n) {
        provider.set_generalized_forces(Eigen::Vector3d(torque, 0.0, 0.0));
        const StepHealth health = provider.step(integrator, state);
        REQUIRE(health.iterations >= 1);
        REQUIRE(health.iterations <= MultibodyConfig::default_iteration_cap);
        worst = std::max(worst, health.max_constraint_drift);
        lowest = std::min(lowest, state.q[0] + state.q[1]);
        highest = std::max(highest, state.q[0] + state.q[1]);
    }
    WARN("four-bar worst drift after projection " << worst << " m, crank turned " << state.q[0] - 0.5 * std::numbers::pi
                                                   << " rad, coupler swing " << highest - lowest << " rad");
    CHECK(worst <= MultibodyConfig::default_drift_threshold_m);
    CHECK(state.q[0] - 0.5 * std::numbers::pi > 0.1);
    CHECK(highest - lowest > 0.1);
    CHECK((provider.outputs().bodies[1].position - tip_start).norm() > 0.01);
    CHECK(state.v[0] > 0.0);
}

TEST_CASE("with a 1e-15 drift threshold the step whose residual exceeds it throws a drift error", "[ENG-012][ENG-024]") {
    // One Gauss-Newton iteration leaves a residual of order e^2 (e the pre-projection error, of order h^2 a), which
    // sits far above 1e-15; a reference run with a loose threshold finds the first such step.
    const FourBar bar = crank_rocker();
    const MultibodyModel model = four_bar_model(bar, Eigen::Vector3d(0.0, -standard_gravity, 0.0));
    MultibodyConfig loose;
    loose.projection_iterations = 1;
    loose.drift_threshold_m = 1.0;
    MultibodyConfig tight = loose;
    tight.drift_threshold_m = 1e-15;
    MultibodyProvider reference(model, loose);
    MultibodyProvider provider(model, tight);
    SemiImplicitEuler integrator(StepConfig{});
    State expected = four_bar_state(bar, 1.0);
    double failing_time = -1.0;
    constexpr int steps = 1000;
    for (int n = 0; n < steps && failing_time < 0.0; ++n) {
        const double t = expected.t;
        const StepHealth health = reference.step(integrator, expected);
        if (health.max_constraint_drift > 1e-15) {
            failing_time = t;
        }
    }
    REQUIRE(failing_time >= 0.0);
    State state = four_bar_state(bar, 1.0);
    bool thrown = false;
    for (int n = 0; n < steps && !thrown; ++n) {
        try {
            provider.step(integrator, state);
        } catch (const DriftError& error) {
            thrown = true;
            CHECK(error.time() == failing_time);
            CHECK(error.constraint_index() == 0);
            CHECK(state.t == failing_time);
        }
    }
    CHECK(thrown);
}

TEST_CASE("drift measurement equals the hand-computed residual and the error names time and constraint",
          "[ENG-024]") {
    // Rods of length l at angles a and b: tips l (cos, sin); |x_a - x_b| = 2 l sin(|a - b| / 2).
    const MultibodyModel model = welded_pair();
    constexpr double angle = 0.3;
    constexpr double offset = 1e-3;
    const Eigen::Vector2d q(angle, angle + offset);
    MultibodyConfig unprojected;
    unprojected.projection_iterations = 0;
    MultibodyProvider provider(model, unprojected);
    const std::vector<ConstraintError> errors = provider.constraint_errors(q);
    REQUIRE(errors.size() == 2);
    CHECK(errors[0].translational == 0.0);
    const Eigen::Vector3d hand(std::cos(angle) - std::cos(angle + offset), std::sin(angle) - std::sin(angle + offset),
                               0.0);
    CHECK((errors[1].residual - hand).cwiseAbs().maxCoeff() <= 1e-16);
    CHECK(std::abs(errors[1].translational - 2.0 * std::sin(0.5 * offset)) <= 1e-16);
    CHECK(errors[1].rotational == 0.0);

    SemiImplicitEuler integrator(StepConfig{});
    State state{q, Eigen::VectorXd::Zero(2), 0.25};
    try {
        provider.step(integrator, state);
        FAIL("no DriftError raised");
    } catch (const DriftError& error) {
        CHECK(error.time() == 0.25);
        CHECK(error.constraint_index() == 1);
        CHECK(error.cause() == "constraint 1 drift exceeds the threshold");
    }
    CHECK(state.q == q);

    MultibodyProvider projected(model);
    const StepHealth health = projected.step(integrator, state);
    CHECK(health.max_constraint_drift <= MultibodyConfig::default_drift_threshold_m);
    CHECK(projected.outputs().constraints[1].translational_error == health.max_constraint_drift);
}

TEST_CASE("hinge constraint holds a free body as a pendulum on the hinge axis", "[ENG-012][ENG-024]") {
    // Body 0 is welded to the world; body 1 is free and hinged to it about z at the origin. The velocity solve
    // removes the out-of-axis rotation, so |omega x z| is bounded by |omega| times the rotational drift plus the
    // velocity tolerance.
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, -standard_gravity, 0.0);
    Body base;
    base.joint.type = JointType::Fixed;
    model.bodies.push_back(base);
    Body arm;
    arm.joint.type = JointType::Free;
    arm.mass = 0.5;
    arm.com = Eigen::Vector3d(0.4, 0.0, 0.0);
    arm.inertia = Eigen::Vector3d(0.001, 0.01, 0.01).asDiagonal();
    model.bodies.push_back(arm);
    LoopConstraint hinge;
    hinge.type = ConstraintType::Hinge;
    hinge.body_a = 0;
    hinge.body_b = 1;
    model.constraints.push_back(hinge);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(StepConfig{});
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(6), 0.0};
    state.v.head<3>() << 0.3, 0.2, 1.0;
    constexpr int steps = 1000;
    for (int n = 0; n < steps; ++n) {
        const StepHealth health = provider.step(integrator, state);
        const MultibodyOutputs& out = provider.outputs();
        REQUIRE(out.constraints.size() == 1);
        REQUIRE(out.constraints[0].impulse.size() == 5);
        CHECK(out.constraints[0].translational_error <= MultibodyConfig::default_drift_threshold_m);
        CHECK(out.constraints[0].rotational_error <= MultibodyConfig::default_drift_threshold_rad);
        const Eigen::Vector3d omega = out.bodies[1].spatial_velocity.head<3>();
        CHECK(omega.cross(Eigen::Vector3d::UnitZ()).norm() <=
              omega.norm() * MultibodyConfig::default_drift_threshold_rad + MultibodyConfig::default_velocity_tolerance);
        CHECK(health.max_constraint_drift <= MultibodyConfig::default_drift_threshold_m);
    }
    const auto& swung = provider.outputs().bodies[1];
    const Eigen::Vector3d com = swung.position + swung.orientation * arm.com;
    CHECK(std::abs(com.norm() - 0.4) <= MultibodyConfig::default_drift_threshold_m + 1e-12);
    CHECK(swung.orientation.angularDistance(Eigen::Quaterniond::Identity()) > 0.1);
}

TEST_CASE("four-bar constraint and joint forces match the hand statics of a parallelogram", "[ENG-027]") {
    // Parallelogram a = c = 0.3 m, b = d = 0.5 m with vertical crank and rocker and a horizontal coupler, gravity -y:
    // an equilibrium with zero crank torque. Rocker moments about its pivot: weight and the coupler force both act
    // along the vertical rocker, so the closure force has no x part. Coupler moments about its left end:
    // R b - m_c g b / 2 = 0, so the closure force on the coupler is (0, m_c g / 2, 0). The world holds the rocker
    // with (0, (m_r + m_c / 2) g, 0) and no moment; the crank joint carries zero torque.
    const FourBar bar{0.3, 0.5, 0.3, 0.5, 2.0};
    MultibodyProvider provider(four_bar_model(bar, Eigen::Vector3d(0.0, -standard_gravity, 0.0)));
    SemiImplicitEuler integrator(StepConfig{});
    const double half_pi = 0.5 * std::numbers::pi;
    State state{Eigen::Vector3d(half_pi, -half_pi, half_pi), Eigen::VectorXd::Zero(3), 0.0};
    provider.step(integrator, state);
    const MultibodyOutputs& out = provider.outputs();
    const double coupler_weight = bar.density * bar.coupler * standard_gravity;
    const double rocker_weight = bar.density * bar.rocker * standard_gravity;
    const Eigen::Vector3d closure(0.0, 0.5 * coupler_weight, 0.0);
    REQUIRE(out.constraints.size() == 1);
    CHECK((out.constraints[0].force - closure).cwiseAbs().maxCoeff() <= 1e-9 * coupler_weight);
    CHECK((out.constraints[0].impulse - integrator.config().h * closure).cwiseAbs().maxCoeff() <=
          1e-9 * coupler_weight * integrator.config().h);
    REQUIRE(out.joints.size() == 3);
    CHECK(std::abs(out.joints[0].generalized_force[0]) <= 1e-9 * coupler_weight);
    CHECK(std::abs(out.joints[2].generalized_force[0]) <= 1e-9 * coupler_weight);
    const Eigen::Vector3d rocker_force(0.0, rocker_weight + 0.5 * coupler_weight, 0.0);
    CHECK((out.joints[2].reaction_wrench.tail<3>() - rocker_force).cwiseAbs().maxCoeff() <= 1e-9 * coupler_weight);
    CHECK(out.joints[2].reaction_wrench.head<3>().cwiseAbs().maxCoeff() <= 1e-9 * coupler_weight);
    CHECK(state.v.cwiseAbs().maxCoeff() <= 1e-12);
    CHECK(out.contacts.empty());
    CHECK(out.time == state.t);
}

TEST_CASE("joint generalized force output equals the applied joint force", "[ENG-027][ENG-028]") {
    // RNEA at the step's mean acceleration (v' - v) / h with the constraint forces lambda / h returns exactly the
    // generalized force the step applied: H dv = h (tau - c) + J^T lambda.
    const FourBar bar = crank_rocker();
    MultibodyProvider provider(four_bar_model(bar, Eigen::Vector3d(0.0, -standard_gravity, 0.0)));
    SemiImplicitEuler integrator(StepConfig{});
    State state = four_bar_state(bar, 0.8);
    const Eigen::Vector3d tau(0.02, -0.01, 0.005);
    provider.set_generalized_forces(tau);
    provider.step(integrator, state);
    for (Eigen::Index i = 0; i < 3; ++i) {
        CHECK(std::abs(provider.outputs().joints[static_cast<std::size_t>(i)].generalized_force[0] - tau[i]) <= 1e-9);
    }
}

TEST_CASE("inputs apply for one step only", "[ENG-028]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, 0.0, -standard_gravity);
    Body puck;
    puck.joint.type = JointType::Free;
    puck.mass = 2.0;
    puck.com = Eigen::Vector3d(0.1, 0.0, 0.0);
    puck.inertia = Eigen::Vector3d(0.01, 0.02, 0.03).asDiagonal();
    model.bodies.push_back(puck);
    model.bodies.push_back(rod("arm", 0, Eigen::Vector3d(0.2, 0.0, 0.0), 0.5, 1.0));
    MultibodyProvider provider(model);
    MultibodyProvider fresh(model);
    SemiImplicitEuler integrator(StepConfig{});
    const State start{provider.neutral_positions(), Eigen::VectorXd::Zero(7), 0.0};

    State unforced = start;
    fresh.step(integrator, unforced);

    State forced = start;
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(7);
    tau[6] = 0.3;
    provider.set_generalized_forces(tau);
    provider.apply_body_wrench(0, Eigen::Vector3d(0.0, 0.0, 0.5), Eigen::Vector3d(4.0, 0.0, 0.0));
    provider.step(integrator, forced);
    CHECK(forced.v != unforced.v);
    CHECK(forced.v[6] != unforced.v[6]);

    State expected = forced;
    fresh.step(integrator, expected);
    provider.step(integrator, forced);
    CHECK(forced.q == expected.q);
    CHECK(forced.v == expected.v);
    CHECK(forced.t == expected.t);
}

TEST_CASE("four-bar run twice is bitwise identical", "[ENG-012]") {
    const FourBar bar = crank_rocker();
    const MultibodyModel model = four_bar_model(bar, Eigen::Vector3d(0.0, -standard_gravity, 0.0));
    const auto run = [&]() {
        MultibodyProvider provider(model);
        SemiImplicitEuler integrator(StepConfig{});
        State state = four_bar_state(bar, 0.5);
        std::vector<double> trace;
        constexpr int steps = 1000;
        for (int n = 0; n < steps; ++n) {
            provider.set_generalized_forces(Eigen::Vector3d(0.01, 0.0, 0.0));
            const StepHealth health = provider.step(integrator, state);
            trace.push_back(health.max_constraint_drift);
            const MultibodyOutputs& out = provider.outputs();
            for (const auto& body : out.bodies) {
                trace.insert(trace.end(), body.position.data(), body.position.data() + 3);
                trace.insert(trace.end(), body.orientation.coeffs().data(), body.orientation.coeffs().data() + 4);
                trace.insert(trace.end(), body.spatial_velocity.data(), body.spatial_velocity.data() + 6);
            }
            for (const auto& joint : out.joints) {
                trace.insert(trace.end(), joint.reaction_wrench.data(), joint.reaction_wrench.data() + 6);
            }
            trace.insert(trace.end(), out.constraints[0].impulse.data(), out.constraints[0].impulse.data() + 3);
        }
        trace.insert(trace.end(), state.q.data(), state.q.data() + state.q.size());
        trace.insert(trace.end(), state.v.data(), state.v.data() + state.v.size());
        return trace;
    };
    const std::vector<double> first = run();
    const std::vector<double> second = run();
    REQUIRE(first.size() == second.size());
    CHECK(std::memcmp(first.data(), second.data(), first.size() * sizeof(double)) == 0);
}

TEST_CASE("contact outputs are present and empty until contact lands", "[ENG-027]") {
    const FourBar bar = crank_rocker();
    MultibodyProvider provider(four_bar_model(bar, Eigen::Vector3d(0.0, -standard_gravity, 0.0)));
    SemiImplicitEuler integrator(StepConfig{});
    State state = four_bar_state(bar, 1.0);
    provider.step(integrator, state);
    CHECK(provider.outputs().contacts.empty());
    CHECK(provider.outputs().bodies.size() == 3);
}
