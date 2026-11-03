#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <string>

#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "multibody_fixtures.hpp"
#include "providers.hpp"

using mechsim::engine::Body;
using mechsim::engine::ConstraintType;
using mechsim::engine::InvalidConfigError;
using mechsim::engine::InvalidModelError;
using mechsim::engine::JointType;
using mechsim::engine::LoopConstraint;
using mechsim::engine::MultibodyConfig;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::NonFiniteStateError;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::SolverNonConvergenceError;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::StepHealth;
using mechsim::engine::test::LinearProvider;
using mechsim::engine::test::rod;
using mechsim::engine::test::standard_gravity;

namespace {

constexpr double not_a_number = std::numeric_limits<double>::quiet_NaN();

MultibodyModel pendulum() {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, -standard_gravity, 0.0);
    model.bodies.push_back(rod("arm", -1, Eigen::Vector3d::Zero(), 1.0, 1.0));
    return model;
}

MultibodyModel pair_with(const LoopConstraint& constraint) {
    MultibodyModel model = pendulum();
    model.bodies.push_back(rod("second", -1, Eigen::Vector3d::Zero(), 1.0, 1.0));
    model.constraints.push_back(constraint);
    return model;
}

template <typename Error>
void expect_cause(const MultibodyModel& model, const MultibodyConfig& config, const std::string& cause) {
    try {
        const MultibodyProvider provider(model, config);
        FAIL("no error raised for " << cause);
    } catch (const Error& error) {
        CHECK(error.cause() == cause);
        CHECK(error.time() == 0.0);
    }
}

void expect_model(const MultibodyModel& model, const std::string& cause) {
    expect_cause<InvalidModelError>(model, MultibodyConfig{}, cause);
}

}  // namespace

TEST_CASE("a tree with one body steps", "[ENG-005][ENG-006]") {
    MultibodyProvider provider(pendulum());
    CHECK(provider.position_size() == 1);
    CHECK(provider.velocity_size() == 1);
    SemiImplicitEuler integrator(StepConfig{});
    State state{Eigen::VectorXd::Constant(1, 0.2), Eigen::VectorXd::Zero(1), 0.0};
    const StepHealth health = provider.step(integrator, state);
    CHECK(health.iterations == 0);
    CHECK(health.max_constraint_drift == 0.0);
    CHECK(state.v[0] < 0.0);
    CHECK(provider.outputs().bodies.size() == 1);
    CHECK(provider.outputs().constraints.empty());
}

TEST_CASE("invalid bodies and joints are rejected before stepping", "[ENG-005][ENG-006][ENG-049]") {
    SECTION("unnormalized joint axis") {
        MultibodyModel model = pendulum();
        model.bodies[0].joint.axis = Eigen::Vector3d(0.0, 0.0, 1.001);
        expect_model(model, "body 0: joint axis is not a unit vector");
    }
    SECTION("unnormalized planar normal") {
        MultibodyModel model = pendulum();
        model.bodies[0].joint.type = JointType::Planar;
        model.bodies[0].joint.axis = Eigen::Vector3d(1.0, 1.0, 0.0);
        expect_model(model, "body 0: joint axis is not a unit vector");
    }
    SECTION("unused axis of a spherical joint is not checked") {
        MultibodyModel model = pendulum();
        model.bodies[0].joint.type = JointType::Spherical;
        model.bodies[0].joint.axis = Eigen::Vector3d::Zero();
        const MultibodyProvider provider(model);
        CHECK(provider.position_size() == 4);
    }
    SECTION("non-finite mass") {
        MultibodyModel model = pendulum();
        model.bodies[0].mass = std::numeric_limits<double>::infinity();
        expect_model(model, "body 0: mass must be finite and positive");
        model.bodies[0].mass = not_a_number;
        expect_model(model, "body 0: mass must be finite and positive");
    }
    SECTION("zero mass") {
        MultibodyModel model = pendulum();
        model.bodies[0].mass = 0.0;
        expect_model(model, "body 0: mass must be finite and positive");
    }
    SECTION("parent that is not an earlier body") {
        MultibodyModel model = pendulum();
        model.bodies[0].parent = 0;
        expect_model(model, "body 0: parent must be the world (-1) or an earlier body");
        model.bodies[0].parent = -2;
        expect_model(model, "body 0: parent must be the world (-1) or an earlier body");
    }
    SECTION("non-finite center of mass") {
        MultibodyModel model = pendulum();
        model.bodies[0].com.x() = not_a_number;
        expect_model(model, "body 0: center of mass is not finite");
    }
    SECTION("non-finite inertia") {
        MultibodyModel model = pendulum();
        model.bodies[0].inertia(1, 1) = not_a_number;
        expect_model(model, "body 0: inertia is not finite");
    }
    SECTION("asymmetric inertia") {
        MultibodyModel model = pendulum();
        model.bodies[0].inertia(0, 1) = 0.01;
        expect_model(model, "body 0: inertia is not symmetric");
    }
    SECTION("non-finite joint position") {
        MultibodyModel model = pendulum();
        model.bodies[0].joint.position.y() = not_a_number;
        expect_model(model, "body 0: joint position is not finite");
    }
    SECTION("non-unit joint orientation") {
        MultibodyModel model = pendulum();
        model.bodies[0].joint.orientation = Eigen::Quaterniond(1.0, 0.1, 0.0, 0.0);
        expect_model(model, "body 0: joint orientation is not a unit quaternion");
    }
    SECTION("non-finite gravity") {
        MultibodyModel model = pendulum();
        model.gravity.z() = not_a_number;
        expect_model(model, "gravity is not finite");
    }
}

TEST_CASE("invalid loop constraints are rejected before stepping", "[ENG-012][ENG-049]") {
    LoopConstraint constraint;
    constraint.body_a = 0;
    constraint.body_b = 1;
    SECTION("missing body") {
        LoopConstraint missing = constraint;
        missing.body_b = 2;
        expect_model(pair_with(missing), "constraint 0: references a missing body");
        missing.body_b = -1;
        expect_model(pair_with(missing), "constraint 0: references a missing body");
        missing = constraint;
        missing.body_a = 5;
        expect_model(pair_with(missing), "constraint 0: references a missing body");
    }
    SECTION("same body twice") {
        LoopConstraint self = constraint;
        self.body_b = 0;
        expect_model(pair_with(self), "constraint 0: joins a body to itself");
    }
    SECTION("non-finite point") {
        LoopConstraint bad = constraint;
        bad.point_b.x() = not_a_number;
        expect_model(pair_with(bad), "constraint 0: point is not finite");
    }
    SECTION("unnormalized hinge axis") {
        LoopConstraint hinge = constraint;
        hinge.type = ConstraintType::Hinge;
        hinge.axis_b = Eigen::Vector3d(0.0, 0.5, 0.5);
        expect_model(pair_with(hinge), "constraint 0: hinge axis is not a unit vector");
    }
}

TEST_CASE("invalid constraint solver configuration is rejected", "[ENG-012][ENG-024][ENG-050]") {
    const MultibodyModel model = pendulum();
    MultibodyConfig config;
    SECTION("iteration cap zero") {
        config.iteration_cap = 0;
        expect_cause<InvalidConfigError>(model, config, "constraint iteration cap must be at least 1");
    }
    SECTION("negative projection iterations") {
        config.projection_iterations = -1;
        expect_cause<InvalidConfigError>(model, config, "projection iterations must be non-negative");
    }
    SECTION("non-positive or non-finite thresholds") {
        config.drift_threshold_m = 0.0;
        expect_cause<InvalidConfigError>(model, config, "translational drift threshold must be finite and positive");
        config = MultibodyConfig{};
        config.drift_threshold_rad = not_a_number;
        expect_cause<InvalidConfigError>(model, config, "rotational drift threshold must be finite and positive");
        config = MultibodyConfig{};
        config.velocity_tolerance = -1.0;
        expect_cause<InvalidConfigError>(model, config, "constraint velocity tolerance must be finite and positive");
        config = MultibodyConfig{};
        config.unit_tolerance = std::numeric_limits<double>::infinity();
        expect_cause<InvalidConfigError>(model, config, "unit tolerance must be finite and positive");
    }
}

TEST_CASE("the constraint solver throws at its iteration cap with the step time", "[ENG-012][ENG-025]") {
    // A free body pinned at two points to a welded base: the two point blocks share the body's motion, so one
    // Gauss-Seidel sweep leaves the first block's residual disturbed by the second.
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, -standard_gravity, 0.0);
    Body base;
    base.joint.type = JointType::Fixed;
    model.bodies.push_back(base);
    Body plate;
    plate.joint.type = JointType::Free;
    plate.com = Eigen::Vector3d(0.3, 0.1, 0.0);
    plate.inertia = Eigen::Vector3d(0.01, 0.02, 0.03).asDiagonal();
    model.bodies.push_back(plate);
    LoopConstraint first;
    first.body_a = 1;
    first.body_b = 0;
    LoopConstraint second = first;
    second.point_a = Eigen::Vector3d(0.1, -0.2, 0.2);
    second.point_b = second.point_a;
    model.constraints = {first, second};
    MultibodyConfig config;
    config.iteration_cap = 1;
    MultibodyProvider provider(model, config);
    SemiImplicitEuler integrator(StepConfig{});
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(6), 1.5};
    try {
        provider.step(integrator, state);
        FAIL("no SolverNonConvergenceError raised");
    } catch (const SolverNonConvergenceError& error) {
        CHECK(error.time() == 1.5);
        CHECK(error.cause() == "constraint solver did not converge within 1 iterations");
    }
    CHECK(state.t == 1.5);
    MultibodyProvider converging(model);
    const StepHealth health = converging.step(integrator, state);
    CHECK(health.iterations > 1);
    CHECK(health.iterations <= MultibodyConfig::default_iteration_cap);
}

TEST_CASE("invalid inputs are rejected", "[ENG-028]") {
    MultibodyProvider provider(pendulum());
    CHECK_THROWS_AS(provider.set_generalized_forces(Eigen::VectorXd::Zero(2)), InvalidModelError);
    CHECK_THROWS_AS(provider.set_generalized_forces(Eigen::VectorXd::Constant(1, not_a_number)), InvalidModelError);
    CHECK_THROWS_AS(provider.apply_body_wrench(1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
                    InvalidModelError);
    CHECK_THROWS_AS(provider.apply_body_wrench(-1, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
                    InvalidModelError);
    CHECK_THROWS_AS(provider.apply_body_wrench(0, Eigen::Vector3d::Constant(not_a_number), Eigen::Vector3d::Zero()),
                    InvalidModelError);
}

TEST_CASE("mass matrix symmetry is checked relative to its size", "[ENG-049]") {
    SemiImplicitEuler integrator(StepConfig{});
    State state{Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), 0.0};
    SECTION("asymmetry within the relative tolerance passes") {
        Eigen::MatrixXd mass(2, 2);
        mass << 2.0e6, 1.0e6, 1.0e6 + 1e-7, 3.0e6;
        const LinearProvider provider(mass, Eigen::MatrixXd::Zero(2, 2), Eigen::MatrixXd::Zero(2, 2),
                                      Eigen::VectorXd::Ones(2), Eigen::VectorXd::Zero(2));
        CHECK_NOTHROW(integrator.step(provider, state));
    }
    SECTION("asymmetry past the relative tolerance is rejected") {
        Eigen::MatrixXd mass(2, 2);
        mass << 2.0, 1.0, 1.0 + 1e-10, 3.0;
        const LinearProvider provider(mass, Eigen::MatrixXd::Zero(2, 2), Eigen::MatrixXd::Zero(2, 2),
                                      Eigen::VectorXd::Ones(2), Eigen::VectorXd::Zero(2));
        CHECK_THROWS_AS(integrator.step(provider, state), InvalidModelError);
    }
    SECTION("a non-finite mass matrix reports a non-finite state") {
        Eigen::MatrixXd mass(2, 2);
        mass << 2.0, not_a_number, not_a_number, 3.0;
        const LinearProvider provider(mass, Eigen::MatrixXd::Zero(2, 2), Eigen::MatrixXd::Zero(2, 2),
                                      Eigen::VectorXd::Ones(2), Eigen::VectorXd::Zero(2));
        try {
            integrator.step(provider, state);
            FAIL("no NonFiniteStateError raised");
        } catch (const NonFiniteStateError& error) {
            CHECK(error.cause() == "mass matrix is not finite");
        }
    }
}

TEST_CASE("an invalid symmetry tolerance is rejected", "[ENG-050]") {
    StepConfig config;
    config.symmetry_tolerance = -1.0;
    CHECK_THROWS_AS(SemiImplicitEuler(config), InvalidConfigError);
    config.symmetry_tolerance = not_a_number;
    CHECK_THROWS_AS(SemiImplicitEuler(config), InvalidConfigError);
    config.symmetry_tolerance = 0.0;
    CHECK_NOTHROW(SemiImplicitEuler(config));
}
