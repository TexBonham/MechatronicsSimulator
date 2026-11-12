#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

#include "beam_helpers.hpp"
#include "mechsim/engine/dynamics_provider.hpp"
#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "multibody_fixtures.hpp"

using mechsim::engine::BeamEnds;
using mechsim::engine::BeamPoint;
using mechsim::engine::BeamSpec;
using mechsim::engine::ConstraintType;
using mechsim::engine::DynamicsTerms;
using mechsim::engine::InvalidModelError;
using mechsim::engine::JointType;
using mechsim::engine::LumpedBeam;
using mechsim::engine::member_nodes;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::rod;
using mechsim::engine::test::steel_bar;
using mechsim::engine::test::world_point;

namespace {

double total_mass(const MultibodyModel& model, const LumpedBeam& beam) {
    double sum = 0.0;
    for (const Eigen::Index body : beam.bodies) {
        sum += model.bodies[static_cast<std::size_t>(body)].mass;
    }
    return sum;
}

}  // namespace

TEST_CASE("cantilever beam has N midpoint spring nodes of E I / l on the clamped stub", "[ENG-022]") {
    const int segments = GENERATE(1, 4, 8);
    BeamSpec spec = steel_bar(1.2, BeamEnds::Cantilever, segments);
    spec.node_damping = 0.25;
    MultibodyModel model;
    const LumpedBeam beam = add_lumped_beam(model, spec);
    const double l = spec.length / segments;
    REQUIRE(beam.bodies.size() == static_cast<std::size_t>(segments));
    CHECK(model.bodies.size() == beam.bodies.size());
    CHECK(model.constraints.empty());
    CHECK(beam.roller == -1);
    CHECK(beam.roller_constraint == -1);
    CHECK(beam.node_stiffness == spec.youngs_modulus * spec.area_moment / l);
    for (std::size_t j = 0; j < beam.bodies.size(); ++j) {
        const auto& body = model.bodies[static_cast<std::size_t>(beam.bodies[j])];
        CHECK(std::abs(beam.starts[j] - (static_cast<double>(j) + 0.5) * l) <= 1e-15);
        CHECK(body.joint.type == JointType::Revolute);
        CHECK(body.joint.stiffness.size() == 1);
        CHECK(body.joint.stiffness[0] == beam.node_stiffness);
        CHECK(body.joint.damping[0] == 0.25);
        CHECK(body.parent == static_cast<Eigen::Index>(j) - 1);
    }
    // The stub [0, l/2] stays with the clamp: the moving mass is rho A (L - l/2).
    const double expected_mass = spec.density * spec.area * (spec.length - 0.5 * l);
    CHECK(std::abs(total_mass(model, beam) - expected_mass) <= 1e-12 * expected_mass);
    const auto& last = model.bodies.back();
    CHECK(std::abs(last.com.x() - 0.25 * l) <= 1e-15);
    CHECK(std::abs(last.inertia(2, 2) - last.mass * 0.25 * l * l / 12.0) <= 1e-12 * last.inertia(2, 2));
    CHECK_NOTHROW(MultibodyProvider(model));
}

TEST_CASE("simply supported beam has a free pin, N spring nodes and a roller loop on the far end", "[ENG-022]") {
    const int segments = GENERATE(1, 4, 8);
    const BeamSpec spec = steel_bar(1.5, BeamEnds::SimplySupported, segments);
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    const LumpedBeam beam = add_lumped_beam(model, spec);
    const double l = spec.length / segments;
    REQUIRE(beam.bodies.size() == static_cast<std::size_t>(segments) + 1);
    const auto& pin = model.bodies[static_cast<std::size_t>(beam.bodies.front())];
    CHECK(pin.joint.stiffness.size() == 0);
    CHECK(pin.parent == -1);
    CHECK(beam.starts.front() == 0.0);
    for (std::size_t j = 1; j < beam.bodies.size(); ++j) {
        CHECK(std::abs(beam.starts[j] - (static_cast<double>(j) - 0.5) * l) <= 1e-15);
        CHECK(model.bodies[static_cast<std::size_t>(beam.bodies[j])].joint.stiffness[0] == beam.node_stiffness);
    }
    const double member_mass = spec.density * spec.area * spec.length;
    CHECK(std::abs(total_mass(model, beam) - member_mass) <= 1e-12 * member_mass);
    REQUIRE(beam.roller == static_cast<Eigen::Index>(model.bodies.size()) - 1);
    const auto& roller = model.bodies.back();
    CHECK(roller.joint.type == JointType::Prismatic);
    CHECK(roller.joint.axis == Eigen::Vector3d::UnitX());
    CHECK(spec.roller_mass_fraction == 1e-3);
    CHECK(std::abs(roller.mass - spec.roller_mass_fraction * member_mass) <= 1e-15 * member_mass);
    REQUIRE(beam.roller_constraint == 0);
    CHECK(model.constraints[0].type == ConstraintType::Point);
    CHECK(model.constraints[0].body_a == beam.bodies.back());
    CHECK(model.constraints[0].body_b == beam.roller);

    MultibodyProvider provider(model);
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    for (const auto& error : provider.constraint_errors(state.q)) {
        CHECK(error.translational <= 1e-15);
    }
    SemiImplicitEuler integrator(StepConfig{});
    provider.step(integrator, state);
    CHECK(state.q.cwiseAbs().maxCoeff() == 0.0);
}

TEST_CASE("clamped beam has N spring nodes on clamped stubs and a hinge roller lock on the far end",
          "[ENG-022][ENG-030]") {
    const int segments = GENERATE(1, 4, 8);
    const BeamSpec spec = steel_bar(1.5, BeamEnds::Clamped, segments);
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    const LumpedBeam beam = add_lumped_beam(model, spec);
    const double l = spec.length / segments;
    REQUIRE(beam.bodies.size() == static_cast<std::size_t>(segments));
    CHECK_FALSE(beam.pinned_root);
    CHECK(member_nodes(beam) == beam.bodies);
    CHECK(model.bodies[static_cast<std::size_t>(beam.bodies.front())].parent == -1);
    for (std::size_t j = 0; j < beam.bodies.size(); ++j) {
        CHECK(std::abs(beam.starts[j] - (static_cast<double>(j) + 0.5) * l) <= 1e-15);
        CHECK(model.bodies[static_cast<std::size_t>(beam.bodies[j])].joint.stiffness[0] == beam.node_stiffness);
    }
    const double member_mass = spec.density * spec.area * spec.length;
    CHECK(std::abs(total_mass(model, beam) - (1.0 - 0.5 / segments) * member_mass) <= 1e-12 * member_mass);
    REQUIRE(beam.roller == static_cast<Eigen::Index>(model.bodies.size()) - 1);
    const auto& roller = model.bodies.back();
    CHECK(roller.joint.type == JointType::Prismatic);
    CHECK(roller.joint.axis == Eigen::Vector3d::UnitX());
    CHECK(std::abs(roller.joint.position.x() - spec.length) <= 1e-15);
    REQUIRE(beam.roller_constraint == 0);
    const auto& lock = model.constraints[0];
    CHECK(lock.type == ConstraintType::Hinge);
    CHECK(lock.body_a == beam.bodies.back());
    CHECK(lock.body_b == beam.roller);
    CHECK(lock.axis_a == Eigen::Vector3d::UnitX());
    CHECK(lock.axis_b == Eigen::Vector3d::UnitX());
    CHECK(std::abs(lock.point_a.x() - 0.5 * l) <= 1e-15);

    MultibodyProvider provider(model);
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    for (const auto& error : provider.constraint_errors(state.q)) {
        CHECK(error.translational <= 1e-15);
        CHECK(error.rotational <= 1e-15);
    }
    SemiImplicitEuler integrator(StepConfig{});
    provider.step(integrator, state);
    CHECK(state.q.cwiseAbs().maxCoeff() == 0.0);
    // A rotation of the last segment alone breaks the lock: the hinge rows see the z rotation.
    Eigen::VectorXd turned = state.q;
    turned[provider.position_offset(beam.bodies.back())] = 1e-3;
    CHECK(std::abs(provider.constraint_errors(turned)[0].rotational - 1e-3) <= 1e-12);
}

TEST_CASE("beam on a parent body follows the root frame and beam_point maps material points", "[ENG-022]") {
    const BeamEnds ends = GENERATE(BeamEnds::Cantilever, BeamEnds::SimplySupported);
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    model.bodies.push_back(rod("carrier", -1, Eigen::Vector3d(0.3, 0.1, 0.0), 1.0, 2.0));
    const Eigen::Vector3d position(0.2, -0.4, 0.05);
    const Eigen::Quaterniond orientation(Eigen::AngleAxisd(0.5 * std::numbers::pi, Eigen::Vector3d::UnitX()));
    const BeamSpec spec = steel_bar(1.0, ends, 4);
    const LumpedBeam beam = add_lumped_beam(model, spec, 0, position, orientation);
    const MultibodyProvider provider(model);
    const State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    for (const double x : {0.0, 0.1, 0.125, 0.3, 0.5, 0.875, 0.95, 1.0}) {
        const BeamPoint point = beam_point(beam, x);
        const Eigen::Vector3d expected =
            Eigen::Vector3d(0.3, 0.1, 0.0) + position + orientation * Eigen::Vector3d(x, 0, 0);
        Eigen::Vector3d actual = world_point(provider, state, point);
        if (point.body == 0) {
            actual = Eigen::Vector3d(0.3, 0.1, 0.0) + point.point;
        }
        CAPTURE(x, point.body);
        CHECK((actual - expected).norm() <= 1e-14);
    }
    if (ends == BeamEnds::Cantilever) {
        CHECK(beam_point(beam, 0.1).body == 0);
        CHECK(beam_point(beam, 0.125).body == beam.bodies.front());
    }
    CHECK(beam_point(beam, 1.0).body == beam.bodies.back());
    CHECK_THROWS_AS(beam_point(beam, -1e-12), InvalidModelError);
    CHECK_THROWS_AS(beam_point(beam, 1.0 + 1e-12), InvalidModelError);
    CHECK_THROWS_AS(beam_point(beam, std::numeric_limits<double>::quiet_NaN()), InvalidModelError);
}

TEST_CASE("beam nodes return their springs and dampers through K and D", "[ENG-022]") {
    BeamSpec spec = steel_bar(1.0, BeamEnds::Cantilever, 4);
    spec.node_damping = 0.75;
    MultibodyModel model;
    const LumpedBeam beam = add_lumped_beam(model, spec);
    const MultibodyProvider provider(model);
    DynamicsTerms terms;
    provider.evaluate(0.0, provider.neutral_positions(), Eigen::VectorXd::Zero(4), terms);
    const Eigen::MatrixXd k = Eigen::VectorXd::Constant(4, beam.node_stiffness).asDiagonal();
    const Eigen::MatrixXd d = Eigen::VectorXd::Constant(4, 0.75).asDiagonal();
    CHECK(terms.stiffness == k);
    CHECK(terms.damping == d);
}

TEST_CASE("invalid beam parameters are rejected with the cause", "[ENG-022]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    struct Case {
        std::string name;
        BeamSpec spec;
        Eigen::Index parent;
        Eigen::Vector3d position;
        std::string cause;
    };
    const BeamSpec good = steel_bar(1.0, BeamEnds::Cantilever, 4);
    const auto with = [&good](auto change) {
        BeamSpec spec = good;
        change(spec);
        return spec;
    };
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    const std::vector<Case> cases{
        {"E zero", with([](BeamSpec& s) { s.youngs_modulus = 0.0; }), -1, zero, "Young's modulus"},
        {"E NaN", with([nan](BeamSpec& s) { s.youngs_modulus = nan; }), -1, zero, "Young's modulus"},
        {"I negative", with([](BeamSpec& s) { s.area_moment = -1.0; }), -1, zero, "area moment"},
        {"rho infinite", with([inf](BeamSpec& s) { s.density = inf; }), -1, zero, "density"},
        {"A zero", with([](BeamSpec& s) { s.area = 0.0; }), -1, zero, "area must"},
        {"L negative", with([](BeamSpec& s) { s.length = -1.0; }), -1, zero, "length"},
        {"no segments", with([](BeamSpec& s) { s.segments = 0; }), -1, zero, "segment count"},
        {"damping negative", with([](BeamSpec& s) { s.node_damping = -1.0; }), -1, zero, "node damping"},
        {"damping NaN", with([nan](BeamSpec& s) { s.node_damping = nan; }), -1, zero, "node damping"},
        {"roller fraction zero", with([](BeamSpec& s) { s.roller_mass_fraction = 0.0; }), -1, zero, "roller mass"},
        {"roller fraction NaN", with([nan](BeamSpec& s) { s.roller_mass_fraction = nan; }), -1, zero, "roller mass"},
        {"parent missing", good, 0, zero, "parent"},
        {"parent below world", good, -2, zero, "parent"},
        {"position NaN", good, -1, Eigen::Vector3d(nan, 0.0, 0.0), "root position"},
    };
    for (const Case& c : cases) {
        DYNAMIC_SECTION(c.name) {
            MultibodyModel model;
            try {
                add_lumped_beam(model, c.spec, c.parent, c.position);
                FAIL("no error");
            } catch (const InvalidModelError& error) {
                const std::string what = error.what();
                CAPTURE(what);
                CHECK(what.find("beam: ") != std::string::npos);
                CHECK(what.find(c.cause) != std::string::npos);
                CHECK(model.bodies.empty());
            }
        }
    }
    MultibodyModel model;
    CHECK(BeamSpec{}.segments == 4);
    CHECK_THROWS_AS(add_lumped_beam(model, BeamSpec{}), InvalidModelError);
}
