#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "mechsim/engine/dynamics_provider.hpp"
#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "multibody_fixtures.hpp"

using mechsim::engine::Body;
using mechsim::engine::DynamicsTerms;
using mechsim::engine::InvalidModelError;
using mechsim::engine::JointType;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::rod;

// Joint springs carry ENG-051; the lumped beam (ENG-022) is the requirement that needs them, its nodes being
// revolute joints with springs and dampers that the integrator must treat implicitly (ADR 0013).

namespace {

Eigen::VectorXd values(std::initializer_list<double> list) {
    Eigen::VectorXd out(static_cast<Eigen::Index>(list.size()));
    Eigen::Index i = 0;
    for (const double x : list) {
        out[i] = x;
        ++i;
    }
    return out;
}

Body rotor(double inertia) {
    Body body;
    body.joint.type = JointType::Revolute;
    body.mass = 1.0;
    body.inertia = Eigen::Vector3d(inertia, inertia, inertia).asDiagonal();
    return body;
}

}  // namespace

TEST_CASE("stiff revolute spring-damper at omega h = 20 settles to its rest angle as package 1 proved", "[ENG-051]") {
    // I theta'' = -k (theta - rest) - d theta' with k = I omega^2, d = 2 zeta I omega, Omega = omega h = 20,
    // zeta = 0.5. K and D implicit: (I + h d + h^2 k) v' = I v - h k e, e' = e + h v' with e = theta - rest, so
    // [e', h v'] = A [e, h v], A = [[1 - Omega^2 / s, 1 / s], [-Omega^2 / s, 1 / s]], s = 1 + 2 zeta Omega +
    // Omega^2 = 421: det A = 1 / s, |lambda| = s^(-1/2) = 0.0487 per step. Package 1 bounds (ENG-003): overshoot at
    // most 1 percent of the step and settled to 1e-6 of it within 100 steps; the trajectory equals A^n e0.
    const StepConfig config;
    const double h = config.h;
    const double inertia = 0.02;
    const double omega = 20.0 / h;
    const double zeta = 0.5;
    const double rest = 0.3;
    MultibodyModel model;
    Body body = rotor(inertia);
    body.joint.stiffness = values({inertia * omega * omega});
    body.joint.damping = values({2.0 * zeta * inertia * omega});
    body.joint.rest = values({rest});
    model.bodies.push_back(body);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(config);
    State state{Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), 0.0};

    const double omega_h = omega * h;
    const double s = 1.0 + 2.0 * zeta * omega_h + omega_h * omega_h;
    Eigen::Matrix2d map;
    map << 1.0 - omega_h * omega_h / s, 1.0 / s, -omega_h * omega_h / s, 1.0 / s;
    Eigen::Vector2d predicted(-rest, 0.0);
    double overshoot = 0.0;
    double worst_mismatch = 0.0;
    int settled_at = -1;
    for (int i = 1; i <= 100; ++i) {
        provider.step(integrator, state);
        predicted = map * predicted;
        worst_mismatch = std::max(worst_mismatch, std::abs(state.q[0] - rest - predicted[0]));
        overshoot = std::max(overshoot, state.q[0] - rest);
        const bool settled = std::abs(state.q[0] - rest) <= 1e-6 * rest;
        if (settled && settled_at < 0) {
            settled_at = i;
        }
        if (!settled) {
            settled_at = -1;
        }
    }
    CAPTURE(overshoot, settled_at, worst_mismatch);
    CHECK(overshoot <= 0.01 * rest);
    CHECK(settled_at > 0);
    CHECK(worst_mismatch <= 1e-12);
    CHECK(std::abs(state.q[0] - rest) <= 1e-6 * rest);
}

TEST_CASE("joint stiffness and damping are returned as diagonal K and D with the spring forces in f", "[ENG-051]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    Body revolute = rod("revolute", -1, Eigen::Vector3d::Zero(), 1.0, 1.0);
    revolute.joint.stiffness = values({3.0});
    revolute.joint.damping = values({0.5});
    revolute.joint.rest = values({0.2});
    model.bodies.push_back(revolute);
    Body prismatic = rod("prismatic", 0, Eigen::Vector3d(1.0, 0.0, 0.0), 1.0, 1.0);
    prismatic.joint.type = JointType::Prismatic;
    prismatic.joint.axis = Eigen::Vector3d::UnitX();
    prismatic.joint.stiffness = values({7.0});
    model.bodies.push_back(prismatic);
    Body planar = rod("planar", 1, Eigen::Vector3d(1.0, 0.0, 0.0), 1.0, 1.0);
    planar.joint.type = JointType::Planar;
    planar.joint.stiffness = values({1.0, 2.0, 0.0});
    planar.joint.damping = values({0.1, 0.2, 0.3});
    planar.joint.rest = values({-0.1, 0.05, 0.4});
    model.bodies.push_back(planar);
    Body spherical = rod("spherical", 2, Eigen::Vector3d(1.0, 0.0, 0.0), 1.0, 1.0);
    spherical.joint.type = JointType::Spherical;
    spherical.joint.damping = values({0.4, 0.5, 0.6});
    model.bodies.push_back(spherical);
    Body fixed = rod("fixed", 3, Eigen::Vector3d(1.0, 0.0, 0.0), 1.0, 1.0);
    fixed.joint.type = JointType::Fixed;
    model.bodies.push_back(fixed);
    Body free = rod("free", -1, Eigen::Vector3d::Zero(), 1.0, 1.0);
    free.joint.type = JointType::Free;
    free.joint.damping = values({1.0, 2.0, 3.0, 4.0, 5.0, 6.0});
    model.bodies.push_back(free);
    model.bodies.push_back(rod("plain", 5, Eigen::Vector3d(1.0, 0.0, 0.0), 1.0, 1.0));
    const MultibodyProvider provider(model);
    REQUIRE(provider.velocity_size() == 15);

    const Eigen::VectorXd k = values({3.0, 7.0, 1.0, 2.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    const Eigen::VectorXd d = values({0.5, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0});
    const Eigen::VectorXd rest = values({0.2, 0.0, -0.1, 0.05, 0.4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    Eigen::VectorXd q = provider.neutral_positions();
    const Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(15, -0.7, 0.7);
    const Eigen::Index aligned = 5;
    q.head(aligned) = values({0.5, 0.25, 0.3, -0.2, 0.9});
    DynamicsTerms terms;
    provider.evaluate(0.0, q, v, terms);

    const Eigen::MatrixXd expected_k = k.asDiagonal();
    const Eigen::MatrixXd expected_d = d.asDiagonal();
    CHECK(terms.stiffness == expected_k);
    CHECK(terms.damping == expected_d);
    Eigen::VectorXd expected_f = -d.cwiseProduct(v);
    expected_f.head(aligned) -= k.head(aligned).cwiseProduct(q.head(aligned) - rest.head(aligned));
    CHECK((terms.force - expected_f).cwiseAbs().maxCoeff() <= 1e-15);
}

TEST_CASE("a joint without springs returns zero K and D and no spring force", "[ENG-051]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    model.bodies.push_back(rod("arm", -1, Eigen::Vector3d::Zero(), 1.0, 1.0));
    const MultibodyProvider provider(model);
    DynamicsTerms terms;
    provider.evaluate(0.0, Eigen::VectorXd::Constant(1, 0.4), Eigen::VectorXd::Constant(1, 2.0), terms);
    CHECK(terms.stiffness.isZero(0.0));
    CHECK(terms.damping.isZero(0.0));
    CHECK(terms.force.isZero(0.0));
}

TEST_CASE("invalid joint spring parameters are rejected with the cause", "[ENG-051]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    struct Case {
        std::string name;
        JointType type;
        Eigen::VectorXd stiffness;
        Eigen::VectorXd damping;
        Eigen::VectorXd rest;
        std::string cause;
    };
    const std::vector<Case> cases{
        {"stiffness size", JointType::Revolute, values({1.0, 2.0}), {}, {}, "joint stiffness must be empty"},
        {"damping size", JointType::Planar, {}, values({1.0}), {}, "joint damping must be empty"},
        {"rest size", JointType::Prismatic, {}, {}, values({0.0, 0.0}), "joint rest position must be empty"},
        {"negative stiffness", JointType::Revolute, values({-1.0}), {}, {}, "joint stiffness must not be negative"},
        {"negative damping", JointType::Revolute, {}, values({-1e-9}), {}, "joint damping must not be negative"},
        {"stiffness NaN", JointType::Revolute, values({nan}), {}, {}, "joint stiffness is not finite"},
        {"damping infinite",
         JointType::Revolute,
         {},
         values({std::numeric_limits<double>::infinity()}),
         {},
         "joint damping is not finite"},
        {"rest NaN", JointType::Revolute, {}, {}, values({nan}), "joint rest position is not finite"},
        {"spherical stiffness", JointType::Spherical, values({1.0, 1.0, 1.0}), {}, {}, "need a Revolute"},
        {"free rest", JointType::Free, {}, {}, values({0, 0, 0, 0, 0, 0}), "need a Revolute"},
        {"fixed damping", JointType::Fixed, {}, values({1.0}), {}, "joint damping must be empty"},
    };
    for (const Case& c : cases) {
        DYNAMIC_SECTION(c.name) {
            MultibodyModel model;
            Body body = rod("arm", -1, Eigen::Vector3d::Zero(), 1.0, 1.0);
            body.joint.type = c.type;
            body.joint.stiffness = c.stiffness;
            body.joint.damping = c.damping;
            body.joint.rest = c.rest;
            model.bodies.push_back(body);
            try {
                const MultibodyProvider provider(model);
                FAIL("no error");
            } catch (const InvalidModelError& error) {
                const std::string what = error.what();
                CAPTURE(what);
                CHECK(what.find("body 0") != std::string::npos);
                CHECK(what.find(c.cause) != std::string::npos);
                CHECK(error.time() == 0.0);
            }
        }
    }
}

TEST_CASE("negative rest positions and zero stiffness entries are valid", "[ENG-051]") {
    MultibodyModel model;
    Body body = rod("arm", -1, Eigen::Vector3d::Zero(), 1.0, 1.0);
    body.joint.stiffness = values({0.0});
    body.joint.damping = values({0.0});
    body.joint.rest = values({-2.5});
    model.bodies.push_back(body);
    CHECK_NOTHROW(MultibodyProvider(model));
}
