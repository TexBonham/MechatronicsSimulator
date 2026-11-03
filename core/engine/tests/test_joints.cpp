#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <numbers>
#include <vector>

#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "multibody_fixtures.hpp"

using mechsim::engine::Body;
using mechsim::engine::BodyOutput;
using mechsim::engine::DynamicsTerms;
using mechsim::engine::JointType;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::standard_gravity;

namespace {

Body free_body(const Eigen::Vector3d& com) {
    Body body;
    body.name = "free";
    body.joint.type = JointType::Free;
    body.mass = 2.5;
    body.com = com;
    Eigen::Matrix3d inertia;
    inertia << 0.30, 0.02, -0.01, 0.02, 0.25, 0.03, -0.01, 0.03, 0.40;
    body.inertia = inertia;
    return body;
}

State rest_state(const MultibodyProvider& provider) {
    return {provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
}

// Semi-implicit Euler from rest under constant g: v_n = n h g exactly, and q_n = q_0 + h sum_{k=1..n} v_k
// = q_0 + g h^2 n (n + 1) / 2 = q_0 + g t^2 / 2 + g h t / 2: the position leads the exact kinematics by h g t / 2.
void check_free_fall(MultibodyProvider& provider, const Eigen::Vector3d& gravity) {
    SemiImplicitEuler integrator(StepConfig{});
    const double h = integrator.config().h;
    State state = rest_state(provider);
    constexpr int steps = 500;
    for (int n = 1; n <= steps; ++n) {
        provider.step(integrator, state);
        const auto count = static_cast<double>(n);
        const double t = count * h;
        const Eigen::Vector3d velocity = count * h * gravity;
        const Eigen::Vector3d position = 0.5 * gravity * t * t + 0.5 * h * gravity * t;
        const BodyOutput& body = provider.outputs().bodies.front();
        CHECK((body.origin_velocity - velocity).norm() <= 1e-12 * velocity.norm());
        CHECK((body.position - position).norm() <= 1e-12 * position.norm());
        CHECK(std::abs(body.orientation.angularDistance(Eigen::Quaterniond::Identity())) <= 1e-12);
    }
}

}  // namespace

TEST_CASE("free fall under the model's gravity vector matches the scheme's exact kinematics", "[ENG-005]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(1.0, -2.0, -9.0);
    model.bodies.push_back(free_body(Eigen::Vector3d(0.1, -0.05, 0.2)));
    MultibodyProvider provider(model);
    check_free_fall(provider, model.gravity);
}

TEST_CASE("free joint under gravity reproduces the ENG-005 free fall", "[ENG-011]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, 0.0, -standard_gravity);
    model.bodies.push_back(free_body(Eigen::Vector3d::Zero()));
    MultibodyProvider provider(model);
    check_free_fall(provider, model.gravity);
}

TEST_CASE("free joint with no applied force keeps its linear velocity exactly and a unit quaternion", "[ENG-011]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    model.bodies.push_back(free_body(Eigen::Vector3d(0.1, 0.2, -0.1)));
    // A power-of-two mass makes m v exact, so v_lin x m v_lin is exactly zero in floating point too.
    model.bodies.front().mass = 2.0;
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(StepConfig{});
    const double h = integrator.config().h;
    SECTION("translating: c = v_lin x m v_lin = 0 exactly, so dv = 0 and v never changes") {
        State state = rest_state(provider);
        state.v.tail<3>() << 0.7, -0.4, 1.3;
        const Eigen::VectorXd v0 = state.v;
        constexpr int steps = 1000;
        for (int n = 1; n <= steps; ++n) {
            provider.step(integrator, state);
            CHECK(state.v == v0);
        }
        const Eigen::Vector3d expected = static_cast<double>(steps) * h * v0.tail<3>();
        CHECK((provider.outputs().bodies.front().position - expected).norm() <= 1e-12 * expected.norm());
    }
    SECTION("tumbling: the quaternion is renormalized every step") {
        State state = rest_state(provider);
        state.v << 3.0, -2.0, 5.0, 0.1, 0.2, 0.3;
        constexpr int steps = 10000;
        for (int n = 1; n <= steps; ++n) {
            provider.step(integrator, state);
            REQUIRE(std::abs(state.q.tail<4>().norm() - 1.0) <= 1e-12);
        }
    }
}

TEST_CASE("revolute pendulum at small amplitude has the scheme's period", "[ENG-006]") {
    // Pendulum: I = I_c + m d^2 about the pivot, omega^2 = m g d / I. Gravity is explicit (in c), so the linear map
    // v' = v - h omega^2 q, q' = q + h v' has trace 2 - (omega h)^2 and rotates by theta = 2 asin(omega h / 2) per
    // step: period error omega h / theta - 1, about -(omega h)^2 / 24. The amplitude adds theta0^2 / 16 (finite
    // amplitude). Asserted within 10 percent of the (omega h)^2 / 24 correction.
    // Note: the (omega h)^2 / 3 of package 1 applies to an implicit spring (K in the system matrix); gravity here
    // is in the bias term, which is the ruled design.
    constexpr double mass = 2.0;
    constexpr double arm = 0.05;
    constexpr double inertia_com = 1e-3;
    constexpr double theta0 = 1e-3;
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, -standard_gravity, 0.0);
    Body bob;
    bob.joint.type = JointType::Revolute;
    bob.mass = mass;
    bob.com = Eigen::Vector3d(0.0, -arm, 0.0);
    bob.inertia = inertia_com * Eigen::Matrix3d::Identity();
    model.bodies.push_back(bob);
    MultibodyProvider provider(model);
    const double inertia = inertia_com + mass * arm * arm;
    const double omega = std::sqrt(mass * standard_gravity * arm / inertia);
    const double omega_h = 0.1;
    SemiImplicitEuler integrator(StepConfig{omega_h / omega});
    const double h = integrator.config().h;
    State state{Eigen::VectorXd::Constant(1, theta0), Eigen::VectorXd::Zero(1), 0.0};
    std::vector<double> crossings;
    constexpr int steps = 1300;
    for (int n = 0; n < steps; ++n) {
        const double before = state.q[0];
        const double t0 = state.t;
        provider.step(integrator, state);
        if (before < 0.0 && state.q[0] >= 0.0) {
            crossings.push_back(t0 + h * before / (before - state.q[0]));
        }
    }
    REQUIRE(crossings.size() >= 15);
    const auto periods = static_cast<double>(crossings.size() - 1);
    const double measured = (crossings.back() - crossings.front()) / periods;
    const double error = measured / (2.0 * std::numbers::pi / omega) - 1.0;
    const double expected = omega_h / (2.0 * std::asin(0.5 * omega_h)) - 1.0 + theta0 * theta0 / 16.0;
    const double correction = omega_h * omega_h / 24.0;
    CAPTURE(error, expected);
    WARN("pendulum period error " << error << ", expected " << expected);
    CHECK(std::abs(error - expected) <= 0.1 * correction);
}

TEST_CASE("prismatic joint on a frictionless incline accelerates at g sin(alpha)", "[ENG-007]") {
    // Slide along axis (cos alpha, -sin alpha, 0) with g = (0, -g, 0): H = m, f - c = m g sin(alpha), so
    // dv = h g sin(alpha) every step; the mean acceleration v_n / (n h) is g sin(alpha) to rounding.
    constexpr double alpha = 0.35;
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, -standard_gravity, 0.0);
    Body slider;
    slider.joint.type = JointType::Prismatic;
    slider.joint.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(-alpha, Eigen::Vector3d::UnitZ()));
    slider.joint.axis = Eigen::Vector3d::UnitX();
    slider.mass = 3.0;
    slider.com = Eigen::Vector3d(0.1, 0.2, 0.0);
    slider.inertia = Eigen::Vector3d(0.1, 0.2, 0.3).asDiagonal();
    model.bodies.push_back(slider);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(StepConfig{});
    const double h = integrator.config().h;
    State state = rest_state(provider);
    const double expected = standard_gravity * std::sin(alpha);
    constexpr int steps = 200;
    for (int n = 1; n <= steps; ++n) {
        provider.step(integrator, state);
        const double acceleration = state.v[0] / (static_cast<double>(n) * h);
        CHECK(std::abs(acceleration - expected) <= 1e-12 * expected);
    }
}

TEST_CASE("fixed joint keeps the child at its placement under arbitrary wrenches", "[ENG-008]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, 0.0, -standard_gravity);
    model.bodies.push_back(free_body(Eigen::Vector3d(0.05, 0.0, 0.0)));
    Body child;
    child.parent = 0;
    child.joint.type = JointType::Fixed;
    child.joint.position = Eigen::Vector3d(0.3, -0.2, 0.1);
    child.joint.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, 2.0) / 3.0));
    child.mass = 0.8;
    child.com = Eigen::Vector3d(0.0, 0.1, 0.0);
    child.inertia = Eigen::Vector3d(0.01, 0.02, 0.03).asDiagonal();
    model.bodies.push_back(child);
    MultibodyProvider provider(model);
    REQUIRE(provider.velocity_size() == 6);
    SemiImplicitEuler integrator(StepConfig{});
    State state = rest_state(provider);
    constexpr int steps = 100;
    for (int n = 0; n < steps; ++n) {
        const double phase = 0.1 * static_cast<double>(n);
        provider.apply_body_wrench(1, Eigen::Vector3d(std::sin(phase), 2.0, -1.0),
                                   Eigen::Vector3d(5.0, -3.0 * std::cos(phase), 4.0));
        provider.apply_body_wrench(0, Eigen::Vector3d(0.0, -1.0, 0.5), Eigen::Vector3d(1.0, 0.0, 0.0));
        provider.step(integrator, state);
        const BodyOutput& parent = provider.outputs().bodies[0];
        const BodyOutput& body = provider.outputs().bodies[1];
        const Eigen::Matrix3d parent_rotation = parent.orientation.toRotationMatrix();
        const Eigen::Vector3d offset = parent_rotation.transpose() * (body.position - parent.position);
        const Eigen::Matrix3d relative = parent_rotation.transpose() * body.orientation.toRotationMatrix();
        CHECK((offset - child.joint.position).cwiseAbs().maxCoeff() <= 1e-15);
        CHECK((relative - child.joint.orientation.toRotationMatrix()).cwiseAbs().maxCoeff() <= 1e-15);
    }
}

TEST_CASE("planar joint keeps position and velocity in its plane under out-of-plane loads", "[ENG-009]") {
    // S spans only in-plane translation and rotation about n, so out-of-plane components vanish structurally; the
    // residue is rounding in the plane basis, well below 1e-14 for coordinates of order 1 m.
    const Eigen::Vector3d normal(0.0, 0.6, 0.8);
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, -standard_gravity, 0.0);
    Body plate;
    plate.joint.type = JointType::Planar;
    plate.joint.position = Eigen::Vector3d(0.1, 0.2, 0.3);
    plate.joint.axis = normal;
    plate.mass = 1.5;
    plate.com = Eigen::Vector3d(0.05, -0.02, 0.04);
    plate.inertia = Eigen::Vector3d(0.02, 0.03, 0.04).asDiagonal();
    model.bodies.push_back(plate);
    MultibodyProvider provider(model);
    REQUIRE(provider.velocity_size() == 3);
    SemiImplicitEuler integrator(StepConfig{});
    State state = rest_state(provider);
    state.v << 0.2, -0.1, 2.0;
    constexpr int steps = 1000;
    for (int n = 0; n < steps; ++n) {
        provider.apply_body_wrench(0, Eigen::Vector3d(3.0, -1.0, 2.0), 7.0 * normal + Eigen::Vector3d(1.0, 0.0, 0.0));
        provider.step(integrator, state);
        const BodyOutput& body = provider.outputs().bodies.front();
        CHECK(std::abs(normal.dot(body.position - plate.joint.position)) <= 1e-14);
        CHECK(std::abs(normal.dot(body.origin_velocity)) <= 1e-14);
        CHECK((body.orientation.toRotationMatrix() * normal - normal).cwiseAbs().maxCoeff() <= 1e-14);
        CHECK((body.spatial_velocity.head<3>().cross(normal)).cwiseAbs().maxCoeff() <= 1e-14);
    }
}

TEST_CASE("spherical joint keeps a unit quaternion and the scheme's angular momentum", "[ENG-010]") {
    // Torque-free about the joint (no gravity): H = I_O (inertia about the joint point), c = omega x I_O omega,
    // so L' = L - h omega x L and |L'|^2 = |L|^2 + h^2 |omega x L|^2 exactly. For a spin about a principal axis
    // omega x L = 0, so L is conserved (to rounding, asserted at 1e-9 relative over 1000 steps). For a general spin
    // the explicit gyroscopic term grows |L|^2 by the sum of h^2 |omega x L|^2, asserted to 1e-12 relative.
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    Body top;
    top.joint.type = JointType::Spherical;
    top.mass = 1.2;
    top.com = Eigen::Vector3d(0.0, 0.0, 0.1);
    top.inertia = Eigen::Vector3d(0.02, 0.03, 0.05).asDiagonal();
    model.bodies.push_back(top);
    MultibodyProvider provider(model);
    REQUIRE(provider.position_size() == 4);
    REQUIRE(provider.velocity_size() == 3);
    const Eigen::Matrix3d about_joint =
        top.inertia + top.mass * (top.com.squaredNorm() * Eigen::Matrix3d::Identity() - top.com * top.com.transpose());
    SemiImplicitEuler integrator(StepConfig{});
    const double h = integrator.config().h;
    SECTION("principal-axis spin conserves the world angular momentum") {
        State state = rest_state(provider);
        state.v << 0.0, 0.0, 4.0;
        const Eigen::Vector3d initial = about_joint * state.v;
        constexpr int steps = 1000;
        for (int n = 0; n < steps; ++n) {
            provider.step(integrator, state);
        }
        const Eigen::Vector3d world = provider.outputs().bodies.front().orientation * (about_joint * state.v);
        CHECK((world - initial).norm() <= 1e-9 * initial.norm());
    }
    SECTION("general spin follows the scheme's momentum growth and stays unit over 10 000 steps") {
        State state = rest_state(provider);
        state.v << 1.0, -2.0, 3.0;
        double predicted = (about_joint * state.v).squaredNorm();
        constexpr int steps = 10000;
        for (int n = 1; n <= steps; ++n) {
            const Eigen::Vector3d momentum = about_joint * state.v;
            const Eigen::Vector3d omega = state.v;
            predicted += h * h * omega.cross(momentum).squaredNorm();
            provider.step(integrator, state);
            REQUIRE(std::abs(state.q.norm() - 1.0) <= 1e-12);
            // Rounding: a few ulps (about 4e-16) per step in |L|^2 and in the predicted sum, so at most about
            // 1000 * 4e-16 = 4e-13 relative by step 1000, inside the 1e-12 bound.
            if (n == 1000) {
                CHECK(std::abs((about_joint * state.v).squaredNorm() - predicted) <= 1e-12 * predicted);
            }
        }
    }
}

TEST_CASE("undamped double pendulum keeps the scheme's modified energy", "[ENG-005][ENG-006]") {
    // Point masses m on links l, hanging along -y, q = [shoulder, elbow relative]; linearized M = m l^2 [[5, 2],
    // [2, 1]], K = m g l [[3, 1], [1, 1]]; omega^2 = (g / l)(2 -+ sqrt 2). With gravity in the bias term the linear map is
    // symplectic Euler, which conserves Q = v^T M v + q^T K q - h q^T K v exactly (no per-step contraction: the
    // 1 / (1 + (omega h)^2) factor needs K in the system matrix). Bounds: (1) the nonlinear remainder is of order
    // theta0^2 per unit of omega t, so |Q_n / Q_0 - 1| <= N (omega_max h) theta0^2; (2) |E - Q / 2| <= (omega h / 2) E
    // per mode, so |E_n / E_0 - 1| <= (1 + omega h / 2) / (1 - omega h / 2) - 1 plus (1).
    constexpr double mass = 1.0;
    constexpr double length = 1.0;
    constexpr double theta0 = 1e-3;
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, -standard_gravity, 0.0);
    for (Eigen::Index i = 0; i < 2; ++i) {
        Body link;
        link.parent = i - 1;
        link.joint.type = JointType::Revolute;
        if (i == 1) {
            link.joint.position = Eigen::Vector3d(0.0, -length, 0.0);
        }
        link.mass = mass;
        link.com = Eigen::Vector3d(0.0, -length, 0.0);
        link.inertia = Eigen::Matrix3d::Zero();
        model.bodies.push_back(link);
    }
    MultibodyProvider provider(model);
    Eigen::Matrix2d m_lin;
    m_lin << 5.0, 2.0, 2.0, 1.0;
    m_lin *= mass * length * length;
    Eigen::Matrix2d k_lin;
    k_lin << 3.0, 1.0, 1.0, 1.0;
    k_lin *= mass * standard_gravity * length;
    DynamicsTerms at_rest;
    provider.evaluate(0.0, Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), at_rest);
    CHECK((at_rest.mass - m_lin).cwiseAbs().maxCoeff() <= 1e-12);
    const Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::Matrix2d> modes(k_lin, m_lin);
    const double omega_max = std::sqrt(modes.eigenvalues().maxCoeff());
    CHECK(std::abs(omega_max - std::sqrt(standard_gravity / length * (2.0 + std::numbers::sqrt2))) <= 1e-12);
    const double omega_h = 0.05;
    SemiImplicitEuler integrator(StepConfig{omega_h / omega_max});
    const double h = integrator.config().h;
    const auto energy = [&](const State& s) {
        DynamicsTerms terms;
        provider.evaluate(s.t, s.q, s.v, terms);
        const double kinetic = 0.5 * s.v.dot(terms.mass * s.v);
        const double height1 = -length * std::cos(s.q[0]);
        const double height2 = height1 - length * std::cos(s.q[0] + s.q[1]);
        return kinetic + mass * standard_gravity * (height1 + height2 + 3.0 * length);
    };
    const auto modified = [&](const State& s) {
        const Eigen::Vector2d q = s.q;
        const Eigen::Vector2d v = s.v;
        return v.dot(m_lin * v) + q.dot(k_lin * q) - h * q.dot(k_lin * v);
    };
    State state{Eigen::Vector2d(theta0, -0.5 * theta0), Eigen::VectorXd::Zero(2), 0.0};
    const double e0 = energy(state);
    const double q0 = modified(state);
    constexpr int steps = 1000;
    const double nonlinear = static_cast<double>(steps) * omega_h * theta0 * theta0;
    const double oscillation = (1.0 + 0.5 * omega_h) / (1.0 - 0.5 * omega_h) - 1.0 + nonlinear;
    double worst_energy = 0.0;
    double worst_modified = 0.0;
    for (int n = 0; n < steps; ++n) {
        provider.step(integrator, state);
        worst_energy = std::max(worst_energy, std::abs(energy(state) / e0 - 1.0));
        worst_modified = std::max(worst_modified, std::abs(modified(state) / q0 - 1.0));
    }
    WARN("double pendulum max |E/E0 - 1| " << worst_energy << " (bound " << oscillation << "), max |Q/Q0 - 1| "
                                           << worst_modified << " (bound " << nonlinear << ")");
    CHECK(worst_modified <= nonlinear);
    CHECK(worst_energy <= oscillation);
}

TEST_CASE("body velocity output agrees with the finite difference of poses", "[ENG-027]") {
    SemiImplicitEuler integrator(StepConfig{});
    const double h = integrator.config().h;
    SECTION("translational: the second link origin of a double pendulum") {
        // p = l (sin q1, -cos q1); the step moves q1 by h w with w = v1 after the step, and the output is J(q') w.
        // Taylor: J(q') w - (p(q') - p(q)) / h = (h / 2) p''[w, w] + (h^2 / 3) p'''[w, w, w], so the gap is at
        // most (h / 2) l w^2 + (h^2 / 3) l |w|^3 (+ 1e-12 rounding).
        constexpr double length = 0.7;
        MultibodyModel model;
        model.gravity = Eigen::Vector3d(0.0, -standard_gravity, 0.0);
        for (Eigen::Index i = 0; i < 2; ++i) {
            Body link;
            link.parent = i - 1;
            link.joint.type = JointType::Revolute;
            if (i == 1) {
                link.joint.position = Eigen::Vector3d(0.0, -length, 0.0);
            }
            link.mass = 1.0;
            link.com = Eigen::Vector3d(0.0, -0.5 * length, 0.0);
            link.inertia = Eigen::Vector3d(0.01, 0.01, 0.04).asDiagonal();
            model.bodies.push_back(link);
        }
        MultibodyProvider provider(model);
        State state{Eigen::Vector2d(1.0, -0.5), Eigen::Vector2d(2.0, 1.0), 0.0};
        provider.step(integrator, state);
        constexpr int steps = 500;
        for (int n = 0; n < steps; ++n) {
            const Eigen::Vector3d before = provider.outputs().bodies[1].position;
            provider.step(integrator, state);
            const BodyOutput& body = provider.outputs().bodies[1];
            const double w = state.v[0];
            const double bound = 0.5 * h * length * w * w + h * h / 3.0 * length * std::abs(w * w * w) + 1e-12;
            CHECK(((body.position - before) / h - body.origin_velocity).norm() <= bound);
        }
    }
    SECTION("rotational: a spherical body's world angular velocity") {
        // R' = R exp(h w) leaves w fixed, so R' w = R w: the quaternion difference gives the output
        // up to quaternion rounding, about 1e-16 / h = 1e-13 rad/s; asserted at 1e-9.
        MultibodyModel model;
        model.gravity = Eigen::Vector3d(0.0, 0.0, -standard_gravity);
        Body top;
        top.joint.type = JointType::Spherical;
        top.com = Eigen::Vector3d(0.0, 0.1, 0.2);
        top.inertia = Eigen::Vector3d(0.02, 0.03, 0.05).asDiagonal();
        model.bodies.push_back(top);
        MultibodyProvider provider(model);
        State state = rest_state(provider);
        state.v << 1.0, -2.0, 3.0;
        provider.step(integrator, state);
        constexpr int steps = 200;
        for (int n = 0; n < steps; ++n) {
            const Eigen::Quaterniond before = provider.outputs().bodies.front().orientation;
            provider.step(integrator, state);
            const BodyOutput& body = provider.outputs().bodies.front();
            const Eigen::AngleAxisd turn(before.conjugate() * body.orientation);
            const Eigen::Vector3d world = before * (turn.angle() / h * turn.axis());
            CHECK((world - body.spatial_velocity.head<3>()).norm() <= 1e-9);
        }
    }
}
