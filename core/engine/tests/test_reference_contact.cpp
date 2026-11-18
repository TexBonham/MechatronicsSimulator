#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

#include "contact_helpers.hpp"
#include "mechsim/engine/collision.hpp"
#include "mechsim/engine/contact.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/state.hpp"

using namespace mechsim::engine;
using namespace mechsim::engine::test;

namespace {

// ENG-019 tolerances (docs/spec/eng.md, proposed): depth 1e-9 m, normal 1e-6 rad. The oracle is the closed form
// written beside each case; no reference collision library is installed (ENG-018 keeps it out of the runtime and
// none installs through uv here), so the closed-form set is the differential oracle.
constexpr double depth_tolerance = 1e-9;
constexpr double normal_tolerance = 1e-6;

void check_contact(const ContactPoint& c, const Eigen::Vector3d& normal, double depth) {
    std::printf("  depth target %.12e measured %.12e; normal angle %.3e rad\n", depth, c.depth,
                angle_between(c.normal, normal));
    CHECK(std::abs(c.depth - depth) <= depth_tolerance);
    CHECK(angle_between(c.normal, normal) <= normal_tolerance);
}

}  // namespace

// ===== ENG-019 closed-form contact set ===================================

TEST_CASE("sphere-sphere matches the closed form", "[ENG-019][reference]") {
    // Centers 0 and d = (0.6, 0.2, -0.3), |d| = sqrt(0.49) = 0.7; radii 0.5 and 0.3. depth = 0.5 + 0.3 - 0.7 = 0.1,
    // n = d / 0.7, point = midpoint of 0.5 n and d - 0.3 n = 0.45 n.
    const Eigen::Vector3d d(0.6, 0.2, -0.3);
    const std::vector<ContactPoint> c =
        collide(sphere_shape(0.5), pose_at(Eigen::Vector3d::Zero()), sphere_shape(0.3), pose_at(d));
    REQUIRE(c.size() == 1);
    check_contact(c[0], d / 0.7, 0.1);
    CHECK((c[0].point - 0.45 * d / 0.7).norm() <= depth_tolerance);
    // Separated by 0.01 (|d| = 0.81 > 0.8): no contact.
    CHECK(collide(sphere_shape(0.5), pose_at(Eigen::Vector3d::Zero()), sphere_shape(0.3),
                  pose_at(Eigen::Vector3d(0.81, 0.0, 0.0)))
              .empty());
}

TEST_CASE("sphere-plane matches the closed form, level and tilted", "[ENG-019][reference]") {
    // Level floor z <= 0, sphere r = 0.2 at z = 0.15: depth 0.05, normal sphere to plane -z, point at z = -0.025.
    std::vector<ContactPoint> c =
        collide(sphere_shape(0.2), pose_at(Eigen::Vector3d(1.0, -2.0, 0.15)), plane_shape(), pose_at({0, 0, 0}));
    REQUIRE(c.size() == 1);
    check_contact(c[0], -Eigen::Vector3d::UnitZ(), 0.05);
    CHECK((c[0].point - Eigen::Vector3d(1.0, -2.0, -0.025)).norm() <= depth_tolerance);
    // Plane tilted 30 degrees about x through (0, 0, 1): outward normal n = R z. Sphere r = 0.15 at the plane origin
    // + 0.1 n + 0.3 (a tangent): depth 0.15 - 0.1 = 0.05, normal -n.
    const Eigen::Matrix3d r = rotation_about(Eigen::Vector3d::UnitX(), std::numbers::pi / 6.0);
    const Eigen::Vector3d n = r.col(2);
    const Eigen::Vector3d origin(0.0, 0.0, 1.0);
    c = collide(sphere_shape(0.15), pose_at(origin + 0.1 * n + 0.3 * r.col(0)), plane_shape(), pose_at(origin, r));
    REQUIRE(c.size() == 1);
    check_contact(c[0], -n, 0.05);
    // Order swapped (plane is a): the normal flips to n, the depth is the same.
    c = collide(plane_shape(), pose_at(origin, r), sphere_shape(0.15), pose_at(origin + 0.1 * n));
    REQUIRE(c.size() == 1);
    check_contact(c[0], n, 0.05);
}

TEST_CASE("box-plane gives every penetrating corner", "[ENG-019][reference]") {
    // Level: half extents (0.1, 0.2, 0.3) at z = 0.25: the four bottom corners at z = -0.05, depth 0.05 each.
    const Eigen::Vector3d half(0.1, 0.2, 0.3);
    std::vector<ContactPoint> c =
        collide(box_shape(half), pose_at(Eigen::Vector3d(0.0, 0.0, 0.25)), plane_shape(), pose_at({0, 0, 0}));
    REQUIRE(c.size() == 4);
    for (const ContactPoint& p : c) {
        check_contact(p, -Eigen::Vector3d::UnitZ(), 0.05);
    }
    // Tilted by 0.3 rad about (1, 2, 3) at z = 0.2: corner k at z_k = 0.2 + (R diag(s) half)_z for signs s; every
    // corner with z_k < 0 is one contact of depth -z_k.
    const Eigen::Matrix3d r = rotation_about(Eigen::Vector3d(1.0, 2.0, 3.0), 0.3);
    std::vector<double> expected;
    for (const double sx : {-1.0, 1.0}) {
        for (const double sy : {-1.0, 1.0}) {
            for (const double sz : {-1.0, 1.0}) {
                const double z = 0.2 + (r * Eigen::Vector3d(sx * half.x(), sy * half.y(), sz * half.z())).z();
                if (z < 0.0) {
                    expected.push_back(-z);
                }
            }
        }
    }
    c = collide(box_shape(half), pose_at(Eigen::Vector3d(0.0, 0.0, 0.2), r), plane_shape(), pose_at({0, 0, 0}));
    REQUIRE(c.size() == expected.size());
    REQUIRE_FALSE(expected.empty());
    std::vector<double> measured;
    for (const ContactPoint& p : c) {
        CHECK(angle_between(p.normal, -Eigen::Vector3d::UnitZ()) <= normal_tolerance);
        measured.push_back(p.depth);
    }
    std::sort(expected.begin(), expected.end());
    std::sort(measured.begin(), measured.end());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CHECK(std::abs(measured[i] - expected[i]) <= depth_tolerance);
    }
}

TEST_CASE("capsule-plane uses the lower cap", "[ENG-019][reference]") {
    // Axis tilted 30 degrees about y, half length 0.5, r = 0.1: cap centers at z0 -+ 0.5 cos 30. With
    // z0 = 0.5 cos 30 + 0.05 the lower cap center is at z = 0.05: depth 0.1 - 0.05 = 0.05; the upper cap is clear.
    const double tilt = std::numbers::pi / 6.0;
    const double z0 = 0.5 * std::cos(tilt) + 0.05;
    const std::vector<ContactPoint> c =
        collide(capsule_shape(0.1, 0.5), pose_at(Eigen::Vector3d(0.0, 0.0, z0), rotation_about(Eigen::Vector3d::UnitY(), tilt)),
                plane_shape(), pose_at({0, 0, 0}));
    REQUIRE(c.size() == 1);
    check_contact(c[0], -Eigen::Vector3d::UnitZ(), 0.05);
    CHECK(std::abs(c[0].point.x() + 0.5 * std::sin(tilt)) <= depth_tolerance);
}

TEST_CASE("capsule-capsule crossed and end to end", "[ENG-019][reference]") {
    // Crossed: a along x at 0, b along y at z = 0.15, r = 0.1 each. Closest points (0,0,0), (0,0,0.15): depth 0.05,
    // normal +z, point (0, 0, 0.075).
    const Eigen::Matrix3d along_x = rotation_about(Eigen::Vector3d::UnitY(), std::numbers::pi / 2.0);
    const Eigen::Matrix3d along_y = rotation_about(Eigen::Vector3d::UnitX(), std::numbers::pi / 2.0);
    std::vector<ContactPoint> c = collide(capsule_shape(0.1, 0.5), pose_at({0, 0, 0}, along_x), capsule_shape(0.1, 0.5),
                                          pose_at({0, 0, 0.15}, along_y));
    REQUIRE(c.size() == 1);
    check_contact(c[0], Eigen::Vector3d::UnitZ(), 0.05);
    CHECK((c[0].point - Eigen::Vector3d(0.0, 0.0, 0.075)).norm() <= depth_tolerance);
    // Collinear along x: a spans [-0.5, 0.5], b centered at 1.15 spans [0.65, 1.65]: gap 0.15, depth 0.05, normal
    // +x, point (0.575, 0, 0).
    c = collide(capsule_shape(0.1, 0.5), pose_at({0, 0, 0}, along_x), capsule_shape(0.1, 0.5),
                pose_at({1.15, 0, 0}, along_x));
    REQUIRE(c.size() == 1);
    check_contact(c[0], Eigen::Vector3d::UnitX(), 0.05);
    CHECK((c[0].point - Eigen::Vector3d(0.575, 0.0, 0.0)).norm() <= depth_tolerance);
}

TEST_CASE("box-box face and edge cases", "[ENG-019][reference]") {
    // Face: a half 0.5 at 0, b half 0.2 at (0.1, -0.05, 0.65) yawed 20 degrees (faces stay parallel). b's bottom at
    // z = 0.45 under a's top at 0.5: depth 0.05, normal +z; the witnesses lie on z = 0.5 and 0.45, point z = 0.475.
    std::vector<ContactPoint> c =
        collide(box_shape(Eigen::Vector3d::Constant(0.5)), pose_at({0, 0, 0}), box_shape(Eigen::Vector3d::Constant(0.2)),
                pose_at({0.1, -0.05, 0.65}, rotation_about(Eigen::Vector3d::UnitZ(), 20.0 * std::numbers::pi / 180.0)));
    REQUIRE(c.size() == 1);
    check_contact(c[0], Eigen::Vector3d::UnitZ(), 0.05);
    CHECK(std::abs(c[0].point.z() - 0.475) <= depth_tolerance);
    // Edge: a (half 0.5) turned 45 degrees about y puts an edge along y on top at z = 0.5 sqrt 2; b (half 0.5) turned
    // 45 degrees about x puts an edge along x at its bottom, z_b - 0.5 sqrt 2. The edges cross at x = y = 0 with
    // normal y x x = +z; with z_b = sqrt 2 - 0.05 the depth is 0.05 (every face and other edge axis overlaps more:
    // 0.39 along a's face normals, 0.61 along the (1, -1, 1) edge axis). Point (0, 0, 0.5 sqrt 2 - 0.025).
    const double half_diagonal = 0.5 * std::numbers::sqrt2;
    c = collide(box_shape(Eigen::Vector3d::Constant(0.5)),
                pose_at({0, 0, 0}, rotation_about(Eigen::Vector3d::UnitY(), std::numbers::pi / 4.0)),
                box_shape(Eigen::Vector3d::Constant(0.5)),
                pose_at({0, 0, std::numbers::sqrt2 - 0.05}, rotation_about(Eigen::Vector3d::UnitX(), std::numbers::pi / 4.0)));
    REQUIRE(c.size() == 1);
    check_contact(c[0], Eigen::Vector3d::UnitZ(), 0.05);
    CHECK((c[0].point - Eigen::Vector3d(0.0, 0.0, half_diagonal - 0.025)).norm() <= 1e-6);
}

TEST_CASE("hull-plane gives every penetrating vertex", "[ENG-019][reference]") {
    // Octahedron a = 0.1 at z = 0.08: only the -z vertex is below the floor, depth 0.02.
    std::vector<ContactPoint> c =
        collide(octahedron(0.1), pose_at({0, 0, 0.08}), plane_shape(), pose_at({0, 0, 0}));
    REQUIRE(c.size() == 1);
    check_contact(c[0], -Eigen::Vector3d::UnitZ(), 0.02);
    // Turned 45 degrees about y: vertices -x and -z sit at z0 - 0.1 sin 45 each; at z0 = 0.06 both penetrate by
    // 0.1 / sqrt 2 - 0.06.
    c = collide(octahedron(0.1), pose_at({0, 0, 0.06}, rotation_about(Eigen::Vector3d::UnitY(), std::numbers::pi / 4.0)),
                plane_shape(), pose_at({0, 0, 0}));
    REQUIRE(c.size() == 2);
    for (const ContactPoint& p : c) {
        check_contact(p, -Eigen::Vector3d::UnitZ(), 0.1 / std::numbers::sqrt2 - 0.06);
    }
}

// ===== ENG-021 rest and hard stop ========================================

namespace {

constexpr double g = 9.81;
constexpr double contact_stiffness = 1e5;  // K_n, N/m
constexpr double contact_damping = 1e3;    // C_n, N s/m

ContactModel contact_with(std::vector<Geometry> geometries, double friction) {
    ContactModel m;
    m.geometries = std::move(geometries);
    m.default_parameters.stiffness = contact_stiffness;
    m.default_parameters.damping = contact_damping;
    m.default_parameters.friction = friction;
    return m;
}

struct Drop {
    MultibodyProvider provider;
    State state;
};

constexpr double drop_radius = 0.05;  // m

Drop sphere_drop() {
    MultibodyModel model;
    model.bodies.push_back(free_body("ball", 1.0, solid_sphere_inertia(1.0, drop_radius)));
    MultibodyProvider provider(model);
    Geometry ball = geometry("sphere", 0);
    ball.radius = drop_radius;
    provider.set_contact_model(contact_with({floor_plane(), ball}, 0.5));
    State state;
    state.q = provider.neutral_positions();
    state.q[2] = 0.1 + drop_radius;
    state.v = Eigen::VectorXd::Zero(provider.velocity_size());
    return {std::move(provider), state};
}

}  // namespace

TEST_CASE("a sphere dropped on a plane rests at m g / K_n", "[ENG-021][reference]") {
    // 1 kg, dropped 0.1 m (impact 1.40 m/s), K_n = 1e5 N/m, C_n = 1e3 N s/m. At rest v = 0, so the damper carries
    // nothing and K_n d = m g: d = 9.81e-5 m, and the step's fixed point (f - c = 0) is that equilibrium exactly.
    // Settling: zeta = C / (2 sqrt(K m)) = 1.58, slow root (C - sqrt(C^2 - 4 K m)) / (2 m) = 112.7 1/s, so after
    // 1 s the residual is e^-112 of the impact transient; implicit Euler damps further. Spec: within 1 % of m g / K_n
    // and no bounce above 1e-6 m after 1 s.
    Drop drop = sphere_drop();
    SemiImplicitEuler integrator{StepConfig{}};
    for (int i = 0; i < 1000; ++i) {
        static_cast<void>(drop.provider.step(integrator, drop.state));
    }
    const double target = 1.0 * g / contact_stiffness;
    double low = drop.state.q[2];
    double high = drop.state.q[2];
    for (int i = 0; i < 500; ++i) {
        static_cast<void>(drop.provider.step(integrator, drop.state));
        low = std::min(low, drop.state.q[2]);
        high = std::max(high, drop.state.q[2]);
    }
    const double penetration = drop_radius - drop.state.q[2];
    std::printf("sphere drop: target %.6e m, measured %.6e m, residual motion %.3e m\n", target, penetration, high - low);
    CHECK(std::abs(penetration - target) <= 0.01 * target);
    CHECK(high - low <= 1e-6);
    REQUIRE(drop.provider.outputs().contacts.size() == 1);
    CHECK(std::abs(drop.provider.outputs().contacts[0].force.z() + 1.0 * g) <= 0.01 * g);
}

TEST_CASE("the sphere drop is bitwise identical on a second run", "[ENG-021][ENG-023][reference]") {
    Drop first = sphere_drop();
    Drop second = sphere_drop();
    SemiImplicitEuler a{StepConfig{}};
    SemiImplicitEuler b{StepConfig{}};
    for (int i = 0; i < 400; ++i) {
        static_cast<void>(first.provider.step(a, first.state));
        static_cast<void>(second.provider.step(b, second.state));
        REQUIRE(first.state.q == second.state.q);
        REQUIRE(first.state.v == second.state.v);
    }
}

TEST_CASE("a body driven into a hard stop stops at F / K_n", "[ENG-021][reference]") {
    // Prismatic x, 1 kg, sphere r = 0.05 driven by F = 50 N into a world plane with outward normal -x through
    // x = 0.5. Contact after 0.45 m (t = 0.134 s, 6.7 m/s); the same zeta = 1.58 transient decays by e^-97 by 1 s.
    // At rest K_n d = F: d = 5e-4 m. Spec: within 1 % of F / K_n.
    MultibodyModel model;
    Body slider;
    slider.name = "slider";
    slider.joint.type = JointType::Prismatic;
    slider.joint.axis = Eigen::Vector3d::UnitX();
    slider.mass = 1.0;
    slider.inertia = solid_sphere_inertia(1.0, drop_radius);
    model.bodies.push_back(slider);
    MultibodyProvider provider(model);
    Geometry ball = geometry("sphere", 0);
    ball.radius = drop_radius;
    const Eigen::Matrix3d facing = rotation_about(Eigen::Vector3d::UnitY(), -std::numbers::pi / 2.0);
    provider.set_contact_model(contact_with({floor_plane(facing, Eigen::Vector3d(0.5, 0.0, 0.0)), ball}, 0.5));
    State state;
    state.q = Eigen::VectorXd::Zero(1);
    state.v = Eigen::VectorXd::Zero(1);
    SemiImplicitEuler integrator{StepConfig{}};
    const double force = 50.0;
    for (int i = 0; i < 1000; ++i) {
        provider.set_generalized_forces(Eigen::VectorXd::Constant(1, force));
        static_cast<void>(provider.step(integrator, state));
    }
    const double target = force / contact_stiffness;
    const double penetration = state.q[0] + drop_radius - 0.5;
    std::printf("hard stop: target %.6e m, measured %.6e m\n", target, penetration);
    CHECK(std::abs(penetration - target) <= 0.01 * target);
    CHECK(std::abs(state.v[0]) <= 1e-6);
}

namespace {

constexpr double incline = 10.0 * std::numbers::pi / 180.0;
const Eigen::Vector3d box_half(0.1, 0.1, 0.05);

struct Slope {
    double displacement = 0.0;  // along the downslope direction, m
    double speed = 0.0;         // along the downslope direction, m/s
};

// Box on a plane tilted 10 degrees about y; returns its downslope motion at times t1 and t2 (s).
std::pair<Slope, Slope> box_on_incline(double friction, double t1, double t2) {
    const Eigen::Matrix3d tilt = rotation_about(Eigen::Vector3d::UnitY(), incline);
    const Eigen::Vector3d normal = tilt.col(2);
    const Eigen::Vector3d downslope(std::cos(incline), 0.0, -std::sin(incline));
    MultibodyModel model;
    model.bodies.push_back(free_body("box", 1.0, solid_box_inertia(1.0, box_half)));
    MultibodyProvider provider(model);
    Geometry box = geometry("box", 0);
    box.size = 2.0 * box_half;
    provider.set_contact_model(contact_with({floor_plane(tilt), box}, friction));
    State state;
    state.q = provider.neutral_positions();
    state.q.head<3>() = box_half.z() * normal;
    const Eigen::Quaterniond r(tilt);
    state.q.segment<4>(3) << r.w(), r.x(), r.y(), r.z();
    state.v = Eigen::VectorXd::Zero(6);
    SemiImplicitEuler integrator{StepConfig{}};
    const Eigen::Vector3d start = state.q.head<3>();
    std::pair<Slope, Slope> out;
    const int n1 = static_cast<int>(std::lround(t1 / integrator.config().h));
    const int n2 = static_cast<int>(std::lround(t2 / integrator.config().h));
    for (int i = 1; i <= n2; ++i) {
        static_cast<void>(provider.step(integrator, state));
        if (i == n1 || i == n2) {
            const BodyOutput body = provider.body_states(state.q, state.v)[0];
            Slope s;
            s.displacement = downslope.dot(body.position - start);
            s.speed = downslope.dot(body.origin_velocity);
            if (i == n1) {
                out.first = s;
            } else {
                out.second = s;
            }
        }
    }
    return out;
}

}  // namespace

TEST_CASE("a box on a 10 degree incline holds at mu 0.5 and slides at mu 0.1", "[ENG-021][ENG-020][reference]") {
    // Stick: tan 10 = 0.176 < 0.5. The regularized law is viscous below v_s with total coefficient mu N / v_s over
    // the four corners (N = m g cos 10), so the box creeps at v_c = v_s tan 10 / mu = 3.53e-5 m/s at most (v_s =
    // 1e-4 m/s default). Checked: speed at 1 s below v_c (1 % margin) and drift over [0.5, 1] s below v_c * 0.5 s.
    const double v_s = ContactModel::default_slip_velocity;
    const double creep = v_s * std::tan(incline) / 0.5;
    const auto [early, late] = box_on_incline(0.5, 0.5, 1.0);
    std::printf("incline mu 0.5: creep bound %.4e m/s, speed %.4e m/s, drift %.4e m\n", creep, late.speed,
                late.displacement - early.displacement);
    CHECK(std::abs(late.speed) <= 1.01 * creep);
    CHECK(std::abs(late.displacement - early.displacement) <= 1.01 * creep * 0.5);
    // Slide: every corner slips (|u_t| >> v_s after 0.2 ms), friction mu N along -t, so a = g (sin 10 - mu cos 10)
    // = 0.7374 m/s^2; constant acceleration is exact under the scheme. Measured as the speed gain over [0.3, 0.6] s.
    // Spec: within 1 %.
    const double expected = g * (std::sin(incline) - 0.1 * std::cos(incline));
    const auto [first, second] = box_on_incline(0.1, 0.3, 0.6);
    const double measured = (second.speed - first.speed) / 0.3;
    std::printf("incline mu 0.1: target %.6f m/s^2, measured %.6f m/s^2\n", expected, measured);
    CHECK(std::abs(measured - expected) <= 0.01 * expected);
}
