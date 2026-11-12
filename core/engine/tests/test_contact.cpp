#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "contact_helpers.hpp"
#include "mechsim/engine/collision.hpp"
#include "mechsim/engine/contact.hpp"
#include "mechsim/engine/dynamics_provider.hpp"
#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/state.hpp"

using namespace mechsim::engine;
using namespace mechsim::engine::test;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr double tol = 1e-9;  // m, ENG-019's proposed depth tolerance, reused for exact closed forms here
constexpr double g = 9.81;

}  // namespace

// ===== ENG-016 detection =================================================

TEST_CASE("sphere-capsule and sphere-box closed forms", "[ENG-016]") {
    // Sphere r 0.1 at (0.15, 0, 0.3) beside a vertical capsule (r 0.1, cap centers z = +-0.5): closest axis point
    // (0, 0, 0.3), distance 0.15, depth 0.05, normal sphere to capsule -x.
    std::vector<ContactPoint> c = collide(sphere_shape(0.1), pose_at({0.15, 0, 0.3}), capsule_shape(0.1, 0.5),
                                          pose_at({0, 0, 0}));
    REQUIRE(c.size() == 1);
    CHECK(std::abs(c[0].depth - 0.05) <= tol);
    CHECK(angle_between(c[0].normal, -Eigen::Vector3d::UnitX()) <= 1e-12);
    // Past the cap: sphere at (0, 0, 0.65) meets the cap center (0, 0, 0.5): depth 0.05, normal -z.
    c = collide(sphere_shape(0.1), pose_at({0, 0, 0.65}), capsule_shape(0.1, 0.5), pose_at({0, 0, 0}));
    REQUIRE(c.size() == 1);
    CHECK(std::abs(c[0].depth - 0.05) <= tol);
    // Capsule first: the normal flips.
    c = collide(capsule_shape(0.1, 0.5), pose_at({0, 0, 0}), sphere_shape(0.1), pose_at({0, 0, 0.65}));
    REQUIRE(c.size() == 1);
    CHECK(angle_between(c[0].normal, Eigen::Vector3d::UnitZ()) <= 1e-12);
    // Box half 0.5. Outside a vertical edge: center (0.55, 0.55, 0), closest (0.5, 0.5, 0), distance 0.05 sqrt 2,
    // depth 0.1 - 0.0707, normal (-1, -1, 0) / sqrt 2.
    const Shape cube = box_shape(Eigen::Vector3d::Constant(0.5));
    c = collide(sphere_shape(0.1), pose_at({0.55, 0.55, 0}), cube, pose_at({0, 0, 0}));
    REQUIRE(c.size() == 1);
    CHECK(std::abs(c[0].depth - (0.1 - 0.05 * std::numbers::sqrt2)) <= tol);
    CHECK(angle_between(c[0].normal, Eigen::Vector3d(-1, -1, 0)) <= 1e-12);
    // Center inside, 0.05 under the +x face: depth r + 0.05 = 0.15, normal -x (the box leaves along -x).
    c = collide(sphere_shape(0.1), pose_at({0.45, 0, 0}), cube, pose_at({0, 0, 0}));
    REQUIRE(c.size() == 1);
    CHECK(std::abs(c[0].depth - 0.15) <= tol);
    CHECK(angle_between(c[0].normal, -Eigen::Vector3d::UnitX()) <= 1e-12);
    c = collide(cube, pose_at({0, 0, 0}), sphere_shape(0.1), pose_at({0.45, 0, 0}));
    REQUIRE(c.size() == 1);
    CHECK(angle_between(c[0].normal, Eigen::Vector3d::UnitX()) <= 1e-12);
    // Clear of the box: none.
    CHECK(collide(sphere_shape(0.1), pose_at({0.7, 0, 0}), cube, pose_at({0, 0, 0})).empty());
}

TEST_CASE("cylinder contacts are exact, four rim points per cap on a plane", "[ENG-016]") {
    // Upright r 0.1, half length 0.2 at z = 0.19: the bottom cap is 0.01 under the floor; four rim points, each 0.01.
    std::vector<ContactPoint> c =
        collide(cylinder_shape(0.1, 0.2), pose_at({0, 0, 0.19}), plane_shape(), pose_at({0, 0, 0}));
    REQUIRE(c.size() == 4);
    for (const ContactPoint& p : c) {
        CHECK(std::abs(p.depth - 0.01) <= tol);
        CHECK(std::abs(std::hypot(p.point.x(), p.point.y()) - 0.1) <= tol);
    }
    // Tilted 30 degrees about y: the deepest rim point sits r sin 30 below the bottom cap center, which is
    // h cos 30 below the center. Center z = h cos 30 + r sin 30 - 0.01: one contact, depth 0.01, the other rim points
    // at least r sin 30 - 0.01 = 0.04 above the floor.
    const double tilt = std::numbers::pi / 6.0;
    c = collide(cylinder_shape(0.1, 0.2),
                pose_at({0, 0, 0.2 * std::cos(tilt) + 0.1 * std::sin(tilt) - 0.01},
                        rotation_about(Eigen::Vector3d::UnitY(), tilt)),
                plane_shape(), pose_at({0, 0, 0}));
    REQUIRE(c.size() == 1);
    CHECK(std::abs(c[0].depth - 0.01) <= tol);
    // Upright on a box (half 0.5) through GJK/EPA: cap at z = 0.49 under the top face at 0.5: depth 0.01, normal from
    // the cylinder to the box -z. The Minkowski boundary there is flat (disk plus face), so EPA is exact.
    c = collide(cylinder_shape(0.1, 0.2), pose_at({0.05, 0, 0.69}), box_shape(Eigen::Vector3d::Constant(0.5)),
                pose_at({0, 0, 0}));
    REQUIRE(c.size() == 1);
    CHECK(std::abs(c[0].depth - 0.01) <= tol);
    CHECK(angle_between(c[0].normal, -Eigen::Vector3d::UnitZ()) <= 1e-6);
    CHECK(collide(cylinder_shape(0.1, 0.2), pose_at({0.05, 0, 0.71}), box_shape(Eigen::Vector3d::Constant(0.5)),
                  pose_at({0, 0, 0}))
              .empty());
}

TEST_CASE("hull and box pairs through GJK/EPA", "[ENG-016]") {
    // A cube given as a hull behaves like the box: half 0.5 at 0, small box half 0.2 at z = 0.65: depth 0.05, +z.
    std::vector<Eigen::Vector3d> corners;
    for (const double x : {-0.5, 0.5}) {
        for (const double y : {-0.5, 0.5}) {
            for (const double z : {-0.5, 0.5}) {
                corners.emplace_back(x, y, z);
            }
        }
    }
    const Shape hull = hull_shape(corners);
    std::vector<ContactPoint> c =
        collide(hull, pose_at({0, 0, 0}), box_shape(Eigen::Vector3d::Constant(0.2)), pose_at({0.1, 0.0, 0.65}));
    REQUIRE(c.size() == 1);
    CHECK(std::abs(c[0].depth - 0.05) <= tol);
    CHECK(angle_between(c[0].normal, Eigen::Vector3d::UnitZ()) <= 1e-6);
    // Hull against hull, exactly concentric: the origin is inside the Minkowski difference at its center; the
    // depth is the smallest face distance, 0.5 + 0.5 = 1.0 along a face normal.
    c = collide(hull, pose_at({0, 0, 0}), hull, pose_at({0, 0, 0}));
    REQUIRE(c.size() == 1);
    CHECK(std::abs(c[0].depth - 1.0) <= tol);
    // Capsule standing on a box: bottom cap center at z = 0.5 + 0.1 - 0.02 over the top face: depth 0.02, normal box
    // to capsule +z. Near the closest point the Minkowski boundary is the translated face, flat, so EPA is exact.
    c = collide(box_shape(Eigen::Vector3d::Constant(0.5)), pose_at({0, 0, 0}), capsule_shape(0.1, 0.3),
                pose_at({0.1, 0.1, 0.5 + 0.3 + 0.1 - 0.02}));
    REQUIRE(c.size() == 1);
    CHECK(std::abs(c[0].depth - 0.02) <= tol);
    CHECK(angle_between(c[0].normal, Eigen::Vector3d::UnitZ()) <= 1e-6);
    // Separated hulls (gap 0.1): none. Touching exactly (zero depth): none.
    CHECK(collide(hull, pose_at({0, 0, 0}), hull, pose_at({1.1, 0, 0})).empty());
    CHECK(collide(octahedron(0.1), pose_at({0, 0, 0.1}), plane_shape(), pose_at({0, 0, 0})).empty());
    CHECK(collide(plane_shape(), pose_at({0, 0, 0}), plane_shape(), pose_at({0, 0, 0.5})).empty());
}

TEST_CASE("axis-aligned bounds and the broad phase", "[ENG-016]") {
    const Aabb sphere = bounds(sphere_shape(0.5), pose_at({1, 2, 3}));
    CHECK((sphere.min - Eigen::Vector3d(0.5, 1.5, 2.5)).norm() <= tol);
    CHECK((sphere.max - Eigen::Vector3d(1.5, 2.5, 3.5)).norm() <= tol);
    const Aabb capsule = bounds(capsule_shape(0.1, 0.3), pose_at({0, 0, 0}));
    CHECK((capsule.max - Eigen::Vector3d(0.1, 0.1, 0.4)).norm() <= tol);
    const Aabb box = bounds(box_shape({0.1, 0.2, 0.3}), pose_at({0, 0, 0}, rotation_about(Eigen::Vector3d::UnitZ(),
                                                                                          std::numbers::pi / 2.0)));
    CHECK((box.max - Eigen::Vector3d(0.2, 0.1, 0.3)).norm() <= tol);
    const Aabb cylinder = bounds(cylinder_shape(0.1, 0.3),
                                 pose_at({0, 0, 0}, rotation_about(Eigen::Vector3d::UnitY(), std::numbers::pi / 2.0)));
    CHECK((cylinder.max - Eigen::Vector3d(0.3, 0.1, 0.1)).norm() <= tol);
    const Aabb hull = bounds(octahedron(0.1), pose_at({1, 0, 0}));
    CHECK((hull.min - Eigen::Vector3d(0.9, -0.1, -0.1)).norm() <= tol);
    const Aabb plane = bounds(plane_shape(), pose_at({0, 0, 0}));
    CHECK(plane.max.x() == std::numeric_limits<double>::infinity());
    CHECK(overlaps(plane, sphere));
    const auto pairs = broad_phase({sphere, hull, plane, bounds(sphere_shape(0.1), pose_at({1, 0, 0.15}))});
    const std::vector<std::pair<std::size_t, std::size_t>> expected{{0, 2}, {1, 2}, {1, 3}, {2, 3}};
    CHECK(pairs == expected);
}

TEST_CASE("curved pairs through GJK/EPA converge under the default limits", "[ENG-016]") {
    // Sphere r 0.1 at (0.15, 0, 0) beside an upright cylinder r 0.1: axis distance 0.15, depth 0.05, normal sphere
    // to cylinder -x. EPA stops when the support gain h(n_F) - d_F along the closest face normal is <= tol = 1e-12 m.
    // The polytope lies inside the Minkowski difference, so d_F <= D (true depth) and h(n_F) <= D + tol: the depth
    // is within tol. Where the boundary is locally curved with radius R, h(n) - D = R (1 - cos theta) for a normal
    // off by theta, so theta <= sqrt(2 tol / R); R >= 0.1 m here (0.15 m sphere-cylinder, 0.1 m for each crossed
    // cylinder), giving 4.5e-6 rad.
    const double curved_normal = std::sqrt(2.0 * CollisionLimits{}.tolerance / 0.1);
    std::vector<ContactPoint> c =
        collide(sphere_shape(0.1), pose_at({0.15, 0, 0}), cylinder_shape(0.1, 0.3), pose_at({0, 0, 0}));
    REQUIRE(c.size() == 1);
    CHECK(std::abs(c[0].depth - 0.05) <= tol);
    CHECK(angle_between(c[0].normal, -Eigen::Vector3d::UnitX()) <= curved_normal);
    // Crossed cylinders r 0.1 (axes x and y), centers 0.15 apart along z: the curved sides meet; depth 0.05, +z.
    c = collide(cylinder_shape(0.1, 0.5), pose_at({0, 0, 0}, rotation_about(Eigen::Vector3d::UnitY(), std::numbers::pi / 2.0)),
                cylinder_shape(0.1, 0.5),
                pose_at({0, 0, 0.15}, rotation_about(Eigen::Vector3d::UnitX(), std::numbers::pi / 2.0)));
    REQUIRE(c.size() == 1);
    CHECK(std::abs(c[0].depth - 0.05) <= tol);
    CHECK(angle_between(c[0].normal, Eigen::Vector3d::UnitZ()) <= curved_normal);
}

TEST_CASE("GJK or EPA exhausting its cap throws instead of dropping the contact", "[ENG-016]") {
    // Invariant 7: the overlapping sphere-cylinder pair above needs more than one EPA expansion (its Minkowski
    // boundary is curved) and more than one GJK iteration; with either cap at 1 the result is a failure, not "no
    // contact".
    CollisionLimits one_epa;
    one_epa.epa_iterations = 1;
    CHECK_THROWS_AS(collide(sphere_shape(0.1), pose_at({0.15, 0, 0}), cylinder_shape(0.1, 0.3), pose_at({0, 0, 0}),
                            one_epa),
                    SolverNonConvergenceError);
    CollisionLimits one_gjk;
    one_gjk.gjk_iterations = 1;
    CHECK_THROWS_WITH(collide(sphere_shape(0.1), pose_at({0.15, 0, 0}), cylinder_shape(0.1, 0.3), pose_at({0, 0, 0}),
                              one_gjk),
                      ContainsSubstring("GJK") && ContainsSubstring("sphere-cylinder"));
    // A genuine separation stays empty even at the smallest caps: GJK's first support already separates.
    CHECK(collide(sphere_shape(0.1), pose_at({1.0, 0, 0}), cylinder_shape(0.1, 0.3), pose_at({0, 0, 0}), one_gjk)
              .empty());
    CollisionLimits bad;
    bad.tolerance = 0.0;
    CHECK_THROWS_AS(collide(sphere_shape(0.1), pose_at({0, 0, 0}), sphere_shape(0.1), pose_at({0, 0, 0}), bad),
                    InvalidConfigError);
}

// ===== ENG-017 unsupported geometry ======================================

TEST_CASE("a geometry that is neither a primitive nor a convex hull ends the run naming it", "[ENG-017]") {
    MultibodyModel model;
    model.bodies.push_back(free_body("ball", 1.0, solid_sphere_inertia(1.0, 0.1)));
    MultibodyProvider provider(model);
    ContactModel contacts;
    contacts.geometries.push_back(geometry("mesh", 0));
    REQUIRE_THROWS_AS(provider.set_contact_model(contacts), InvalidModelError);
    REQUIRE_THROWS_WITH(provider.set_contact_model(contacts),
                        ContainsSubstring("contact geometry 0") && ContainsSubstring("'mesh'"));
    CHECK_THROWS_WITH(shape_kind("heightfield"), ContainsSubstring("'heightfield'"));
    for (const char* name : {"sphere", "capsule", "box", "cylinder", "plane", "convex_hull"}) {
        CHECK(std::string(shape_name(shape_kind(name))) == name);
    }
}

TEST_CASE("contact models with bad sizes, poses, bodies or parameters are rejected", "[ENG-017][ENG-020]") {
    MultibodyModel model;
    model.bodies.push_back(free_body("ball", 1.0, solid_sphere_inertia(1.0, 0.1)));
    MultibodyProvider provider(model);
    const auto rejects = [&provider](const ContactModel& m, const char* text) {
        CHECK_THROWS_WITH(provider.set_contact_model(m), ContainsSubstring(text));
    };
    ContactModel m;
    m.geometries = {geometry("sphere", 0)};
    rejects(m, "sphere radius");
    m.geometries[0].radius = 0.1;
    m.geometries[0].body = 3;
    rejects(m, "body is missing");
    m.geometries = {geometry("plane", 0)};
    rejects(m, "only on the world");
    Geometry capsule = geometry("capsule", 0);
    capsule.radius = 0.1;
    m.geometries = {capsule};
    rejects(m, "half length");
    Geometry box = geometry("box", 0);
    box.size = Eigen::Vector3d(0.1, 0.0, 0.1);
    m.geometries = {box};
    rejects(m, "half extents");
    Geometry hull = geometry("convex_hull", 0);
    hull.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    m.geometries = {hull};
    rejects(m, "at least four");
    hull.vertices.emplace_back(1, 1, 0);
    m.geometries = {hull};
    rejects(m, "coplanar");
    Geometry turned = geometry("sphere", 0);
    turned.radius = 0.1;
    turned.orientation = Eigen::Quaterniond(2.0, 0.0, 0.0, 0.0);
    m.geometries = {turned};
    rejects(m, "unit quaternion");
    m.geometries.clear();
    m.slip_velocity = 0.0;
    rejects(m, "slip velocity");
    m.slip_velocity = 1e-4;
    m.default_parameters.stiffness = -1.0;
    rejects(m, "stiffness");
    m.default_parameters.stiffness = 1e5;
    m.pairs.push_back({"a", "b", {1e5, -1.0, 0.5}});
    rejects(m, "damping");
    m.pairs[0].parameters = {1e5, 1e3, std::nan("")};
    rejects(m, "friction");
    m.pairs.clear();
    m.exclusions.emplace_back(0, 7);
    rejects(m, "exclusion");
}

// ===== ENG-020 soft contact ==============================================

TEST_CASE("the contact law: spring-damper, never adhesive, regularized Coulomb", "[ENG-020]") {
    const ContactParameters p{1e5, 1e3, 0.5};
    const double v_s = 1e-4;
    ContactPoint c;
    c.normal = Eigen::Vector3d::UnitZ();
    c.depth = 0.01;
    // At rest: lambda = K d = 1000 N along n on b; K block K n n^T, D block C n n^T.
    ContactForce f = contact_force(p, v_s, c, Eigen::Vector3d::Zero());
    CHECK(f.active);
    CHECK((f.force_on_b - Eigen::Vector3d(0, 0, 1000.0)).norm() <= 1e-9);
    CHECK(std::abs(f.stiffness(2, 2) - 1e5) <= 1e-9);
    CHECK(std::abs(f.damping(2, 2) - 1e3) <= 1e-9);
    // Separating at 2 m/s: K d - C 2 = -1000 < 0, clamped: no force, no K or D.
    f = contact_force(p, v_s, c, Eigen::Vector3d(0, 0, 2.0));
    CHECK_FALSE(f.active);
    CHECK(f.force_on_b.isZero());
    CHECK(f.stiffness.isZero());
    CHECK(f.damping.isZero());
    // Approaching at 0.5 m/s: lambda = 1000 + 500.
    f = contact_force(p, v_s, c, Eigen::Vector3d(0, 0, -0.5));
    CHECK(std::abs(f.normal_force - 1500.0) <= 1e-9);
    // Slip below v_s (5e-5 m/s along x): viscous, -mu lambda u_t / v_s = -0.5 * 1000 * 0.5 = -250 N along x; the
    // tangent damping is mu lambda / v_s = 5e6 N s/m in x and y.
    f = contact_force(p, v_s, c, Eigen::Vector3d(5e-5, 0, 0));
    CHECK(std::abs(f.force_on_b.x() + 250.0) <= 1e-9);
    CHECK(std::abs(f.damping(0, 0) - 5e6) <= 1e-3);
    CHECK(std::abs(f.damping(1, 1) - 5e6) <= 1e-3);
    // Sliding at 0.2 m/s along y: -mu lambda = -500 N along y; damping mu lambda / |u_t| = 2500 across the slip
    // (x), none along it (y).
    f = contact_force(p, v_s, c, Eigen::Vector3d(0, 0.2, 0));
    CHECK(std::abs(f.force_on_b.y() + 500.0) <= 1e-9);
    CHECK(std::abs(f.damping(0, 0) - 2500.0) <= 1e-9);
    CHECK(std::abs(f.damping(1, 1)) <= 1e-9);
    CHECK((f.damping - f.damping.transpose()).norm() == 0.0);
    // Frictionless: no tangential force.
    f = contact_force({1e5, 1e3, 0.0}, v_s, c, Eigen::Vector3d(0, 0.2, 0));
    CHECK(f.force_on_b.head<2>().isZero());
}

namespace {

MultibodyProvider resting_ball(double stiffness) {
    MultibodyModel model;
    model.bodies.push_back(free_body("ball", 1.0, solid_sphere_inertia(1.0, 0.05)));
    MultibodyProvider provider(model);
    Geometry ball = geometry("sphere", 0);
    ball.radius = 0.05;
    ContactModel contacts;
    contacts.geometries = {floor_plane(), ball};
    contacts.default_parameters = {stiffness, 1e3, 0.5};
    provider.set_contact_model(contacts);
    return provider;
}

}  // namespace

TEST_CASE("contact enters evaluate as force, K and D blocks", "[ENG-020]") {
    // Ball r 0.05 at z = 0.04 (depth 0.01) on the floor. Free joint v = [w; v_lin] (body = world axes here); the
    // contact point (0, 0, -0.01) lies under the origin, so its z velocity is v_z alone: G's z row is e_5. Hence
    // K = K_n e5 e5^T and f_5 = K_n d = 1000 N.
    const MultibodyProvider provider = resting_ball(1e5);
    Eigen::VectorXd q = provider.neutral_positions();
    q[2] = 0.04;
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(6);
    DynamicsTerms terms;
    provider.evaluate(0.0, q, v, terms);
    CHECK(std::abs(terms.stiffness(5, 5) - 1e5) <= 1e-6);
    CHECK(std::abs(terms.damping(5, 5) - 1e3) <= 1e-6);
    CHECK(std::abs(terms.force[5] - 1000.0) <= 1e-6);
    Eigen::MatrixXd rest = terms.stiffness;
    rest(5, 5) = 0.0;
    CHECK(rest.norm() <= 1e-9);
    const std::vector<ContactOutput> found = provider.contacts(q, v);
    REQUIRE(found.size() == 1);
    CHECK(found[0].body_a == -1);
    CHECK(found[0].body_b == 0);
    CHECK(std::abs(found[0].force.z() + 1000.0) <= 1e-6);
    CHECK(std::abs(found[0].depth - 0.01) <= tol);
    q[2] = 0.06;
    CHECK(provider.contacts(q, v).empty());
}

TEST_CASE("a stiff contact does not limit the step", "[ENG-020]") {
    // K_n = 1e9 N/m on 1 kg: w h = 31.6 at h = 1 ms, far past the explicit limit of 2. Implicit Euler on the
    // linearized spring-damper is unconditionally stable, and its fixed point is K_n d = m g: d = 9.81e-9 m.
    MultibodyProvider provider = resting_ball(1e9);
    State state;
    state.q = provider.neutral_positions();
    state.q[2] = 0.05;
    state.v = Eigen::VectorXd::Zero(6);
    SemiImplicitEuler integrator{StepConfig{}};
    for (int i = 0; i < 500; ++i) {
        static_cast<void>(provider.step(integrator, state));
        REQUIRE(state.q.allFinite());
    }
    const double target = g / 1e9;
    CHECK(std::abs((0.05 - state.q[2]) - target) <= 0.01 * target);
}

TEST_CASE("exclusions, material pairs and same-body geometry", "[ENG-020]") {
    MultibodyModel model;
    model.bodies.push_back(free_body("ball", 1.0, solid_sphere_inertia(1.0, 0.05)));
    MultibodyProvider provider(model);
    Geometry ball = geometry("sphere", 0);
    ball.radius = 0.05;
    ball.material = "rubber";
    const Geometry twin = ball;
    ContactModel contacts;
    contacts.geometries = {floor_plane(), ball, twin};
    contacts.pairs.push_back({"rubber", "steel", {2e5, 0.0, 0.0}});
    provider.set_contact_model(contacts);
    Eigen::VectorXd q = provider.neutral_positions();
    q[2] = 0.04;
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(6);
    // Two floor contacts (ball and twin), none between the two spheres of one body; the pair's K_n = 2e5 gives
    // 2e5 * 0.01 = 2000 N each.
    const std::vector<ContactOutput> found = provider.contacts(q, v);
    REQUIRE(found.size() == 2);
    for (const ContactOutput& c : found) {
        CHECK(std::abs(c.force.z() + 2000.0) <= 1e-6);
    }
    contacts.exclusions.emplace_back(0, -1);
    provider.set_contact_model(contacts);
    CHECK(provider.contacts(q, v).empty());
}

// ===== ENG-035 payload change ============================================

namespace {

struct Rig {
    MultibodyProvider provider;
    State state;
};

// Carrier: body 0 fixed to the world at z = 1. Payload: body 1, free, 2 kg, at z = 0.8.
Rig carrier_and_payload() {
    MultibodyModel model;
    Body carrier;
    carrier.name = "carrier";
    carrier.joint.type = JointType::Fixed;
    carrier.joint.position = Eigen::Vector3d(0, 0, 1);
    model.bodies.push_back(carrier);
    model.bodies.push_back(free_body("payload", 2.0, solid_box_inertia(2.0, {0.1, 0.1, 0.1})));
    MultibodyProvider provider(model);
    State state;
    state.q = provider.neutral_positions();
    state.q[2] = 0.8;
    state.v = Eigen::VectorXd::Zero(provider.velocity_size());
    return {std::move(provider), state};
}

void run(Rig& rig, Integrator& integrator, int steps) {
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(rig.provider.step(integrator, rig.state));
    }
}

}  // namespace

TEST_CASE("a welded payload carries m g and falls g t^2 / 2 once released", "[ENG-035]") {
    // Held: at rest the weld balances gravity, F = m g = 19.62 N up, whatever its stiffness (critically damped at
    // 1 kHz, it settles in a few ms; sag g / w^2 = 2.5e-7 m). Spec: within 1 %.
    Rig rig = carrier_and_payload();
    SemiImplicitEuler integrator{StepConfig{}};
    rig.provider.set_attachment(1, 0, true, rig.state.q);
    run(rig, integrator, 200);
    REQUIRE(rig.provider.outputs().payloads.size() == 1);
    PayloadOutput p = rig.provider.outputs().payloads[0];
    CHECK(p.attached);
    CHECK(p.carrier == 0);
    std::printf("weld force: target %.6f N, measured %.6f N\n", 2.0 * g, p.force.z());
    CHECK(std::abs(p.force.z() - 2.0 * g) <= 0.01 * 2.0 * g);
    CHECK(std::abs(rig.state.q[2] - 0.8) <= 1e-6);
    // Released: free fall from rest. Semi-implicit Euler gives z drop g h^2 n (n + 1) / 2 after n steps:
    // 0.19718 m at n = 200 against g t^2 / 2 = 0.19620 m, a +0.5 % scheme offset (h g t / 2 relative to g t^2 / 2 is
    // h / t). Spec: within 1 %.
    rig.provider.set_attachment(1, 0, false, rig.state.q);
    const double z0 = rig.state.q[2];
    run(rig, integrator, 200);
    const double drop = z0 - rig.state.q[2];
    const double expected = 0.5 * g * 0.2 * 0.2;
    std::printf("release drop: target %.6f m, measured %.6f m\n", expected, drop);
    CHECK(std::abs(drop - expected) <= 0.01 * expected);
    p = rig.provider.outputs().payloads[0];
    CHECK_FALSE(p.attached);
    CHECK(p.carrier == -1);
    CHECK(p.force.isZero());
    // Re-enabled at the carrier's pose now: the weld arrests the fall and carries m g again.
    rig.provider.set_attachment(1, 0, true, rig.state.q);
    const double held_at = rig.state.q[2];
    run(rig, integrator, 300);
    CHECK(std::abs(rig.provider.outputs().payloads[0].force.z() - 2.0 * g) <= 0.01 * 2.0 * g);
    CHECK(std::abs(rig.state.q[2] - held_at) <= 0.01);
    CHECK(rig.provider.attachments().size() == 1);
}

TEST_CASE("attach, release and mass change are listed as events in the next step", "[ENG-035]") {
    Rig rig = carrier_and_payload();
    SemiImplicitEuler integrator{StepConfig{}};
    rig.provider.set_attachment(1, 0, true, rig.state.q);
    run(rig, integrator, 1);
    REQUIRE(rig.provider.outputs().events.size() == 1);
    CHECK(rig.provider.outputs().events[0].kind == ModelChange::PayloadAttached);
    CHECK(rig.provider.outputs().events[0].target == 1);
    CHECK(rig.provider.outputs().events[0].index == 0);
    CHECK(rig.provider.outputs().events[0].time == 0.0);
    run(rig, integrator, 1);
    CHECK(rig.provider.outputs().events.empty());
    const double now = rig.provider.outputs().time;
    rig.provider.set_body_mass(1, 3.0);
    rig.provider.set_attachment(1, 0, false, rig.state.q);
    run(rig, integrator, 1);
    const std::vector<ModelChangeEvent>& events = rig.provider.outputs().events;
    REQUIRE(events.size() == 2);
    CHECK(events[0].kind == ModelChange::PayloadMass);
    CHECK(events[0].value == 3.0);
    CHECK(events[1].kind == ModelChange::LoadReleased);
    CHECK(events[1].source == LoadSource::Payload);
    CHECK(events[1].target == 1);
    CHECK(events[1].index == 0);
    CHECK(events[1].time == now);
}

TEST_CASE("releasing a payload with no held weld throws", "[ENG-034][ENG-035]") {
    Rig rig = carrier_and_payload();
    SemiImplicitEuler integrator{StepConfig{}};
    CHECK_THROWS_AS(rig.provider.set_attachment(1, 0, false, rig.state.q), InvalidModelError);
    rig.provider.set_attachment(1, 0, true, rig.state.q);
    rig.provider.set_attachment(1, 0, false, rig.state.q);
    CHECK_THROWS_AS(rig.provider.set_attachment(1, 0, false, rig.state.q), InvalidModelError);
    run(rig, integrator, 1);
    CHECK(rig.provider.outputs().events.size() == 2);
}

TEST_CASE("a mid-run mass change rescales inertia and the weld load", "[ENG-035]") {
    Rig rig = carrier_and_payload();
    SemiImplicitEuler integrator{StepConfig{}};
    rig.provider.set_attachment(1, -1, true, rig.state.q);
    run(rig, integrator, 50);
    const Eigen::Matrix3d before = rig.provider.model().bodies[1].inertia;
    rig.provider.set_body_mass(1, 4.0);
    CHECK(rig.provider.model().bodies[1].mass == 4.0);
    CHECK((rig.provider.model().bodies[1].inertia - 2.0 * before).norm() <= 1e-15);
    run(rig, integrator, 200);
    const PayloadOutput p = rig.provider.outputs().payloads[0];
    CHECK(p.carrier == -1);
    CHECK(std::abs(p.force.z() - 4.0 * g) <= 0.01 * 4.0 * g);
}

TEST_CASE("payload calls reject bad arguments", "[ENG-035]") {
    Rig rig = carrier_and_payload();
    CHECK_THROWS_AS(rig.provider.set_body_mass(2, 1.0), InvalidModelError);
    CHECK_THROWS_AS(rig.provider.set_body_mass(1, 0.0), InvalidModelError);
    CHECK_THROWS_AS(rig.provider.set_body_mass(1, std::nan("")), InvalidModelError);
    CHECK_THROWS_AS(rig.provider.set_attachment(1, 1, true, rig.state.q), InvalidModelError);
    CHECK_THROWS_AS(rig.provider.set_attachment(5, 0, true, rig.state.q), InvalidModelError);
    CHECK_THROWS_AS(rig.provider.set_attachment(1, -2, true, rig.state.q), InvalidModelError);
    CHECK_THROWS_AS(rig.provider.set_attachment(1, 0, true, Eigen::VectorXd::Zero(2)), InvalidModelError);
    CHECK_THROWS_AS(rig.provider.set_attachment(1, 0, true, rig.state.q, 0.0), InvalidModelError);
}
