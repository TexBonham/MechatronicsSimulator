#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "json.hpp"
#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/physics/native/native_adapter.hpp"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using mechsim::kernel::AdapterError;
using mechsim::kernel::JointDrive;
using mechsim::kernel::PhysicsInputs;
using mechsim::kernel::PreparedModel;
using mechsim::kernel::Rate;
using mechsim::physics::native::NativeAdapter;

// ===== Fixtures and helpers ==============================================

namespace {

constexpr double g = 9.81;

struct DofSpec {
    std::string friction = "0.0";
    std::string lower = "null";
    std::string upper = "null";
};

std::string body(const std::string& name, int parent, const std::string& type, const std::string& axis, double mass,
                 const std::string& stiffness = "[]", const std::string& damping = "[]", const DofSpec& dof = {},
                 const std::string& position = "[0, 0, 0]", const std::string& com = "[0, 0, 0]",
                 const std::string& inertia = "[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]") {
    std::string extras = R"("friction": [], "lower": [], "upper": [])";
    if (type == "revolute" || type == "prismatic") {
        extras = R"("friction": [)" + dof.friction + R"(], "lower": [)" + dof.lower + R"(], "upper": [)" + dof.upper + "]";
    }
    return R"({"name": ")" + name + R"(", "parent": )" + std::to_string(parent) + R"(, "mass": )" + std::to_string(mass) +
           R"(, "com": )" + com + R"(, "inertia": )" + inertia + R"(, "joint": {"type": ")" + type +
           R"(", "position": )" + position + R"(, "orientation": [1, 0, 0, 0], "axis": )" + axis +
           R"(, "stiffness": )" + stiffness + R"(, "damping": )" + damping + R"(, "rest": [], )" + extras + "}}";
}

struct Doc {
    std::string elements = R"(["body", "joint.prismatic"])";
    std::string gravity = "[0, 0, -9.81]";
    std::vector<std::string> bodies;
    int neutral_bodies = -1;
    std::string constraints = "[]";
    std::string members = "[]";
    std::string transmissions = "[]";
    std::string ropes = "[]";
    std::string contact = "null";
    std::string payloads = "[]";
    std::string starts = "[]";
    int substeps = 0;  // 0: the key is left out (one engine step per adapter step)

    [[nodiscard]] std::string text() const {
        std::string list;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            if (i > 0) {
                list += ", ";
            }
            list += bodies[i];
        }
        int neutral = neutral_bodies;
        if (neutral < 0) {
            neutral = static_cast<int>(bodies.size());
        }
        return R"({"format": "mechsim-native-1", "elements": )" + elements + R"(, "gravity": )" + gravity +
               R"(, "config": {"iteration_cap": 50, "velocity_tolerance": 1e-9, "projection_iterations": 3, )"
               R"("drift_threshold_m": 1e-6, "drift_threshold_rad": 1e-6, "unit_tolerance": 1e-12, )"
               R"("symmetry_tolerance": 1e-12}, "bodies": [)" +
               list + R"(], "neutral_bodies": )" + std::to_string(neutral) + R"(, "constraints": )" + constraints +
               R"(, "members": )" + members + R"(, "transmissions": )" + transmissions + R"(, "ropes": )" + ropes +
               R"(, "contact": )" + contact + R"(, "payloads": )" + payloads + R"(, "starts": )" + starts +
               substeps_key() + "}";
    }

    [[nodiscard]] std::string substeps_key() const {
        if (substeps == 0) {
            return "";
        }
        return R"(, "substeps": )" + std::to_string(substeps);
    }
};

void load(NativeAdapter& adapter, const Doc& doc, std::int64_t hz = 1000) {
    adapter.load(PreparedModel{doc.text(), "", "1.0.0"}, Rate(hz));
}

Doc slider(double mass, const DofSpec& dof = {}) {
    Doc doc;
    doc.bodies.push_back(body("slider", -1, "prismatic", "[0, 0, 1]", mass, "[]", "[]", dof));
    return doc;
}

void drive(NativeAdapter& adapter, double rest, double stiffness, double damping) {
    PhysicsInputs inputs;
    inputs.joint_drives.push_back(JointDrive{0, rest, stiffness, damping, true});
    adapter.set_inputs(inputs);
}

}  // namespace

// ===== Load and step =====================================================

TEST_CASE("native adapter: load refuses an artifact, a second load and unknown elements", "[PHY-001][PHY-019]") {
    NativeAdapter adapter;
    const Doc doc = slider(1.0);
    CHECK_THROWS_AS(adapter.load(PreparedModel{doc.text(), "<mujoco/>", "1.0.0"}, Rate(1000)), AdapterError);
    Doc unknown = slider(1.0);
    unknown.elements = R"(["body", "joint.prismatic", "tendon"])";
    CHECK_THROWS_AS(load(adapter, unknown), AdapterError);
    Doc malformed = slider(1.0);
    malformed.gravity = "[0, 0]";
    CHECK_THROWS_AS(load(adapter, malformed), AdapterError);
    CHECK_THROWS_AS(adapter.load(PreparedModel{"{", "", "1.0.0"}, Rate(1000)), AdapterError);
    load(adapter, doc);
    CHECK_THROWS_AS(load(adapter, doc), AdapterError);
    adapter.reset();
    load(adapter, doc);
    CHECK(adapter.integrator() == "backward_euler");
    CHECK(adapter.timestep() == 1e-3);
}

TEST_CASE("native adapter: calls before load throw and outputs are empty", "[PHY-002]") {
    NativeAdapter adapter;
    CHECK_THROWS_AS(adapter.step(), AdapterError);
    CHECK_THROWS_AS(adapter.set_inputs(PhysicsInputs{}), AdapterError);
    CHECK_THROWS_AS(adapter.disable_constraint(0), AdapterError);
    CHECK_THROWS_AS(adapter.set_transmission_slip(0, 0.5), AdapterError);
    CHECK(adapter.outputs().bodies.empty());
    CHECK(adapter.health().steps == 0);
    CHECK(adapter.supported_elements().size() == 21);
}

TEST_CASE("native adapter: free fall advances one step per call with the step's time", "[PHY-002][PHY-032][PHY-031]") {
    // Semi-implicit Euler under constant g: v_n = -g n h, z_n = -g h^2 n (n + 1) / 2 (exact recurrence).
    NativeAdapter adapter;
    load(adapter, slider(2.0), 500);
    const double h = 1.0 / 500.0;
    REQUIRE(adapter.outputs().bodies.size() == 1);
    for (int n = 1; n <= 10; ++n) {
        adapter.step();
        CHECK(adapter.outputs().steps == n);
        CHECK(adapter.health().steps == n);
    }
    CHECK_THAT(adapter.outputs().joint_positions[0], WithinRel(-g * h * h * 10.0 * 11.0 / 2.0, 1e-12));
    CHECK_THAT(adapter.outputs().bodies[0].linear_velocity[2], WithinRel(-g * h * 10.0, 1e-12));
    CHECK(adapter.health().physics_rate.numerator() == 500);
}

TEST_CASE("native adapter: two runs are bitwise identical", "[PHY-010]") {
    std::vector<double> first;
    std::vector<double> second;
    for (std::vector<double>* out : {&first, &second}) {
        NativeAdapter adapter;
        Doc doc;
        doc.elements = R"(["body", "joint.revolute"])";
        doc.bodies.push_back(body("arm", -1, "revolute", "[1, 0, 0]", 1.0, "[]", "[]", {}, "[0, 0, 0]",
                                  "[0, 0, -0.5]"));
        doc.starts = R"([{"body": 0, "value": 0.3}])";
        load(adapter, doc);
        for (int n = 0; n < 200; ++n) {
            adapter.step();
            out->push_back(adapter.outputs().joint_positions[0]);
            out->push_back(adapter.outputs().joints[0].generalized_force[0]);
        }
    }
    CHECK(first == second);
}

TEST_CASE("native adapter: an engine error ends the run as AdapterError", "[PHY-009]") {
    NativeAdapter adapter;
    load(adapter, slider(1.0));
    PhysicsInputs inputs;
    inputs.generalized_forces = {std::numeric_limits<double>::quiet_NaN()};
    CHECK_THROWS_AS(adapter.set_inputs(inputs), AdapterError);
    PhysicsInputs wrong;
    wrong.generalized_forces = {1.0, 2.0};
    CHECK_THROWS_AS(adapter.set_inputs(wrong), AdapterError);
}

// ===== Joint drives (PHY-034) ============================================

TEST_CASE("native adapter: a drive's static sag and torque are exact", "[PHY-034]") {
    // Backward Euler holds the spring at the end state: at rest K (rest - q) = m g, so q = rest - m g / K, and the
    // read-back torque K (rest - q') - D v' = m g. Damping D = 2 sqrt(K m) (critical) settles it.
    NativeAdapter adapter;
    const double m = 2.0;
    const double k = 4.0e4;
    load(adapter, slider(m));
    for (int n = 0; n < 3000; ++n) {
        drive(adapter, 0.1, k, 2.0 * std::sqrt(k * m));
        adapter.step();
    }
    CHECK_THAT(adapter.outputs().joint_positions[0], WithinAbs(0.1 - m * g / k, 1e-12));
    REQUIRE(adapter.outputs().joint_drive_torques.size() == 1);
    CHECK_THAT(adapter.outputs().joint_drive_torques[0].torque, WithinRel(m * g, 1e-9));
}

TEST_CASE("native adapter: a drive's steady lag behind a moving rest is D u / K", "[PHY-034]") {
    // Rest r_n = u t_n with zero gravity: the steady state v = u, a = 0 needs K (r - q') - D u = 0 at the end state,
    // so q' = r - D u / K behind the same step's rest; any start transient decays.
    NativeAdapter adapter;
    Doc doc = slider(1.0);
    doc.gravity = "[0, 0, 0]";
    load(adapter, doc);
    const double k = 1.0e4;
    const double d = 200.0;
    const double u = 0.05;
    double rest = 0.0;
    for (int n = 1; n <= 4000; ++n) {
        rest = u * n * 1e-3;
        drive(adapter, rest, k, d);
        adapter.step();
    }
    CHECK_THAT(rest - adapter.outputs().joint_positions[0], WithinRel(d * u / k, 1e-9));
    CHECK_THAT(adapter.outputs().joint_drive_torques[0].torque, WithinAbs(0.0, 1e-9));
}

TEST_CASE("native adapter: a drive is refused on a non-coordinate joint and twice on one joint", "[PHY-034]") {
    NativeAdapter adapter;
    load(adapter, slider(1.0));
    PhysicsInputs twice;
    twice.joint_drives = {JointDrive{0, 0.0, 1.0, 0.0, true}, JointDrive{0, 0.0, 1.0, 0.0, true}};
    CHECK_THROWS_AS(adapter.set_inputs(twice), AdapterError);
    PhysicsInputs negative;
    negative.joint_drives = {JointDrive{0, 0.0, -1.0, 0.0, true}};
    CHECK_THROWS_AS(adapter.set_inputs(negative), AdapterError);
    PhysicsInputs outside;
    outside.joint_drives = {JointDrive{3, 0.0, 1.0, 0.0, true}};
    CHECK_THROWS_AS(adapter.set_inputs(outside), AdapterError);
    PhysicsInputs disabled;
    disabled.joint_drives = {JointDrive{0, 0.5, 1.0e4, 0.0, false}};
    adapter.set_inputs(disabled);
    adapter.step();
    CHECK(adapter.outputs().joint_drive_torques[0].torque == 0.0);
}

// ===== Friction and limits (PHY-027, PHY-030; engine ENG-013, ENG-014) ======

TEST_CASE("native adapter: a joint loaded below its friction creeps at v_s L / F", "[PHY-027]") {
    // Engine friction (ruling 38): below the slip threshold v_s the friction is the damper c = F / v_s, so a vertical
    // slider loaded by L = m g < F solves (m + h c) v' = m v - h m g each step: fixed point v* = -v_s m g / F, reached
    // geometrically with ratio m / (m + h c) = 1 / 201 per step (m = 1, F = 20, h = 1 ms): 50 steps leave 1e-115.
    NativeAdapter adapter;
    const double m = 1.0;
    const double f = 20.0;
    const double vs = 1e-4;
    load(adapter, slider(m, DofSpec{"20.0", "null", "null"}));
    for (int n = 0; n < 50; ++n) {
        adapter.step();
    }
    CHECK_THAT(adapter.outputs().bodies[0].linear_velocity[2], WithinRel(-vs * m * g / f, 1e-9));
}

TEST_CASE("native adapter: a sliding joint stops within the engine's frozen-rate bounds", "[PHY-027][PHY-030]") {
    // Horizontal slider, m = 3, F = 1.5, a = F / m. One push step from rest (c = F / v_s): v1 = h P / (m + h F / v_s).
    // Sliding (|v| >= v_s) the engine freezes c = F / |v| at the step start, so with u = |v| / (a h) one step gives
    // u' = u^2 / (u + 1): the speed drops by a h u / (u + 1) (exact Coulomb drops a h). Derivation of the bounds:
    // the decrement d_k = u_k / (u_k + 1) < 1 and 1 - d_k = d_k / u_k <= the integral of du / u over the step, so
    // the steps to fall from u_1 to below u_s = v_s / (a h) number N with u_1 - u_s <= N <= u_1 - u_s +
    // ln(u_1 / u_s) + 1. Distance: x advances h v_{k+1} = a h^2 u_k d_k = a h^2 ((u_k^2 - u_{k+1}^2) / 2 + d_k^2 / 2),
    // so over those steps the excess over (v_1^2 - v_N^2) / (2 a) is a h^2 sum d_k^2 / 2 in [0, h (v_1 - v_N) / 2].
    NativeAdapter adapter;
    const double m = 3.0;
    const double f = 1.5;
    const double a = f / m;
    const double h = 1e-3;
    const double vs = 1e-4;
    Doc doc;
    doc.bodies.push_back(body("slider", -1, "prismatic", "[1, 0, 0]", m, "[]", "[]", DofSpec{"1.5", "null", "null"}));
    load(adapter, doc);
    PhysicsInputs push;
    push.generalized_forces = {3000.0};
    adapter.set_inputs(push);
    adapter.step();
    const double v1 = adapter.outputs().bodies[0].linear_velocity[0];
    const double x1 = adapter.outputs().joint_positions[0];
    CHECK_THAT(v1, WithinRel(h * 3000.0 / (m + h * f / vs), 1e-12));
    double v = v1;
    int steps = 0;
    while (v >= vs && steps < 100000) {
        adapter.step();
        ++steps;
        const double next = adapter.outputs().bodies[0].linear_velocity[0];
        const double u = v / (a * h);
        CHECK_THAT(v - next, WithinRel(a * h * u / (u + 1.0), 1e-9));
        v = next;
    }
    const double u1 = v1 / (a * h);
    const double us = vs / (a * h);
    INFO("steps " << steps << ", bounds [" << u1 - us << ", " << u1 - us + std::log(u1 / us) + 1.0 << "]");
    CHECK(static_cast<double>(steps) >= u1 - us);
    CHECK(static_cast<double>(steps) <= u1 - us + std::log(u1 / us) + 1.0);
    const double excess = (adapter.outputs().joint_positions[0] - x1) - ((v1 * v1) - (v * v)) / (2.0 * a);
    INFO("distance excess " << excess << " of at most " << h * (v1 - v) / 2.0);
    CHECK(excess >= -1e-12);
    CHECK(excess <= (h * (v1 - v) / 2.0) + 1e-12);
}

TEST_CASE("native adapter: a lower limit holds a body on the bound with reaction m g, then releases",
          "[PHY-027]") {
    // Engine limit row (ENG-013, ruling 43): at rest on the bound the row carries m g, the joint coordinate is the
    // bound (no penetration, no projection), and the joint's generalized force S^T f at zero acceleration is m g.
    // An upward force P > m g releases it from the first step: v = h (P - m g) / m.
    NativeAdapter adapter;
    const double m = 2.0;
    const double h = 1e-3;
    load(adapter, slider(m, DofSpec{"0.0", "0.0", "1.0"}));
    for (int n = 0; n < 100; ++n) {
        adapter.step();
    }
    CHECK_THAT(adapter.outputs().joint_positions[0], WithinAbs(0.0, 1e-12));
    CHECK_THAT(adapter.outputs().joints[0].generalized_force[0], WithinRel(m * g, 1e-9));
    PhysicsInputs lift;
    lift.generalized_forces = {3.0 * m * g};
    adapter.set_inputs(lift);
    adapter.step();
    CHECK_THAT(adapter.outputs().bodies[0].linear_velocity[2], WithinRel(h * 2.0 * g, 1e-9));
}

TEST_CASE("native adapter: an upper limit holds a body pushed up against it with reaction -m g", "[PHY-027]") {
    NativeAdapter adapter;
    const double m = 2.0;
    Doc doc = slider(m, DofSpec{"0.0", "-1.0", "0.0"});
    doc.gravity = "[0, 0, 9.81]";
    load(adapter, doc);
    for (int n = 0; n < 100; ++n) {
        adapter.step();
    }
    CHECK_THAT(adapter.outputs().joint_positions[0], WithinAbs(0.0, 1e-12));
    CHECK_THAT(adapter.outputs().joints[0].generalized_force[0], WithinRel(-m * g, 1e-9));
}

TEST_CASE("native adapter: an impact lands on the limit, stops on the next step and gains no energy", "[PHY-027]") {
    // From q0 = 0.1 m above a lower bound at 0 under gravity. Semi-implicit free fall loses g^2 h^2 / 2 per unit mass
    // each step (E_n = g q0 - g^2 h^2 n / 2); the anticipatory row s v >= -gap / h only removes speed and lands the
    // crossing step on the bound (ruling 43), so E = v^2 / 2 + g q never rises; the next step stops it (v = 0).
    NativeAdapter adapter;
    Doc doc = slider(1.0, DofSpec{"0.0", "0.0", "null"});
    doc.starts = R"([{"body": 0, "value": 0.1}])";
    load(adapter, doc);
    double energy = g * 0.1;
    int landed = -1;
    for (int n = 0; n < 400; ++n) {
        adapter.step();
        const double q = adapter.outputs().joint_positions[0];
        const double v = adapter.outputs().bodies[0].linear_velocity[2];
        const double now = (0.5 * v * v) + (g * q);
        CHECK(now <= energy + 1e-12);
        energy = now;
        if (landed < 0 && q <= 1e-12) {
            landed = n;
            CHECK_THAT(q, WithinAbs(0.0, 1e-12));
        } else if (landed >= 0 && n == landed + 1) {
            CHECK_THAT(v, WithinAbs(0.0, 1e-12));
            CHECK_THAT(q, WithinAbs(0.0, 1e-12));
        }
        CHECK(q >= -1e-12);
    }
    CHECK(landed > 0);
}

TEST_CASE("native adapter: limits on a planar joint or start outside a bound are load errors", "[PHY-027][PHY-019]") {
    NativeAdapter adapter;
    Doc outside = slider(1.0, DofSpec{"0.0", "0.0", "1.0"});
    outside.starts = R"([{"body": 0, "value": -0.5}])";
    CHECK_THROWS_AS(load(adapter, outside), AdapterError);
    Doc planar;
    planar.elements = R"(["body", "joint.planar"])";
    std::string plane = body("plate", -1, "planar", "[0, 0, 1]", 1.0);
    plane.replace(plane.find(R"("lower": [])"), 11, R"("lower": [0.0, null, null])");
    plane.replace(plane.find(R"("upper": [])"), 11, R"("upper": [null, null, null])");
    plane.replace(plane.find(R"("friction": [])"), 14, R"("friction": [0.0, 0.0, 0.0])");
    planar.bodies.push_back(plane);
    CHECK_THROWS_AS(load(adapter, planar), AdapterError);
}

// ===== Transmissions (PHY-006, PHY-018, PHY-033) =========================

namespace {

Doc belt(const std::string& slip = "0.0", const std::string& efficiency = "0.8") {
    // Shaft (revolute about z, J = 1e-4) drives a carriage on a vertical prismatic through a belt of radius r.
    Doc doc;
    doc.elements = R"(["body", "joint.revolute", "joint.prismatic", "transmission.belt_pulley"])";
    doc.bodies.push_back(body("shaft", -1, "revolute", "[0, 0, 1]", 0.1, "[]", "[]", {}, "[0, 0, 0]", "[0, 0, 0]",
                              "[1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]"));
    doc.bodies.push_back(body("carriage", -1, "prismatic", "[0, 0, 1]", 2.0, "[]", "[]", {}, "[0.5, 0, 0]"));
    doc.transmissions = R"([{"type": "belt_pulley", "input_body": 0, "output_body": 1, "ratio": 0.01, )"
                        R"("reverses": false, "efficiency": )" + efficiency + R"(, "back_efficiency": )" + efficiency +
                        R"(, "self_locking": false, "stiffness": 1e5, "damping": 50.0, "backlash": 0.0, )"
                        R"("slip": )" + slip + "}]";
    return doc;
}

}  // namespace

TEST_CASE("native adapter: a held belt publishes the shaft load torque r m g", "[PHY-033][PHY-018]") {
    // Static hold: power flow Static gives gain n = r, so the shaft must supply r F with F = m g on the belt; the
    // shaft joint's generalized force is that load torque (PHY-033), and the drive holds it.
    NativeAdapter adapter;
    load(adapter, belt("0.0", "1.0"));
    const double k = 50.0;
    for (int n = 0; n < 4000; ++n) {
        drive(adapter, 0.0, k, 0.5);
        adapter.step();
    }
    const auto& out = adapter.outputs();
    REQUIRE(out.transmissions.size() == 1);
    CHECK_THAT(out.transmissions[0].force, WithinRel(2.0 * g, 1e-6));
    CHECK_THAT(out.transmissions[0].shaft_load_torque, WithinRel(0.01 * 2.0 * g, 1e-6));
    CHECK(out.joints[0].generalized_force[0] == out.transmissions[0].shaft_load_torque);
    CHECK_THAT(out.joint_drive_torques[0].torque, WithinRel(0.01 * 2.0 * g, 1e-6));
}

TEST_CASE("native adapter: two transmissions on one shaft are refused", "[PHY-018]") {
    NativeAdapter adapter;
    Doc doc = belt();
    doc.bodies.push_back(body("other", -1, "prismatic", "[1, 0, 0]", 1.0, "[]", "[]", {}, "[1, 0, 0]"));
    doc.transmissions = doc.transmissions.substr(0, doc.transmissions.size() - 1) + ", " +
                        doc.transmissions.substr(1, doc.transmissions.size() - 2) + "]";
    CHECK_THROWS_AS(load(adapter, doc), AdapterError);
}

TEST_CASE("native adapter: scheduled slip loses the fraction of the shaft travel", "[PHY-006]") {
    // Gravity off; the shaft turns at a constant drive rate; with slip s the engine's offset grows by s n dtheta,
    // published back in shaft units as s dtheta.
    NativeAdapter adapter;
    Doc doc = belt();
    doc.gravity = "[0, 0, 0]";
    load(adapter, doc);
    CHECK_THROWS_AS(adapter.set_transmission_slip(0, 1.5), AdapterError);
    CHECK_THROWS_AS(adapter.set_transmission_slip(3, 0.5), AdapterError);
    adapter.set_transmission_slip(0, 0.25);
    for (int n = 1; n <= 200; ++n) {
        drive(adapter, 0.002 * n, 10.0, 0.2);
        adapter.step();
    }
    const double shaft = adapter.outputs().joint_positions[0];
    CHECK(adapter.outputs().transmissions[0].slipping);
    CHECK_THAT(adapter.outputs().transmissions[0].slip, WithinRel(0.25 * shaft, 1e-9));
}

// ===== Drives through transmissions (PHY-034, ruling 39; PHY-006, PHY-018) ===

namespace {

Doc screw(const std::string& inertia = "1e-2") {
    // Self-locking lead screw (lead 8 mm, n = 1.27e-3 m/rad): shaft J about z, a 2 kg carriage on a vertical prismatic,
    // k = 1e6 N/m, critical damping 2 sqrt(k m). The engine holds it by a one-sided velocity row on the shaft (ruling
    // 48), so the hold does not depend on J (run at 1e-5 and 1e-2).
    Doc doc;
    doc.elements = R"(["body", "joint.revolute", "joint.prismatic", "transmission.lead_screw"])";
    doc.bodies.push_back(body("shaft", -1, "revolute", "[0, 0, 1]", 0.1, "[]", "[]", {}, "[0, 0, 0]", "[0, 0, 0]",
                              "[" + inertia + ", 0, 0, 0, " + inertia + ", 0, 0, 0, " + inertia + "]"));
    doc.bodies.push_back(body("carriage", -1, "prismatic", "[0, 0, 1]", 2.0, "[]", "[]", {}, "[0.5, 0, 0]"));
    doc.transmissions = R"([{"type": "lead_screw", "input_body": 0, "output_body": 1, "ratio": 0.008, )"
                        R"("reverses": false, "efficiency": 0.4, "back_efficiency": 0.4, "self_locking": true, )"
                        R"("stiffness": 1e6, "damping": 2828.42712474619, "backlash": 0.0, "slip": 0.0}])";
    return doc;
}

Doc gear() {
    // Gear pair, ratio 4 (n = 1 / 4), efficiency 0.9: shaft J = 1e-4 and an output of 1e-2 kg m^2, both about z.
    Doc doc;
    doc.elements = R"(["body", "joint.revolute", "transmission.gear_pair"])";
    doc.bodies.push_back(body("shaft", -1, "revolute", "[0, 0, 1]", 0.1, "[]", "[]", {}, "[0, 0, 0]", "[0, 0, 0]",
                              "[1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]"));
    doc.bodies.push_back(body("output", -1, "revolute", "[0, 0, 1]", 1.0, "[]", "[]", {}, "[0.5, 0, 0]", "[0, 0, 0]",
                              "[1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2]"));
    doc.transmissions = R"([{"type": "gear_pair", "input_body": 0, "output_body": 1, "ratio": 4.0, )"
                        R"("reverses": false, "efficiency": 0.9, "back_efficiency": 0.9, "self_locking": false, )"
                        R"("stiffness": 1e3, "damping": 1.0, "backlash": 0.0, "slip": 0.0}])";
    return doc;
}

// Static sag through a lossy transmission (ruling 47, PHY-034 tolerance row). The coupling starts stretched to its
// static force F (the output's start coordinate), the shaft at rest at 0, held by a drive K, D at rest 0; the sag is
// n F / K. Below omega_s the efficiency loss is the shaft damping c = |alpha| n F / omega_s (alpha = 1 / eta - 1
// forward, 1 - eta backward) at the gain n, so while |omega| < omega_s (asserted every step; the start speed is at
// most K (n F / K) / D < omega_s by the choice of D) the scheme is backward Euler on the linear two-DOF system: mass
// diag(J, m_out), stiffness k a a^T + diag(K, 0), damping c_t a a^T + diag(D + c, 0), a = [n, -1]. settling_steps
// returns N with rho^N <= 1e-20 over its step matrix; 1e-20 against the asserted 1e-6 leaves 1e14 for the start
// amplitude and conditioning. The run takes 1 step (the first, at the larger loss) + the larger of the two sides' N,
// the creep time constant tau = (c + D) / K setting the slow root.
constexpr double omega_s = 1e-4;

std::string number(double value) {
    std::array<char, 32> text{};
    std::snprintf(text.data(), text.size(), "%.17g", value);
    return text.data();
}
constexpr double sag_tolerance = 1e-6;

long settling_steps(const Eigen::Matrix2d& m, const Eigen::Matrix2d& k, const Eigen::Matrix2d& d, double h) {
    const Eigen::Matrix2d s = m + (h * d) + (h * h * k);
    const Eigen::Matrix2d si = s.inverse();
    Eigen::Matrix4d a;
    a.topLeftCorner<2, 2>() = Eigen::Matrix2d::Identity() - (h * h * si * k);
    a.topRightCorner<2, 2>() = h * si * m;
    a.bottomLeftCorner<2, 2>() = -h * si * k;
    a.bottomRightCorner<2, 2>() = si * m;
    const Eigen::EigenSolver<Eigen::Matrix4d> solver(a);
    double rho = 0.0;
    for (Eigen::Index i = 0; i < 4; ++i) {
        rho = std::max(rho, std::abs(solver.eigenvalues()[i]));
    }
    REQUIRE(rho < 1.0);
    return static_cast<long>(std::ceil(std::log(1e-20) / std::log(rho)));
}

struct SagCase {
    Doc doc;
    double n = 0.0;
    double force = 0.0;
    double eta = 1.0;
    double shaft_inertia = 0.0;
    double output_inertia = 0.0;
    double coupling_stiffness = 0.0;
    double coupling_damping = 0.0;
    std::vector<double> forces;
};

long sag_settling(const SagCase& c, double k, double d) {
    const double h = 1e-3;
    Eigen::Matrix2d ab;
    ab << c.n * c.n, -c.n, -c.n, 1.0;
    long longest = 0;
    for (const double alpha : {(1.0 / c.eta) - 1.0, 1.0 - c.eta}) {
        const double loss = alpha * c.n * c.force / omega_s;
        Eigen::Matrix2d stiffness = c.coupling_stiffness * ab;
        stiffness(0, 0) += k;
        Eigen::Matrix2d damping = c.coupling_damping * ab;
        damping(0, 0) += d + loss;
        longest = std::max(longest, settling_steps(Eigen::Vector2d(c.shaft_inertia, c.output_inertia).asDiagonal(),
                                                   stiffness, damping, h));
    }
    return 1 + longest;
}

void check_static_sag(const char* name, const SagCase& c, double k, double d) {
    const double target = c.n * c.force / k;
    REQUIRE(k * target / d < omega_s);
    const long derived = sag_settling(c, k, d);
    NativeAdapter adapter;
    load(adapter, c.doc);
    long observed = -1;
    double lag = 0.0;
    for (long i = 0; i < derived; ++i) {
        PhysicsInputs inputs;
        inputs.generalized_forces = c.forces;
        inputs.joint_drives.push_back(JointDrive{0, 0.0, k, d, true});
        adapter.set_inputs(inputs);
        adapter.step();
        REQUIRE(std::abs(adapter.outputs().bodies[0].angular_velocity[2]) < omega_s);
        lag = -adapter.outputs().joint_positions[0];
        if (observed < 0 && std::abs(lag - target) <= sag_tolerance * target) {
            observed = i + 1;
        }
    }
    std::printf("  %s static sag: n F / K %.15g measured %.15g; settling derived %ld steps, observed %ld\n", name,
                target, lag, derived, observed);
    CHECK_THAT(lag, WithinRel(target, sag_tolerance));
}

// Move then stop (ruling 47): the rest ramps at u = 100 omega_s for `ramp` steps (the shaft runs at u, the gain at
// n / eta exactly), then holds. Let L be the last step of the run whose shaft speed is at or above omega_s; from L on
// every step is below omega_s, so the scheme is the linear backward Euler of sag_settling and its count N (rho^N <=
// 1e-20, set by the creep tau = (n F (1 / eta - 1) / omega_s + D) / K) applies from the state at L, whose deviation
// is at most of the order of the moving lag (D u + n F / eta) / K, about 1e2 of the sag, inside the 1e14 margin. The
// run lasts ramp + 4 N steps and asserts L + N within it, then the sag n F / K (1e-6) at L + N and at the end.
void check_stop_sag(const char* name, const SagCase& c, double k, double d, int ramp) {
    const double h = 1e-3;
    const double u = 100.0 * omega_s;
    const double target = c.n * c.force / k;
    const long derived = sag_settling(c, k, d);
    const long horizon = ramp + (4 * derived);
    NativeAdapter adapter;
    load(adapter, c.doc);
    std::vector<double> lags;
    long last_fast = -1;
    for (long i = 0; i < horizon; ++i) {
        const double rest = u * h * static_cast<double>(std::min<long>(i + 1, ramp));
        PhysicsInputs inputs;
        inputs.generalized_forces = c.forces;
        inputs.joint_drives.push_back(JointDrive{0, rest, k, d, true});
        adapter.set_inputs(inputs);
        adapter.step();
        if (std::abs(adapter.outputs().bodies[0].angular_velocity[2]) >= omega_s) {
            last_fast = i;
        }
        lags.push_back(rest - adapter.outputs().joint_positions[0]);
    }
    REQUIRE(last_fast >= ramp / 2);
    REQUIRE(last_fast + derived < horizon);
    long observed = -1;
    for (long i = last_fast + 1; i < horizon && observed < 0; ++i) {
        if (std::abs(lags[static_cast<std::size_t>(i)] - target) <= sag_tolerance * target) {
            observed = i - last_fast;
        }
    }
    const double settled = lags[static_cast<std::size_t>(last_fast + derived)];
    std::printf("  %s stop sag: n F / K %.15g measured %.15g; last step above omega_s %ld, derived %ld more, "
                "observed %ld\n",
                name, target, settled, last_fast + 1, derived, observed);
    CHECK_THAT(settled, WithinRel(target, sag_tolerance));
    CHECK_THAT(lags.back(), WithinRel(target, sag_tolerance));
}

// Runs a drive on joint 0 at rest rate u (end-of-step rest u t_n) for steps; returns rest - q at the end.
double drive_lag(NativeAdapter& adapter, double u, double k, double d, int steps, const std::vector<double>& forces = {}) {
    double rest = 0.0;
    for (int n = 1; n <= steps; ++n) {
        rest = u * n * 1e-3;
        PhysicsInputs inputs;
        inputs.generalized_forces = forces;
        inputs.joint_drives.push_back(JointDrive{0, rest, k, d, true});
        adapter.set_inputs(inputs);
        adapter.step();
    }
    return rest - adapter.outputs().joint_positions[0];
}

}  // namespace

TEST_CASE("native adapter: a belt drive is lossless at rest and follows the efficiency by power direction",
          "[PHY-034]") {
    // Belt n = r = 0.01, eta = 0.8, carriage m = 2 under g, drive K = 50, D = 0.5 on the shaft. At steady state the
    // carriage moves at constant speed, so the belt carries F = m g (its damper sees dE/dt = 0) and the shaft balances
    // K (rest - q') - D w = gain F at the end state (backward Euler on the drive, ruling 42), the gain at or above
    // omega_s exactly (ruling 47):
    //   lifting (w = u > 0, shaft drives the load, P < 0): gain n / eta, lag (D u + n m g / eta) / K;
    //   lowering (w = -u, load drives the shaft, P > 0): gain n eta, lag (-D u + n eta m g) / K.
    // Settling: reflected inertia J + m n^2 = 3e-4 with K = 50, D = 0.5 is overdamped, slow root about 100 1/s, so
    // 3 s leave e^-300 of the start transient. Static sag n m g / K: check_static_sag, at K = 1e5 and D = 2500 so the
    // creep (tau = (c + D) / K, 30 ms) settles within its derived count and the start speed stays below omega_s.
    const double n = 0.01;
    const double eta = 0.8;
    const double m = 2.0;
    const double k = 50.0;
    const double d = 0.5;
    const double u = 1.0;
    {
        SagCase c;
        c.doc = belt("0.0", "0.8");
        c.doc.starts = R"([{"body": 1, "value": )" + number(-m * g / 1e5) + "}]";
        c.n = n;
        c.force = m * g;
        c.eta = eta;
        c.shaft_inertia = 1e-4;
        c.output_inertia = m;
        c.coupling_stiffness = 1e5;
        c.coupling_damping = 50.0;
        check_static_sag("belt", c, 1e5, 2500.0);
        check_stop_sag("belt", c, 1e5, 2500.0, 1000);
    }
    {
        NativeAdapter adapter;
        load(adapter, belt("0.0", "0.8"));
        const double lag = drive_lag(adapter, u, k, d, 3000);
        INFO("lifting lag " << lag << " closed form " << ((d * u) + (n * m * g / eta)) / k);
        CHECK_THAT(lag, WithinRel(((d * u) + (n * m * g / eta)) / k, 1e-6));
    }
    {
        NativeAdapter adapter;
        load(adapter, belt("0.0", "0.8"));
        const double lag = drive_lag(adapter, -u, k, d, 3000);
        INFO("lowering lag " << lag << " closed form " << ((-d * u) + (n * eta * m * g)) / k);
        CHECK_THAT(lag, WithinRel(((-d * u) + (n * eta * m * g)) / k, 1e-6));
    }
}

TEST_CASE("native adapter: a self-locking lead screw holds, lowers under a drive and lifts at n F / eta",
          "[PHY-006][PHY-018][PHY-034]") {
    // ENG-044, ruling 48, at rotor inertia 1e-5 and 1e-2 kg m^2. Hold: with no shaft torque the lock row binds every
    // step, so the shaft stays at exactly 0 (drift exactly zero) and the carriage hangs on the screw's stiffness at
    // -m g / k (critically damped carriage, 707 rad/s: 2 s leave e^-1400; 1e-6). Under a drive K = 10, D = 0.5
    // moving its rest at u = 1 rad/s, the carriage moves at constant speed, so the screw carries F = m g:
    //   lowering (rest -u t): the load would back-drive, the drive pushes the same way, no row, gain 0 (unresisted in
    //   this tier, ENG-044 as written): K (rest - q') + D u = 0, lag -D u / K;
    //   lifting (rest +u t): gain n / eta (u above omega_s), lag (D u + n m g / eta) / K.
    // Settling (both lags 1e-6): J + m n^2 is 1.3e-5 or 1e-2; at 1e-5 the drive mode is overdamped with slow root
    // K / D = 20 1/s, at 1e-2 it decays at D / 2 J = 25 1/s; 3 s leave at most e^-60.
    const double n = 0.008 / (2.0 * std::numbers::pi);
    const double m = 2.0;
    const double eta = 0.4;
    const double k = 10.0;
    const double d = 0.5;
    const double u = 1.0;
    for (const char* inertia : {"1e-5", "1e-2"}) {
        INFO("rotor inertia " << inertia);
        {
            NativeAdapter adapter;
            load(adapter, screw(inertia));
            for (int i = 0; i < 2000; ++i) {
                adapter.step();
                REQUIRE(adapter.outputs().joint_positions[0] == 0.0);
            }
            CHECK_THAT(adapter.outputs().joint_positions[1], WithinRel(-m * g / 1e6, 1e-6));
        }
        {
            NativeAdapter adapter;
            load(adapter, screw(inertia));
            const double lag = drive_lag(adapter, -u, k, d, 3000);
            std::printf("  J %s lowering lag %.15g closed form %.15g\n", inertia, lag, -d * u / k);
            CHECK_THAT(lag, WithinRel(-d * u / k, 1e-6));
        }
        {
            NativeAdapter adapter;
            load(adapter, screw(inertia));
            const double lag = drive_lag(adapter, u, k, d, 3000);
            const double target = ((d * u) + (n * m * g / eta)) / k;
            std::printf("  J %s lifting lag %.15g closed form %.15g\n", inertia, lag, target);
            CHECK_THAT(lag, WithinRel(target, 1e-6));
        }
    }
}

TEST_CASE("native adapter: a belt's backlash is a dead band equal to the lash on reversal", "[PHY-018]") {
    // Gravity off; the shaft is held by a drive at rest 0 (K = 50, D = 0.5) and the carriage pushed by a constant P,
    // first along -z, then reversed to +z. At static equilibrium the belt carries F = P on the side the load presses:
    // E = n theta - x = b / 2 + P / k, then E = -(b / 2 + P / k), so the reversal moves the carriage relative to the
    // shaft by b + 2 P / k: (x_B - x_A) - n (theta_B - theta_A) - 2 P / k = b exactly (no friction, no slip). The belt
    // mode (k = 1e5, m = 2, damping 50, backward Euler at x = 0.22) and the drive mode settle within 2 s to e^-30.
    const double n = 0.01;
    const double b = 0.002;
    const double p = 1.0;
    const double k = 1e5;
    Doc doc = belt("0.0", "1.0");
    doc.gravity = "[0, 0, 0]";
    doc.transmissions.replace(doc.transmissions.find(R"("backlash": 0.0)"), 15, R"("backlash": 0.002)");
    NativeAdapter adapter;
    load(adapter, doc);
    const auto hold = [&adapter](double force) {
        for (int i = 0; i < 2000; ++i) {
            PhysicsInputs inputs;
            inputs.generalized_forces = {0.0, force};
            inputs.joint_drives.push_back(JointDrive{0, 0.0, 50.0, 0.5, true});
            adapter.set_inputs(inputs);
            adapter.step();
        }
    };
    hold(-p);
    const double theta_a = adapter.outputs().joint_positions[0];
    const double x_a = adapter.outputs().joint_positions[1];
    CHECK_THAT((n * theta_a) - x_a, WithinAbs((b / 2.0) + (p / k), 1e-9));
    hold(p);
    const double theta_b = adapter.outputs().joint_positions[0];
    const double x_b = adapter.outputs().joint_positions[1];
    const double band = (x_b - x_a) - (n * (theta_b - theta_a)) - (2.0 * p / k);
    INFO("dead band " << band << " lash " << b);
    CHECK_THAT(band, WithinAbs(b, 1e-9));
}

TEST_CASE("native adapter: a gear pair turns the output at the ratio and loses the efficiency when driving",
          "[PHY-018][PHY-034]") {
    // n = 1 / 4, eta = 0.9, output load torque tau_L = -0.5 N m. Static hold: the gear carries F = 0.5 at gain n, so
    // the shaft sags n F / K (check_static_sag, K = 1e5, D = 1500: creep tau 16 ms). Driving at u = 2 rad/s under
    // K = 1, D = 0.05: the output turns at n u, and the shaft lags (D u + n F / eta) / K. Settling: J + I n^2 =
    // 7.25e-4 with K = 1, D = 0.05 is damped at zeta 0.93 (decay about 34 1/s); 3 s leave e^-100.
    const double n = 0.25;
    const double f = 0.5;
    const double k = 1.0;
    const double d = 0.05;
    const double u = 2.0;
    {
        SagCase c;
        c.doc = gear();
        c.doc.starts = R"([{"body": 1, "value": )" + number(-f / 1e3) + "}]";
        c.n = n;
        c.force = f;
        c.eta = 0.9;
        c.shaft_inertia = 1e-4;
        c.output_inertia = 1e-2;
        c.coupling_stiffness = 1e3;
        c.coupling_damping = 1.0;
        c.forces = {0.0, -f};
        check_static_sag("gear", c, 1e5, 1500.0);
        check_stop_sag("gear", c, 1e5, 1500.0, 1000);
    }
    NativeAdapter adapter;
    load(adapter, gear());
    const double lag = drive_lag(adapter, u, k, d, 3000, {0.0, -f});
    INFO("driving lag " << lag << " closed form " << ((d * u) + (n * f / 0.9)) / k);
    CHECK_THAT(lag, WithinRel(((d * u) + (n * f / 0.9)) / k, 1e-6));
    CHECK_THAT(adapter.outputs().bodies[1].angular_velocity[2], WithinRel(n * u, 1e-6));
}

TEST_CASE("native adapter: substeps interpolate the drive rest so the steady lag is exact", "[PHY-034][PHY-031]") {
    // Ruling 49. A 1 kg slider under g on a drive K = 100, D = 20 whose rest ramps at u = 0.1 m/s (end-of-tick rest
    // u t_n). Each engine step drives toward the rest at its own end time (interpolated from the previous tick's), so
    // backward Euler's steady state K (rest - q') - D u = m g holds at every engine step: lag (D u + m g) / K at 1 and
    // 4 substeps alike (1e-6). Taking the tick's rest at every substep would put the rest u h (S - s) / S ahead and
    // raise the lag by about 3 u h / 8 = 4e-5 m (3e-4 relative). Settling: critically damped at 10 rad/s, 3 s leave
    // 31 e^-30 of the start transient.
    const double m = 1.0;
    const double k = 100.0;
    const double d = 20.0;
    const double u = 0.1;
    for (const int substeps : {1, 4}) {
        Doc doc = slider(m);
        doc.substeps = substeps;
        NativeAdapter adapter;
        load(adapter, doc);
        CHECK(adapter.timestep() == 1e-3 / substeps);
        const double lag = drive_lag(adapter, u, k, d, 3000);
        std::printf("  substeps %d: lag %.15g closed form %.15g\n", substeps, lag, ((d * u) + (m * g)) / k);
        CHECK_THAT(lag, WithinRel(((d * u) + (m * g)) / k, 1e-6));
    }
}

TEST_CASE("native adapter: a joint undriven in the previous tick takes the tick-end rest on every substep",
          "[PHY-034][PHY-031]") {
    // Ruling 49, PHY-034 row. A 1 kg slider without gravity at 4 substeps (h = 2.5e-4): a tick without a drive, then
    // a tick driven to r1 = 0.01 (no previous rest: r1 on all four substeps), then one driven to r2 = 0.02 (rest
    // r1 + (r2 - r1) s / 4 on substep s). Each engine step is the scheme's own recurrence for a drive K = 100, D = 0:
    // (m + h^2 K) dv = h K (r - q) - h^2 K v, v += dv, q += h v (1e-12).
    const double m = 1.0;
    const double k = 100.0;
    const double h = 2.5e-4;
    Doc doc = slider(m);
    doc.gravity = "[0, 0, 0]";
    doc.substeps = 4;
    NativeAdapter adapter;
    load(adapter, doc);
    adapter.step();
    double q = 0.0;
    double v = 0.0;
    const auto engine_step = [&](double rest) {
        v += ((h * k * (rest - q)) - (h * h * k * v)) / (m + (h * h * k));
        q += h * v;
    };
    drive(adapter, 0.01, k, 0.0);
    adapter.step();
    for (int s = 1; s <= 4; ++s) {
        engine_step(0.01);
    }
    CHECK_THAT(adapter.outputs().joint_positions[0], WithinRel(q, 1e-12));
    drive(adapter, 0.02, k, 0.0);
    adapter.step();
    for (int s = 1; s <= 4; ++s) {
        engine_step(0.01 + (0.01 * s / 4.0));
    }
    CHECK_THAT(adapter.outputs().joint_positions[0], WithinRel(q, 1e-12));
}

TEST_CASE("native adapter: the scheme decay record outlives the adapter", "[PHY-031]") {
    std::shared_ptr<const mechsim::physics::native::DrivenSchemeDecayRecord> record;
    {
        Doc doc = slider(1.0);
        NativeAdapter adapter;
        load(adapter, doc);
        record = adapter.scheme_decay_record();
        drive(adapter, 0.0, 1e4, 0.0);
        adapter.step();
    }
    REQUIRE(record->values.size() == 1);
    CHECK_THAT(record->values[0].decay, WithinRel(std::numbers::pi * 0.1, 1e-12));
}

TEST_CASE("native adapter: the largest scheme decay per driven joint is kept over the run", "[PHY-031][PHY-034]") {
    // Rulings 44, 45: a 1 kg slider on a drive K publishes pi omega h, omega = sqrt(K / m). K = 1e4 gives x = 0.1, then
    // K = 100 (x = 0.01): the kept value stays pi 0.1 (1e-12). At 2 substeps h halves, so x = 0.05. An undriven model
    // keeps nothing; the accessor needs a loaded model.
    NativeAdapter unloaded;
    CHECK_THROWS_AS(unloaded.driven_scheme_decay(), AdapterError);
    for (const int substeps : {1, 2}) {
        Doc doc = slider(1.0);
        doc.substeps = substeps;
        NativeAdapter adapter;
        load(adapter, doc);
        adapter.step();
        CHECK(adapter.driven_scheme_decay().empty());
        for (int i = 0; i < 5; ++i) {
            drive(adapter, 0.0, 1e4, 0.0);
            adapter.step();
        }
        for (int i = 0; i < 5; ++i) {
            drive(adapter, 0.0, 100.0, 0.0);
            adapter.step();
        }
        const auto decay = adapter.driven_scheme_decay();
        REQUIRE(decay.size() == 1);
        CHECK(decay[0].joint == 0);
        CHECK_THAT(decay[0].decay, WithinRel(std::numbers::pi * 0.1 / substeps, 1e-12));
    }
}

// ===== Payloads and mid-run changes (PHY-006, PHY-008) ===================

TEST_CASE("native adapter: payload mass change, release and a second release", "[PHY-006][PHY-008]") {
    NativeAdapter adapter;
    Doc doc;
    doc.elements = R"(["body", "joint.prismatic", "joint.free"])";
    doc.bodies.push_back(body("carrier", -1, "prismatic", "[1, 0, 0]", 5.0));
    doc.bodies.push_back(body("payload", -1, "free", "[0, 0, 1]", 1.0, "[]", "[]", {}, "[0, 0, -0.2]"));
    doc.payloads = R"([{"body": 1, "carrier": 0}])";
    load(adapter, doc);
    REQUIRE(adapter.outputs().payloads.size() == 1);
    CHECK(adapter.outputs().payloads[0].attached);
    for (int n = 0; n < 200; ++n) {
        adapter.step();
    }
    // The weld is critically damped at 1 kHz: at rest it carries m g exactly (ENG-035).
    CHECK_THAT(adapter.outputs().payloads[0].force[2], WithinRel(g, 1e-6));
    CHECK_THROWS_AS(adapter.set_payload_mass(0, 2.0), AdapterError);
    adapter.set_payload_mass(1, 2.0);
    adapter.step();
    CHECK(adapter.events().size() == 1);
    CHECK(adapter.events()[0].kind == "payload_mass");
    adapter.release_load(1);
    adapter.step();
    REQUIRE(adapter.events().size() == 1);
    CHECK(adapter.events()[0].kind == "load_released");
    CHECK(adapter.events()[0].source == "payload");
    CHECK_FALSE(adapter.outputs().payloads[0].attached);
    CHECK(adapter.outputs().payloads[0].carrier == mechsim::kernel::kWorldBody);
    CHECK_THROWS_AS(adapter.release_load(1), AdapterError);
    CHECK_THROWS_AS(adapter.release_load(7), AdapterError);
}

TEST_CASE("native adapter: a joint spring offset and stiffness change move the rest", "[PHY-008]") {
    // A revolute spring K0 = 10 with rest 0, heavy damping, no gravity: after set_spring_stiffness(0, 40) and
    // set_spring_offset(0, 0.2) the static rest is 0.2 (the force -K1 (q - 0.2) vanishes there).
    NativeAdapter adapter;
    Doc doc;
    doc.elements = R"(["body", "joint.revolute"])";
    doc.gravity = "[0, 0, 0]";
    doc.bodies.push_back(body("arm", -1, "revolute", "[0, 0, 1]", 1.0, "[10.0]", "[1.0]"));
    load(adapter, doc);
    CHECK_THROWS_AS(adapter.set_spring_offset(1, 0.1), AdapterError);
    CHECK_THROWS_AS(adapter.set_spring_stiffness(0, -1.0), AdapterError);
    adapter.set_spring_stiffness(0, 40.0);
    adapter.set_spring_offset(0, 0.2);
    for (int n = 0; n < 20000; ++n) {
        adapter.step();
    }
    CHECK_THAT(adapter.outputs().joint_positions[0], WithinAbs(0.2, 1e-9));
}

TEST_CASE("native adapter: constraint disable and its index check", "[PHY-008]") {
    NativeAdapter adapter;
    Doc doc;
    doc.elements = R"(["body", "joint.free", "joint.fixed", "constraint.point"])";
    doc.bodies.push_back(body("ball", -1, "free", "[0, 0, 1]", 1.0));
    doc.bodies.push_back(body("world_anchor", -1, "fixed", "[0, 0, 1]", 1.0));
    doc.neutral_bodies = 1;
    doc.constraints = R"([{"type": "point", "body_a": 0, "body_b": 1, "point_a": [0, 0, 0], )"
                      R"("point_b": [0, 0, 0]}])";
    load(adapter, doc);
    adapter.step();
    REQUIRE(adapter.outputs().constraints.size() == 1);
    CHECK_THAT(adapter.outputs().constraints[0].force[2], WithinRel(g, 1e-6));
    CHECK_THROWS_AS(adapter.disable_constraint(1), AdapterError);
    adapter.disable_constraint(0);
    adapter.step();
    CHECK(adapter.events()[0].kind == "constraint_disabled");
    CHECK_THROWS_AS(adapter.disable_constraint(0), AdapterError);
}

// ===== Members and ropes (PHY-004, PHY-007) ==============================

TEST_CASE("native adapter: a clamped member below eight segments is refused", "[PHY-007][PHY-018]") {
    NativeAdapter adapter;
    Doc doc;
    doc.elements = R"(["body", "joint.fixed", "lumped_member"])";
    doc.bodies.push_back(body("base", -1, "fixed", "[0, 0, 1]", 1.0));
    const std::string member = R"({"parent": 0, "position": [0, 0, 0], "orientation": [1, 0, 0, 0], )"
                               R"("youngs_modulus": 2e11, "area_moment": 1e-8, "density": 7800, "area": 1e-4, )"
                               R"("length": 1.0, "node_damping": 0.0, "roller_mass_fraction": 0.001, )";
    doc.members = "[" + member + R"("segments": 4, "ends": "clamped"}])";
    CHECK_THROWS_AS(load(adapter, doc), AdapterError);
    doc.members = "[" + member + R"("segments": 8, "ends": "cantilever"}])";
    load(adapter, doc);
    adapter.step();
    CHECK(adapter.outputs().members.size() == 1);
    // The neutral body, then the member's eight chain bodies root to tip (member node poses, BLD-017).
    REQUIRE(adapter.outputs().bodies.size() == 9);
    CHECK_THAT(adapter.outputs().bodies[1].position[0], WithinAbs(0.5 / 8.0, 1e-12));
    CHECK_THAT(adapter.outputs().bodies[8].position[0], WithinAbs(7.5 / 8.0, 1e-9));
    PhysicsInputs tip;
    tip.body_wrenches.push_back(mechsim::kernel::BodyWrench{8, {}});
    adapter.set_inputs(tip);
    tip.body_wrenches[0].body = 9;
    CHECK_THROWS_AS(adapter.set_inputs(tip), AdapterError);
}

TEST_CASE("native adapter: a rope holding a load publishes its tensions", "[PHY-004]") {
    // A 1 kg load on a 4-segment rope from the world; at rest every segment carries the load plus the rope below it.
    NativeAdapter adapter;
    Doc doc;
    doc.elements = R"(["body", "joint.free", "rope"])";
    doc.bodies.push_back(body("load", -1, "free", "[0, 0, 1]", 1.0, "[]", "[]", {}, "[0, 0, -1]"));
    doc.ropes = R"([{"from_body": -1, "from_point": [0, 0, 0], "to_body": 0, "to_point": [0, 0, 0], )"
                R"("segments": 4, "axial_rigidity": 1e5, "damping": 50.0, "mass_per_length": 0.01, )"
                R"("rest_length": 1.0, "breaking_tension": null, "attachment_damping": 0.0, )"
                R"("attachment_friction": 0.0}])";
    load(adapter, doc);
    for (int n = 0; n < 3000; ++n) {
        adapter.step();
    }
    REQUIRE(adapter.outputs().ropes.size() == 1);
    const std::vector<double> loads = adapter.rope_load_end_tensions();
    // Load end: the load plus half the last segment's mass (the end half-mass sits on the load body).
    CHECK_THAT(loads[0], WithinRel((1.0 + 0.01 * 0.125) * g, 1e-3));
    CHECK(adapter.rope_segment_tensions()[0].size() == 4);
    // Ruling 40: tension is the load end, max_tension the largest segment (the top one: load plus 3.5 segments).
    CHECK_THAT(adapter.outputs().ropes[0].tension, WithinRel((1.0 + 0.01 * 0.125) * g, 1e-3));
    CHECK(adapter.outputs().ropes[0].tension == loads[0]);
    CHECK_THAT(adapter.outputs().ropes[0].max_tension, WithinRel((1.0 + 0.01 * 0.875) * g, 1e-3));
    CHECK_FALSE(adapter.outputs().ropes[0].slack);
    CHECK_THROWS_AS(adapter.release_load(2), AdapterError);
    adapter.release_load(1);
    adapter.step();
    CHECK(adapter.events()[0].source == "rope");
}

// ===== Model change events (PHY-008, ruling 40) ===========================

namespace {

using mechsim::kernel::ModelChangeEvent;
using mechsim::kernel::ModelChangeKind;
using mechsim::kernel::ModelChangeSource;

// A 1 kg load hanging from the world on a 4-segment rope that starts unstretched: the load falls, the rope stretches
// and the tension passes 9.5 N (just under the load's weight; the first stretch overshoots to about 2 m g) a few
// steps in, so the rope snaps. Beside it a free ball held to the world by a point constraint, independent of the
// rope, so a constraint can be disabled at the start of a tick.
Doc snapping_rope(int substeps) {
    Doc doc;
    doc.elements = R"(["body", "joint.free", "joint.fixed", "constraint.point", "rope"])";
    doc.bodies.push_back(body("load", -1, "free", "[0, 0, 1]", 1.0, "[]", "[]", {}, "[0, 0, -1]"));
    doc.bodies.push_back(body("ball", -1, "free", "[0, 0, 1]", 1.0, "[]", "[]", {}, "[2, 0, 0]"));
    doc.bodies.push_back(body("world_anchor", -1, "fixed", "[0, 0, 1]", 1.0, "[]", "[]", {}, "[2, 0, 0]"));
    doc.neutral_bodies = 2;
    doc.constraints = R"([{"type": "point", "body_a": 1, "body_b": 2, "point_a": [0, 0, 0], )"
                      R"("point_b": [0, 0, 0]}])";
    doc.ropes = R"([{"from_body": -1, "from_point": [0, 0, 0], "to_body": 0, "to_point": [0, 0, 0], )"
                R"("segments": 4, "axial_rigidity": 1e5, "damping": 50.0, "mass_per_length": 0.01, )"
                R"("rest_length": 1.0, "breaking_tension": 9.5, "attachment_damping": 0.0, )"
                R"("attachment_friction": 0.0}])";
    doc.substeps = substeps;
    return doc;
}

struct Tick {
    std::int64_t tick = 0;
    ModelChangeEvent change;
};

std::vector<Tick> run_ticks(NativeAdapter& adapter, int ticks) {
    std::vector<Tick> out;
    for (int n = 1; n <= ticks; ++n) {
        adapter.step();
        for (const ModelChangeEvent& change : adapter.outputs().model_changes) {
            out.push_back(Tick{n, change});
        }
    }
    return out;
}

}  // namespace

TEST_CASE("native adapter: a snap lists RopeSnapped then LoadReleased, each once", "[PHY-004][PHY-008]") {
    // The engine detects the snap at the end of engine step k and releases the load in the same call (one
    // release_load path, ENG-047): both events belong to step k, RopeSnapped first.
    NativeAdapter reference;
    load(reference, snapping_rope(0));
    const std::vector<Tick> events = run_ticks(reference, 400);
    REQUIRE(events.size() == 2);
    CHECK(events[0].change.kind == ModelChangeKind::RopeSnapped);
    CHECK(events[0].change.source == ModelChangeSource::Rope);
    CHECK(events[0].change.index == 0);
    CHECK(events[0].change.value > 9.5);
    CHECK(events[1].change.kind == ModelChangeKind::LoadReleased);
    CHECK(events[1].change.source == ModelChangeSource::Rope);
    CHECK(events[1].change.index == 0);
    CHECK(events[1].tick == events[0].tick);
    const std::int64_t k = events[0].tick;
    REQUIRE(k >= 2);

    SECTION("two engine steps per tick: each event once, in the tick holding its engine step") {
        NativeAdapter adapter;
        adapter.load(PreparedModel{snapping_rope(2).text(), "", "1.0.0"}, Rate(500));
        CHECK(adapter.timestep() == 1e-3);
        const std::vector<Tick> paired = run_ticks(adapter, 200);
        REQUIRE(paired.size() == 2);
        CHECK(paired[0].change.kind == ModelChangeKind::RopeSnapped);
        CHECK(paired[0].change.value == events[0].change.value);
        CHECK(paired[0].tick == (k + 1) / 2);
        CHECK(paired[1].change.kind == ModelChangeKind::LoadReleased);
        CHECK(paired[1].tick == (k + 1) / 2);
    }
    SECTION("events from the first and the last engine step of one tick are both listed, in step order") {
        const int substeps = static_cast<int>(k);
        NativeAdapter adapter;
        adapter.load(PreparedModel{snapping_rope(substeps).text(), "", "1.0.0"}, Rate(1000, substeps));
        CHECK(adapter.timestep() == 1e-3);
        adapter.disable_constraint(0);
        adapter.step();
        const std::vector<ModelChangeEvent> first = adapter.outputs().model_changes;
        REQUIRE(first.size() == 3);
        CHECK(first[0].kind == ModelChangeKind::ConstraintDisabled);
        CHECK(first[0].source == ModelChangeSource::Constraint);
        CHECK(first[0].index == 0);
        CHECK(first[1].kind == ModelChangeKind::RopeSnapped);
        CHECK(first[1].value == events[0].change.value);
        CHECK(first[2].kind == ModelChangeKind::LoadReleased);
        CHECK(adapter.events().size() == 3);
        adapter.step();
        CHECK(adapter.outputs().model_changes.empty());
        CHECK(adapter.events().empty());
    }
    SECTION("run twice bitwise") {
        std::array<std::vector<double>, 2> runs;
        for (std::vector<double>& out : runs) {
            NativeAdapter adapter;
            adapter.load(PreparedModel{snapping_rope(2).text(), "", "1.0.0"}, Rate(500));
            for (int n = 0; n < 100; ++n) {
                adapter.step();
                out.push_back(adapter.outputs().ropes[0].tension);
                out.push_back(adapter.outputs().ropes[0].max_tension);
                out.push_back(adapter.outputs().bodies[0].position[2]);
                for (const ModelChangeEvent& change : adapter.outputs().model_changes) {
                    out.push_back(static_cast<double>(change.kind));
                    out.push_back(change.value);
                }
            }
        }
        CHECK(runs[0] == runs[1]);
    }
}

TEST_CASE("native adapter: substeps below one are refused", "[PHY-031]") {
    NativeAdapter adapter;
    Doc doc = slider(1.0);
    doc.substeps = -1;
    CHECK_THROWS_AS(load(adapter, doc), AdapterError);
}

TEST_CASE("native adapter: joint and member spring changes are listed once, in the next tick", "[PHY-008]") {
    NativeAdapter adapter;
    Doc doc;
    doc.elements = R"(["body", "joint.revolute", "joint.fixed", "lumped_member"])";
    doc.gravity = "[0, 0, 0]";
    doc.bodies.push_back(body("arm", -1, "revolute", "[0, 0, 1]", 1.0, "[10.0]", "[1.0]"));
    doc.bodies.push_back(body("base", -1, "fixed", "[0, 0, 1]", 1.0));
    doc.members = R"([{"parent": 1, "position": [0, 0, 0], "orientation": [1, 0, 0, 0], "youngs_modulus": 2e11, )"
                  R"("area_moment": 1e-8, "density": 7800, "area": 1e-4, "length": 1.0, "node_damping": 0.0, )"
                  R"("roller_mass_fraction": 0.001, "segments": 8, "ends": "cantilever"}])";
    doc.substeps = 2;
    adapter.load(PreparedModel{doc.text(), "", "1.0.0"}, Rate(500));
    adapter.set_spring_stiffness(0, 40.0);
    adapter.set_spring_offset(0, 0.2);
    adapter.set_spring_offset(3, 0.01);
    adapter.step();
    const std::vector<ModelChangeEvent> changes = adapter.outputs().model_changes;
    REQUIRE(changes.size() == 3);
    CHECK(changes[0].kind == ModelChangeKind::SpringStiffness);
    CHECK(changes[0].source == ModelChangeSource::Spring);
    CHECK(changes[0].index == 0);
    CHECK(changes[0].value == 40.0);
    CHECK(changes[1].kind == ModelChangeKind::SpringOffset);
    CHECK(changes[1].value == 0.2);
    CHECK(changes[2].kind == ModelChangeKind::SpringOffset);
    CHECK(changes[2].index == 3);
    CHECK(changes[2].value == 0.01);
    // The engine's own per-node event stays on events() only.
    REQUIRE(adapter.events().size() == 1);
    CHECK(adapter.events()[0].kind == "spring_offset");
    adapter.step();
    CHECK(adapter.outputs().model_changes.empty());
}

// ===== JSON reader =======================================================

TEST_CASE("native adapter: the JSON reader takes the translation's subset and refuses the rest", "[PHY-001]") {
    using mechsim::physics::native::json::ParseError;
    using mechsim::physics::native::json::Value;
    const Value v = Value::parse(R"({"a": [1, -2.5e-3, true, false, null], "b": "x\"\u0041"})");
    CHECK(v.at("a").array("a").size() == 5);
    CHECK(v.at("a").array("a")[1].number("n") == -2.5e-3);
    CHECK(v.at("b").string("b") == R"(x"A)");
    CHECK(v.at("a").array("a")[4].is_null());
    CHECK_THROWS_AS(Value::parse("[1, 2"), ParseError);
    CHECK_THROWS_AS(Value::parse("{} x"), ParseError);
    CHECK_THROWS_AS(Value::parse("[nan]"), ParseError);
    CHECK_THROWS_AS(v.at("missing"), ParseError);
    CHECK_THROWS_AS(v.at("b").number("b"), ParseError);
    CHECK_THROWS_AS(Value::parse("[1.5]").array("x")[0].integer("x"), ParseError);
}

TEST_CASE("native adapter: the JSON reader refuses duplicate keys and non-JSON numbers", "[PHY-009]") {
    using mechsim::physics::native::json::ParseError;
    using mechsim::physics::native::json::Value;
    CHECK_THROWS_AS(Value::parse(R"({"a": 1, "a": 2})"), ParseError);
    CHECK_THROWS_AS(Value::parse(R"({"a": {"b": 1, "b": 1}})"), ParseError);
    for (const char* bad : {"[+1]", "[01]", "[-01]", "[.5]", "[1.]", "[-.5]", "[0x10]", "[NaN]", "[nan]", "[Infinity]",
                            "[-Infinity]", "[inf]", "[1e]", "[1e+]", "[--1]", "[-]", "[1e999]", R"(["\x"])",
                            "[\"a\tb\"]"}) {
        CAPTURE(bad);
        CHECK_THROWS_AS(Value::parse(bad), ParseError);
    }
    for (const char* good : {"[0]", "[-0]", "[10]", "[0.5]", "[-0.5e-3]", "[1E+2]", "[1e2]", R"(["\/\\\""])"}) {
        CAPTURE(good);
        CHECK_NOTHROW(Value::parse(good));
    }
    CHECK(Value::parse("[1E+2]").array("x")[0].number("x") == 100.0);
}

TEST_CASE("native adapter: the JSON reader refuses empty, truncated and too deeply nested input", "[PHY-009]") {
    using mechsim::physics::native::json::ParseError;
    using mechsim::physics::native::json::Value;
    CHECK_THROWS_AS(Value::parse(""), ParseError);
    CHECK_THROWS_AS(Value::parse("   "), ParseError);
    const std::string text = slider(1.0).text();
    for (const std::size_t cut : {std::size_t{1}, text.size() / 2, text.size() - 1}) {
        CAPTURE(cut);
        CHECK_THROWS_AS(Value::parse(text.substr(0, cut)), ParseError);
    }
    // Depth cap: 64 nested arrays parse, 65 are refused.
    const std::string deep64 = std::string(64, '[') + std::string(64, ']');
    const std::string deep65 = std::string(65, '[') + std::string(65, ']');
    CHECK_NOTHROW(Value::parse(deep64));
    CHECK_THROWS_AS(Value::parse(deep65), ParseError);
    NativeAdapter adapter;
    CHECK_THROWS_AS(adapter.load(PreparedModel{"", "", "1.0.0"}, Rate(1000)), AdapterError);
    CHECK_THROWS_AS(adapter.load(PreparedModel{text.substr(0, text.size() - 1), "", "1.0.0"}, Rate(1000)),
                    AdapterError);
}

TEST_CASE("native adapter: every body index in the document is range-checked with the field named", "[PHY-009]") {
    // Each case swaps one index for one outside its range; the load ends with AdapterError naming the field.
    const auto refused = [](const Doc& doc, const std::string& field) {
        NativeAdapter adapter;
        try {
            load(adapter, doc);
        } catch (const AdapterError& error) {
            CAPTURE(error.cause());
            CHECK(error.cause().find(field) != std::string::npos);
            return;
        }
        FAIL("loaded a document with a bad " + field);
    };
    Doc parent = slider(1.0);
    parent.bodies[0] = body("slider", 0, "prismatic", "[0, 0, 1]", 1.0);
    refused(parent, "body parent");
    Doc neutral = slider(1.0);
    neutral.neutral_bodies = 2;
    refused(neutral, "neutral_bodies");
    Doc start = slider(1.0);
    start.starts = R"([{"body": 1, "value": 0.1}])";
    refused(start, "start body");
    Doc negative = slider(1.0);
    negative.starts = R"([{"body": -1, "value": 0.1}])";
    refused(negative, "start body");
    Doc constraint = slider(1.0);
    constraint.elements = R"(["body", "joint.prismatic", "constraint.point"])";
    constraint.constraints = R"([{"type": "point", "body_a": 0, "body_b": 5, "point_a": [0, 0, 0], )"
                             R"("point_b": [0, 0, 0]}])";
    refused(constraint, "constraint body_b");
    Doc member = slider(1.0);
    member.elements = R"(["body", "joint.prismatic", "lumped_member"])";
    member.members = R"([{"parent": 3, "position": [0, 0, 0], "orientation": [1, 0, 0, 0], "youngs_modulus": 2e11, )"
                     R"("area_moment": 1e-8, "density": 7800, "area": 1e-4, "length": 1.0, "node_damping": 0.0, )"
                     R"("roller_mass_fraction": 0.001, "segments": 8, "ends": "cantilever"}])";
    refused(member, "member parent");
    Doc rope = slider(1.0);
    rope.elements = R"(["body", "joint.prismatic", "rope"])";
    rope.ropes = R"([{"from_body": -2, "from_point": [0, 0, 0], "to_body": 0, "to_point": [0, 0, 0], )"
                 R"("segments": 4, "axial_rigidity": 1e5, "damping": 50.0, "mass_per_length": 0.01, )"
                 R"("rest_length": 1.0, "breaking_tension": null, "attachment_damping": 0.0, )"
                 R"("attachment_friction": 0.0}])";
    refused(rope, "rope from_body");
    Doc payload = slider(1.0);
    payload.payloads = R"([{"body": 4, "carrier": 0}])";
    refused(payload, "payload body");
    Doc geometry = slider(1.0);
    geometry.elements = R"(["body", "joint.prismatic", "contacts"])";
    geometry.contact = R"({"default": {"stiffness": 1e5, "damping": 10, "friction": 0.5}, "pairs": [], )"
                       R"("exclusions": [], "slip_velocity": 1e-4, "geometries": [{"kind": "sphere", "body": 7, )"
                       R"("position": [0, 0, 0], "orientation": [1, 0, 0, 0], "radius": 0.1, "length": 0, )"
                       R"("size": [0, 0, 0], "vertices": [], "material": "steel"}]})";
    refused(geometry, "geometry body");
}
