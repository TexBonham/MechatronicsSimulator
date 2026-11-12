#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
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

// ===== Friction and limits (PHY-027) =====================================

TEST_CASE("native adapter: Coulomb joint friction decelerates at F / m", "[PHY-027][PHY-030]") {
    // Horizontal slider (gravity off the axis). From rest the regularised friction is the damper F / v_s, so one step
    // of force P gives v1 = h P / (m + h F / v_s) (1/6 m/s here), past v_s: sliding, where the constant -F gives
    // dv = -F h / m exactly per step, until the velocity re-enters the regularised band and stops.
    NativeAdapter adapter;
    const double m = 3.0;
    const double f = 1.5;
    Doc doc;
    doc.elements = R"(["body", "joint.prismatic"])";
    doc.bodies.push_back(body("slider", -1, "prismatic", "[1, 0, 0]", m, "[]", "[]", DofSpec{"1.5", "null", "null"}));
    load(adapter, doc);
    PhysicsInputs push;
    push.generalized_forces = {3000.0};
    adapter.set_inputs(push);
    adapter.step();
    const double v1 = adapter.outputs().bodies[0].linear_velocity[0];
    CHECK_THAT(v1, WithinRel(1e-3 * 3000.0 / (m + 1e-3 * f / 1e-4), 1e-12));
    adapter.step();
    const double v2 = adapter.outputs().bodies[0].linear_velocity[0];
    CHECK_THAT(v1 - v2, WithinRel(f * 1e-3 / m, 1e-9));
    for (int n = 0; n < 2000; ++n) {
        adapter.step();
    }
    CHECK(std::abs(adapter.outputs().bodies[0].linear_velocity[0]) < 1e-4);
}

TEST_CASE("native adapter: a joint limit holds a body under gravity at the stated penetration", "[PHY-027]") {
    // Lower limit at 0 on a vertical slider: the implicit limit spring K = m w^2, w = 1 / (2 h), D = 2 m w (critical)
    // holds m g at q = -g / w^2 = -4 g h^2.
    NativeAdapter adapter;
    load(adapter, slider(1.0, DofSpec{"0.0", "0.0", "1.0"}));
    for (int n = 0; n < 500; ++n) {
        adapter.step();
    }
    const double h = 1e-3;
    CHECK_THAT(adapter.outputs().joint_positions[0], WithinRel(-4.0 * g * h * h, 1e-6));
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
