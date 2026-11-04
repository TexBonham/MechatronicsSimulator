#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/physics/mujoco/mujoco_adapter.hpp"

namespace {

using mechsim::kernel::AdapterError;
using mechsim::kernel::PhysicsInputs;
using mechsim::kernel::PhysicsOutputs;
using mechsim::kernel::PreparedModel;
using mechsim::kernel::Rate;
using mechsim::physics::mujoco::MujocoAdapter;

// Pendulum constants, as written in schemas/examples/physics_model.pendulum.yaml (ENG-036, ENG-037).
constexpr double mass = 1.0;
constexpr double com_distance = 0.25;
constexpr double inertia_com = 0.020833333333333332;
constexpr double inertia_pivot = inertia_com + mass * com_distance * com_distance;
constexpr double damping = 0.04520785330006282;
constexpr double gravity = 9.81;
constexpr double h = 1e-3;

std::string fixture(const std::string& name) {
    const std::ifstream file(std::string(MECHSIM_MUJOCO_FIXTURES) + "/" + name);
    REQUIRE(file.good());
    std::stringstream text;
    text << file.rdbuf();
    return text.str();
}

std::unique_ptr<MujocoAdapter> loaded(const std::string& name, const Rate& rate = Rate(1000)) {
    auto adapter = std::make_unique<MujocoAdapter>();
    adapter->load(PreparedModel{"", fixture(name), "0.1.0"}, rate);
    return adapter;
}

double relative(double measured, double target) {
    return std::abs(measured - target) / std::abs(target);
}

// The pendulum hangs along -y: body orientation = Rz(-pi/2) Rz(q), so q = 2 atan2(z, w) + pi/2.
double pendulum_angle(const PhysicsOutputs& outputs) {
    const auto& quat = outputs.bodies.at(0).orientation;
    double angle = 2.0 * std::atan2(quat[3], quat[0]) + std::numbers::pi / 2.0;
    if (angle > std::numbers::pi) {
        angle -= 2.0 * std::numbers::pi;
    }
    return angle;
}

PhysicsInputs torque(double value) {
    PhysicsInputs inputs;
    inputs.generalized_forces = {value};
    return inputs;
}

void replace(std::string& text, const std::string& from, const std::string& to) {
    const std::size_t at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
}

}  // namespace

TEST_CASE("MuJoCo adapter declares the translation's element kinds", "[PHY-001][PHY-019]") {
    const MujocoAdapter adapter;
    const std::span<const std::string> kinds = adapter.supported_elements();
    REQUIRE(kinds.size() == 20);
    CHECK(kinds.front() == "body");
    CHECK(kinds.back() == "transmission.gear_pair");
}

TEST_CASE("MuJoCo adapter runs implicitfast at the run's physics step", "[PHY-014][PHY-031]") {
    const auto adapter = loaded("pendulum.xml");
    CHECK(adapter->integrator() == "implicitfast");
    CHECK(adapter->timestep() == 1e-3);
    const auto fast = loaded("pendulum.xml", Rate(2000));
    CHECK(fast->timestep() == 5e-4);
    CHECK(adapter->timestep() == 1e-3);
    CHECK(fast->health().physics_rate == Rate(2000));
}

TEST_CASE("MuJoCo adapter refuses calls out of order", "[PHY-001][PHY-032]") {
    MujocoAdapter adapter;
    CHECK_THROWS_AS(adapter.step(), AdapterError);
    CHECK_THROWS_AS(adapter.outputs(), AdapterError);
    CHECK_THROWS_AS(adapter.health(), AdapterError);
    CHECK_THROWS_AS(adapter.set_inputs(PhysicsInputs{}), AdapterError);
    CHECK_THROWS_AS(adapter.set_spring_offset(0, 0.0), AdapterError);
    CHECK_THROWS_AS(adapter.set_spring_stiffness(0, 0.0), AdapterError);
    CHECK_THROWS_AS(adapter.disable_constraint(0), AdapterError);
    CHECK_THROWS_AS(adapter.release_load(0), AdapterError);
    CHECK_THROWS_AS(adapter.set_payload_mass(0, 1.0), AdapterError);
    CHECK_THROWS_AS(adapter.set_payload_attachment(0, 0), AdapterError);
    const PreparedModel model{"", fixture("pendulum.xml"), "0.1.0"};
    adapter.load(model, Rate(1000));
    CHECK_THROWS_AS(adapter.load(model, Rate(1000)), AdapterError);
    adapter.step();
    adapter.reset();
    CHECK_THROWS_AS(adapter.step(), AdapterError);
    adapter.load(model, Rate(1000));
    CHECK(adapter.outputs().steps == 0);
}

TEST_CASE("MuJoCo adapter refuses an artifact it cannot map, naming why", "[PHY-019][PHY-009]") {
    MujocoAdapter adapter;
    SECTION("unsupported element kind") {
        std::string text = fixture("pendulum.xml");
        replace(text, "data=\"body joint.revolute", "data=\"body rope joint.revolute");
        try {
            adapter.load(PreparedModel{"", text, "0.1.0"}, Rate(1000));
            FAIL("load accepted an unsupported element");
        } catch (const AdapterError& error) {
            CHECK(error.cause().find("'rope'") != std::string::npos);
        }
    }
    SECTION("no mechsim element list") {
        std::string text = fixture("pendulum.xml");
        replace(text, "mechsim:elements", "other:elements");
        CHECK_THROWS_AS(adapter.load(PreparedModel{"", text, "0.1.0"}, Rate(1000)), AdapterError);
    }
    SECTION("listed body missing from the MJCF") {
        std::string text = fixture("pendulum.xml");
        replace(text, R"(name="mechsim:bodies" data="link")", R"(name="mechsim:bodies" data="ghost")");
        CHECK_THROWS_AS(adapter.load(PreparedModel{"", text, "0.1.0"}, Rate(1000)), AdapterError);
    }
    SECTION("empty artifact") {
        CHECK_THROWS_AS(adapter.load(PreparedModel{"", "", "0.1.0"}, Rate(1000)), AdapterError);
    }
    SECTION("MJCF that does not parse") {
        CHECK_THROWS_AS(adapter.load(PreparedModel{"", "<mujoco><worldbody>", "0.1.0"}, Rate(1000)), AdapterError);
    }
    SECTION("MJCF that does not compile") {
        std::string text = fixture("pendulum.xml");
        replace(text, "mass=\"1.0\"", "mass=\"-1.0\"");
        CHECK_THROWS_AS(adapter.load(PreparedModel{"", text, "0.1.0"}, Rate(1000)), AdapterError);
    }
    CHECK_THROWS_AS(adapter.step(), AdapterError);
}

TEST_CASE("MuJoCo adapter publishes poses, velocities and forces at step 0 and every step",
          "[PHY-002][PHY-003][PHY-012][PHY-032]") {
    auto adapter = loaded("pendulum.xml");
    const PhysicsOutputs& start = adapter->outputs();
    REQUIRE(start.steps == 0);
    REQUIRE(start.bodies.size() == 1);
    REQUIRE(start.joints.size() == 1);
    CHECK(start.joints[0].generalized_force.size() == 1);
    CHECK(start.bodies[0].position == mechsim::kernel::Vec3{0.0, 0.0, 0.0});
    CHECK(std::abs(start.bodies[0].orientation[0] - std::sqrt(0.5)) < 1e-15);
    CHECK(std::abs(start.bodies[0].orientation[3] + std::sqrt(0.5)) < 1e-15);
    CHECK(start.constraints.empty());
    CHECK(start.members.empty());
    CHECK(start.ropes.empty());
    CHECK(start.transmissions.empty());
    // Hanging at rest, the pivot carries the weight: parent on child is m g along +y.
    CHECK(std::abs(start.joints[0].reaction.force[1] - mass * gravity) < 1e-12);
    for (int i = 1; i <= 3; ++i) {
        adapter->step();
        CHECK(adapter->outputs().steps == i);
        CHECK(adapter->health().steps == i);
    }
    CHECK(adapter->health().converged);
    CHECK(adapter->health().max_constraint_drift == 0.0);
}

// One implicitfast step of the hinge: (I + h D) v1 = I v0 + h (tau - m g d sin q0 - k (q0 - r)),
// i.e. damping implicit and every position force explicit at q0; q1 = q0 + h v1.
TEST_CASE("MuJoCo adapter applies inputs for one step, then clears them", "[PHY-005]") {
    constexpr double tau = 2.0;
    const double v1 = h * tau / (inertia_pivot + h * damping);
    const double q1 = h * v1;
    const double v2 = (inertia_pivot * v1 - h * mass * gravity * com_distance * std::sin(q1)) /
                      (inertia_pivot + h * damping);

    SECTION("generalized force") {
        auto adapter = loaded("pendulum.xml");
        adapter->set_inputs(torque(tau));
        adapter->step();
        CHECK(relative(adapter->outputs().bodies[0].angular_velocity[2], v1) < 1e-12);
        adapter->step();
        CHECK(relative(adapter->outputs().bodies[0].angular_velocity[2], v2) < 1e-12);
    }
    SECTION("body wrench") {
        auto adapter = loaded("pendulum.xml");
        PhysicsInputs inputs;
        inputs.body_wrenches.push_back({0, {{0.0, 0.0, tau}, {0.0, 0.0, 0.0}}});
        adapter->set_inputs(inputs);
        adapter->step();
        CHECK(relative(adapter->outputs().bodies[0].angular_velocity[2], v1) < 1e-12);
        adapter->step();
        CHECK(relative(adapter->outputs().bodies[0].angular_velocity[2], v2) < 1e-12);
    }
    SECTION("malformed inputs") {
        auto adapter = loaded("pendulum.xml");
        PhysicsInputs two;
        two.generalized_forces = {1.0, 2.0};
        CHECK_THROWS_AS(adapter->set_inputs(two), AdapterError);
        CHECK_THROWS_AS(adapter->set_inputs(torque(std::nan(""))), AdapterError);
        PhysicsInputs stranger;
        stranger.body_wrenches.push_back({1, {}});
        CHECK_THROWS_AS(adapter->set_inputs(stranger), AdapterError);
        PhysicsInputs infinite;
        infinite.body_wrenches.push_back({0, {{0.0, 0.0, 0.0}, {INFINITY, 0.0, 0.0}}});
        CHECK_THROWS_AS(adapter->set_inputs(infinite), AdapterError);
        adapter->step();
        CHECK(adapter->outputs().bodies[0].angular_velocity[2] == 0.0);
    }
}

// PHY-028 against the compound pendulum closed form (ENG-036 tolerance 0.1 % of period; brief M1-C, P1).
//
// Closed form: I = I_com + m d^2, omega0 = sqrt(m g d / I), zeta = D / (2 I omega0); the damped period is
// Td = 2 pi / (omega0 sqrt(1 - zeta^2)), and the zero crossings of a damped linear oscillator are exactly Td / 2
// apart, so 20 periods between the first and the 21st upward crossing measure Td.
//
// Scheme (separate check): implicitfast takes damping implicitly and gravity explicitly. Linearized,
//   v' = (v - h omega0^2 q) / c,  q' = q + h v',  c = 1 + h D / I,
// whose matrix has det 1 / c and trace (c + 1 - h^2 omega0^2) / c. Its eigenvalues r e^{+-i theta} have r^2 = 1 / c
// and 2 r cos theta = trace, so with a = c + 1 - h^2 omega0^2,
//   theta = atan2(sqrt(4 c - a^2), a),  T_scheme = 2 pi h / theta.
// The discrete solution r^n sin(n theta + phi) crosses zero exactly every pi / theta steps.
// Bound of the scheme check: the amplitude A0 <= 0.02 rad lengthens the period by at most A0^2 / 16 = 2.5e-5
// (sin q against q); linear interpolation of a crossing errs by about h^2 zeta omega0 / 4 = 7e-8 s, the same at
// every crossing, so it cancels in the difference. Asserted: 5e-5, twice the amplitude term.
TEST_CASE("MuJoCo pendulum recovers the damped compound-pendulum period", "[PHY-028][reference]") {
    const double omega0 = std::sqrt(mass * gravity * com_distance / inertia_pivot);
    const double zeta = damping / (2.0 * inertia_pivot * omega0);
    const double damped_period = 2.0 * std::numbers::pi / (omega0 * std::sqrt(1.0 - zeta * zeta));
    const double c = 1.0 + h * damping / inertia_pivot;
    const double a = c + 1.0 - h * h * omega0 * omega0;
    const double theta = std::atan2(std::sqrt(4.0 * c - a * a), a);
    const double scheme_period = 2.0 * std::numbers::pi * h / theta;

    // An impulse sets the amplitude: v1 = h tau / (I + h D), A0 ~ v1 / omega0 = 0.0199 rad for tau = 9 N m.
    constexpr double tau = 9.0;
    const double amplitude = h * tau / (inertia_pivot + h * damping) / omega0;
    REQUIRE(amplitude <= 0.02);

    auto adapter = loaded("pendulum.xml");
    adapter->set_inputs(torque(tau));
    std::vector<double> crossings;
    double previous = pendulum_angle(adapter->outputs());
    const auto steps = static_cast<int>(25.0 * damped_period / h);
    for (int n = 1; n <= steps && crossings.size() < 21; ++n) {
        adapter->step();
        const double angle = pendulum_angle(adapter->outputs());
        if (previous < 0.0 && angle >= 0.0) {
            crossings.push_back((n - 1 + previous / (previous - angle)) * h);
        }
        previous = angle;
    }
    REQUIRE(crossings.size() == 21);
    const double measured = (crossings.back() - crossings.front()) / 20.0;
    std::cout << "PHY-028 pendulum: closed-form damped period " << damped_period << " s, scheme period "
              << scheme_period << " s, measured " << measured << " s, error "
              << 100.0 * relative(measured, damped_period) << " %\n";
    CHECK(relative(measured, damped_period) <= 1e-3);
    CHECK(relative(measured, scheme_period) <= 5e-5);
}

TEST_CASE("MuJoCo adapter runs twice bitwise identical", "[PHY-010]") {
    const auto run = [] {
        auto adapter = loaded("mechanism.xml");
        std::vector<double> trace;
        for (int n = 0; n < 500; ++n) {
            if (n == 0) {
                PhysicsInputs inputs;
                inputs.body_wrenches.push_back({0, {{0.0, 0.0, 0.3}, {0.0, 1.0, 0.0}}});
                adapter->set_inputs(inputs);
            }
            if (n == 250) {
                adapter->disable_constraint(0);
            }
            adapter->step();
            const PhysicsOutputs& out = adapter->outputs();
            for (const auto& body : out.bodies) {
                trace.insert(trace.end(), body.position.begin(), body.position.end());
                trace.insert(trace.end(), body.orientation.begin(), body.orientation.end());
                trace.insert(trace.end(), body.angular_velocity.begin(), body.angular_velocity.end());
                trace.insert(trace.end(), body.linear_velocity.begin(), body.linear_velocity.end());
            }
            for (const auto& joint : out.joints) {
                trace.insert(trace.end(), joint.generalized_force.begin(), joint.generalized_force.end());
                trace.insert(trace.end(), joint.reaction.force.begin(), joint.reaction.force.end());
            }
            for (const auto& constraint : out.constraints) {
                trace.insert(trace.end(), constraint.force.begin(), constraint.force.end());
                trace.push_back(constraint.translational_error);
            }
            for (const auto& member : out.members) {
                trace.insert(trace.end(), member.start.moment.begin(), member.start.moment.end());
            }
            trace.push_back(adapter->health().max_constraint_drift);
        }
        return trace;
    };
    const std::vector<double> first = run();
    const std::vector<double> second = run();
    REQUIRE(first.size() == second.size());
    CHECK(first == second);
}

TEST_CASE("MuJoCo adapter ends the run on a non-finite state", "[PHY-009]") {
    auto adapter = loaded("pendulum.xml");
    adapter->set_inputs(torque(1.0));
    adapter->step();
    adapter->set_spring_stiffness(0, 1e300);
    bool raised = false;
    for (int n = 0; n < 10 && !raised; ++n) {
        try {
            adapter->step();
        } catch (const AdapterError& error) {
            raised = true;
            CHECK(error.step() >= 1);
        }
    }
    CHECK(raised);
}

TEST_CASE("MuJoCo adapter changes springs mid-run", "[PHY-008]") {
    constexpr double tau = 2.0;
    const double v1 = h * tau / (inertia_pivot + h * damping);
    const double q1 = h * v1;

    SECTION("stiffness takes effect from the next step") {
        constexpr double stiffness = 3.0;
        auto adapter = loaded("pendulum.xml");
        adapter->set_inputs(torque(tau));
        adapter->step();
        adapter->set_spring_stiffness(0, stiffness);
        adapter->step();
        const double v2 =
            (inertia_pivot * v1 - h * (mass * gravity * com_distance * std::sin(q1) + stiffness * q1)) /
            (inertia_pivot + h * damping);
        CHECK(relative(adapter->outputs().bodies[0].angular_velocity[2], v2) < 1e-12);
    }
    SECTION("offset moves the spring's rest position") {
        constexpr double stiffness = 3.0;
        constexpr double offset = 0.1;
        auto adapter = loaded("pendulum.xml");
        adapter->set_spring_stiffness(0, stiffness);
        adapter->set_spring_offset(0, offset);
        adapter->step();
        const double expected = h * stiffness * offset / (inertia_pivot + h * damping);
        CHECK(relative(adapter->outputs().bodies[0].angular_velocity[2], expected) < 1e-12);
    }
    SECTION("member nodes follow the joint springs; others are refused") {
        auto adapter = loaded("mechanism.xml");
        for (std::size_t spring = 0; spring < 5; ++spring) {
            adapter->set_spring_stiffness(spring, 50.0);
            adapter->set_spring_offset(spring, 0.0);
        }
        CHECK_THROWS_AS(adapter->set_spring_stiffness(5, 1.0), AdapterError);
        CHECK_THROWS_AS(adapter->set_spring_offset(5, 1.0), AdapterError);
        CHECK_THROWS_AS(adapter->set_spring_stiffness(0, -1.0), AdapterError);
        CHECK_THROWS_AS(adapter->set_spring_stiffness(0, std::nan("")), AdapterError);
        CHECK_THROWS_AS(adapter->set_spring_offset(0, INFINITY), AdapterError);
    }
}

// The mechanism's arm: rod of the pendulum, horizontal along +x, pinned at the origin and tied at x = 0.5 m.
// Static: the tie carries m g d / 0.5 = 4.905 N and the hinge transmits no torque about its axis.
TEST_CASE("MuJoCo adapter disables a constraint mid-run", "[PHY-008][PHY-003]") {
    auto adapter = loaded("mechanism.xml");
    for (int n = 0; n < 200; ++n) {
        adapter->step();
    }
    const PhysicsOutputs& held = adapter->outputs();
    REQUIRE(held.constraints.size() == 1);
    REQUIRE(held.constraints[0].force.size() == 3);
    const double tie = mass * gravity * com_distance / 0.5;
    CHECK(relative(std::abs(held.constraints[0].force[1]), tie) < 1e-6);
    CHECK(std::abs(held.joints[0].generalized_force[0]) < 1e-9);
    CHECK(held.constraints[0].translational_error < 1e-4);
    CHECK(adapter->health().max_constraint_drift == held.constraints[0].translational_error);

    adapter->disable_constraint(0);
    // Released from rest, the arm turns at alpha = m g d / I; after t = 0.1 s it has turned alpha t^2 / 2 = 0.147
    // rad, so the gravity torque m g d cos(angle) stays above 0.98 of its start: |omega| in [0.98, 1] alpha t.
    constexpr int fall = 100;
    for (int n = 0; n < fall; ++n) {
        adapter->step();
    }
    const double alpha = mass * gravity * com_distance / inertia_pivot;
    const double omega = -adapter->outputs().bodies[0].angular_velocity[2];
    CHECK(omega >= 0.98 * alpha * fall * h);
    CHECK(omega <= alpha * fall * h);
    CHECK(adapter->outputs().constraints[0].force.empty());
    CHECK(adapter->health().max_constraint_drift == 0.0);
    CHECK_THROWS_AS(adapter->disable_constraint(1), AdapterError);
}

// The clamped beam: segment masses rho A s with spans l, l, l, l/2 (l = 0.25 m) from x = l/2. At rest the root carries
// the hanging weight m g = rho A (L - l/2) g up and the moment g rho A sum(s x_c) = 0.4828 N m about +z. Tip sag w L^4
// / (8 E I) = 0.012 m shortens lever arms by O(sag^2 / L) ~ 1e-4, so the moment is checked to 1e-3. Settling: the
// first mode omega1 = 1.875^2 sqrt(E I / (rho A L^4)) = 35 rad/s sees node dampers c = 1 N m s/rad against node
// springs k = E I / l = 40 N m/rad as zeta ~ c omega1 / (2 k) = 0.44, so 3 s is 46 decay times, e^-46 ~ 1e-20.
TEST_CASE("MuJoCo adapter publishes member end-loads", "[PHY-007][PHY-018]") {
    auto adapter = loaded("mechanism.xml");
    for (int n = 0; n < 3000; ++n) {
        adapter->step();
    }
    const PhysicsOutputs& out = adapter->outputs();
    REQUIRE(out.members.size() == 1);
    constexpr double line_mass = 1000.0 * 1e-4;
    const double weight = line_mass * (1.0 - 0.125) * gravity;
    const double moment = line_mass * gravity * (0.25 * (0.25 + 0.5 + 0.75) + 0.125 * 0.9375);
    CHECK(relative(out.members[0].start.force[1], weight) < 1e-6);
    CHECK(std::abs(out.members[0].start.force[0]) < 1e-6 * weight);
    CHECK(relative(out.members[0].start.moment[2], moment) < 1e-3);
    CHECK(out.members[0].end.force == mechsim::kernel::Vec3{0.0, 0.0, 0.0});
}

// Doubling the mass doubles I_com and m d^2 together, so the first step answers (2 I + h D) v1 = h tau.
TEST_CASE("MuJoCo adapter changes payload mass and refuses what it cannot map", "[PHY-008]") {
    auto adapter = loaded("pendulum.xml");
    adapter->set_payload_mass(0, 2.0 * mass);
    adapter->set_inputs(torque(2.0));
    adapter->step();
    const double expected = h * 2.0 / (2.0 * inertia_pivot + h * damping);
    CHECK(relative(adapter->outputs().bodies[0].angular_velocity[2], expected) < 1e-12);
    CHECK_THROWS_AS(adapter->set_payload_mass(1, 1.0), AdapterError);
    CHECK_THROWS_AS(adapter->set_payload_mass(0, 0.0), AdapterError);
    CHECK_THROWS_AS(adapter->set_payload_mass(0, std::nan("")), AdapterError);
    CHECK_THROWS_AS(adapter->release_load(0), AdapterError);
    CHECK_THROWS_AS(adapter->set_payload_attachment(0, 0), AdapterError);
    CHECK_THROWS_AS(adapter->set_payload_attachment(1, 0), AdapterError);
    CHECK_THROWS_AS(adapter->set_payload_attachment(0, 1), AdapterError);
}

// A cube at rest on the floor: the contacts carry its weight, m g on the cube, so -m g z on the world (body_a).
TEST_CASE("MuJoCo adapter publishes contact forces", "[PHY-003]") {
    auto adapter = loaded("box_on_floor.xml");
    for (int n = 0; n < 1000; ++n) {
        adapter->step();
    }
    const PhysicsOutputs& out = adapter->outputs();
    REQUIRE(!out.contacts.empty());
    double total = 0.0;
    for (const auto& contact : out.contacts) {
        CHECK(contact.body_a == mechsim::kernel::kWorldBody);
        CHECK(contact.body_b == 0);
        total += contact.force[2];
    }
    CHECK(relative(total, -1.0 * gravity) < 1e-6);
    CHECK(std::abs(out.bodies[0].linear_velocity[2]) < 1e-6);
}

// Reflected lead screw (PHY-018, ENG-048): rotor J = 5.4e-6 kg m^2 on an 8 mm lead driving a 10 kg carriage, so
// c = lead / 2 pi and the rigid system is (m + J / c^2) a = F with F the shaft torque mapped to the carriage.
namespace {

constexpr double rotor_inertia = 5.4e-6;
constexpr double carriage_mass = 10.0;
const double lead_c = 0.008 / (2.0 * std::numbers::pi);
const double reflected_mass = carriage_mass + rotor_inertia / (lead_c * lead_c);

PhysicsInputs shaft_torque(double value) {
    PhysicsInputs inputs;
    inputs.generalized_forces = {value, 0.0};
    return inputs;
}

}  // namespace

// Closed form a = tau (2 pi / lead) / (m + J (2 pi / lead)^2). Semi-implicit Euler under a constant force gives
// v_n = n h a and x_n = a h^2 n (n + 1) / 2 exactly, so the scheme check is held to 1e-6 (rounding is ~n eps, 1e-12)
// and the continuum x = a t^2 / 2 differs by the scheme's 1 / n, held to 1 / n + 1e-6.
TEST_CASE("MuJoCo lead screw tracks the closed-form rigid drive", "[PHY-018][PHY-033][ENG-048][reference]") {
    constexpr double tau = 0.1;
    const double a = tau / lead_c / reflected_mass;
    auto adapter = loaded("lead_screw.xml");
    constexpr int n = 2000;
    for (int k = 0; k < n; ++k) {
        adapter->set_inputs(shaft_torque(tau));
        adapter->step();
    }
    const PhysicsOutputs& out = adapter->outputs();
    const double t = n * h;
    const double x = out.bodies[1].position[0];
    const double scheme = a * h * h * n * (n + 1) / 2.0;
    std::cout << "lead screw: closed-form x(2 s) " << a * t * t / 2.0 << " m, scheme " << scheme << " m, measured "
              << x << " m\n";
    CHECK(relative(x, scheme) < 1e-6);
    CHECK(relative(x, a * t * t / 2.0) < 1.0 / n + 1e-6);
    CHECK(relative(out.bodies[1].linear_velocity[0], a * t) < 1e-6);
    // The shaft reports q_driven / c: its spin about z is v / c.
    CHECK(relative(out.bodies[0].angular_velocity[2], out.bodies[1].linear_velocity[0] / lead_c) < 1e-12);
    // Load torque: tau - J a / c = tau m / (m + J / c^2) = m a c, the torque that accelerates the carriage alone.
    CHECK(relative(out.joints[0].generalized_force.at(0), carriage_mass * a * lead_c) < 1e-9);
}

// Efficiency by power direction: one step from rest is driving, F = eta tau / c; a reversed torque against the
// motion is back-driven, F = tau / (eta c). Each step changes v by h F / (m + J / c^2) exactly.
TEST_CASE("MuJoCo lead screw applies efficiency by power direction", "[PHY-018][PHY-033][ENG-048]") {
    constexpr double eta = 0.8;
    constexpr double tau = 0.1;
    auto adapter = loaded("lead_screw_lossy.xml");
    adapter->set_inputs(shaft_torque(tau));
    adapter->step();
    const double v1 = adapter->outputs().bodies[1].linear_velocity[0];
    CHECK(relative(v1, h * eta * tau / lead_c / reflected_mass) < 1e-9);
    adapter->set_inputs(shaft_torque(-0.01));
    adapter->step();
    const double v2 = adapter->outputs().bodies[1].linear_velocity[0];
    CHECK(relative(v2 - v1, h * (-0.01) / (eta * lead_c) / reflected_mass) < 1e-9);
    adapter->step();
    CHECK(adapter->outputs().bodies[1].linear_velocity[0] == v2);
    CHECK(adapter->outputs().joints[0].generalized_force.at(0) == 0.0);
}

TEST_CASE("MuJoCo reflected shaft refuses body-level changes", "[PHY-008]") {
    auto adapter = loaded("lead_screw.xml");
    PhysicsInputs wrench;
    wrench.body_wrenches.push_back({0, {}});
    CHECK_THROWS_AS(adapter->set_inputs(wrench), AdapterError);
    CHECK_THROWS_AS(adapter->set_spring_stiffness(0, 1.0), AdapterError);
    CHECK_THROWS_AS(adapter->set_payload_mass(0, 1.0), AdapterError);
    adapter->set_payload_mass(1, 20.0);
}
