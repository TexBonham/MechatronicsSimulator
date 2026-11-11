#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
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
#include "mechsim/signals/safety.hpp"
#include "mechsim/signals/signal_component.hpp"
#include "mechsim/signals/stepper.hpp"

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
TEST_CASE("MuJoCo lead screw tracks the closed-form rigid drive", "[PHY-018][PHY-033][ENG-048]") {
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

// Lifting a 10 kg carriage on an 8 mm lead screw (eta 0.8) at constant speed, gravity m g along -x as a body force.
// Rigid and steady: the driving thrust eta tau / c balances m g exactly, so a = 0 and the load torque is the closed
// form m g c / eta = m g lead / (2 pi eta), positive along the shaft's + axis (PHY-033). Every term is one or two
// roundings from the closed form, held to 1e-9. The stepper, stepping at a constant rate, demands J * 0 + load.
TEST_CASE("MuJoCo lead screw lifting a load publishes the positive closed-form load torque the stepper demands",
          "[PHY-033][ENG-048][SIG-033]") {
    constexpr double eta = 0.8;
    const double weight = carriage_mass * gravity;
    const double closed_form = weight * 0.008 / (2.0 * std::numbers::pi * eta);
    auto adapter = loaded("lead_screw_lossy.xml");
    for (int k = 0; k < 50; ++k) {
        adapter->set_inputs(shaft_torque(0.1));
        adapter->step();
    }
    REQUIRE(adapter->outputs().bodies[1].linear_velocity[0] > 0.0);
    PhysicsInputs lift = shaft_torque(closed_form);
    lift.body_wrenches.push_back({1, {{0.0, 0.0, 0.0}, {-weight, 0.0, 0.0}}});
    for (int k = 0; k < 20; ++k) {
        adapter->set_inputs(lift);
        adapter->step();
    }
    const mechsim::kernel::TransmissionState& state = adapter->outputs().transmissions.at(0);
    std::cout << "lead screw lift: closed-form load torque " << closed_form << " N m, measured "
              << state.shaft_load_torque << " N m\n";
    CHECK(state.shaft_load_torque > 0.0);
    CHECK(relative(state.shaft_load_torque, closed_form) < 1e-9);
    CHECK(relative(state.force, weight) < 1e-9);
    CHECK(state.power_flow == mechsim::kernel::PowerFlow::Forward);
    CHECK(state.stretch == 0.0);
    CHECK(adapter->outputs().joints[0].generalized_force.at(0) == state.shaft_load_torque);

}

TEST_CASE("A stepper holds the lift through the MuJoCo drive within 1 mm over 2 s", "[SIG-033][PHY-034][reference]") {
    // The behavioral stepper on lift_screw.xml (10 kg, 8 mm lead, J = 5.4e-6 kg m^2, h = 1 ms), zero steps commanded:
    // its drive K_hold = pi 0.40 / (2 pi / 100) = 20 N m/rad holds tau_load = m g c = 0.1249 N m < 0.40 N m with the
    // shaft sag tau_load / K_hold = 6.245e-3 rad, c times that = 7.95 um of lift; released from rest the swing about
    // the sag never exceeds sag / sqrt(1 - s0 h^2 / 4) (the explicit loop at s0 h^2 = 0.925, the lift test above).
    mechsim::signals::BehavioralStepperParams params;
    params.step_angle = std::numbers::pi / 100.0;
    params.rotor_inertia = rotor_inertia;
    params.holding_torque = 0.40;
    params.torque_speed = {{0.0, 0.40}, {31.4, 0.36}};
    mechsim::signals::BehavioralStepper motor("lift_motor", params);
    motor.start(mechsim::signals::ComponentClock{Rate(1000), mechsim::kernel::Duration(1), 1});
    auto adapter = loaded("lift_screw.xml");
    const std::vector<double> hold{0.0, 1.0, 1.0, 24.0};
    const double c = 0.008 / (2.0 * std::numbers::pi);
    const double sag = carriage_mass * gravity * c / 20.0;
    double missed = 0.0;
    double lowest = 0.0;
    for (int k = 0; k < 2000; ++k) {
        motor.begin_tick(mechsim::kernel::SimTime(k));
        motor.sample(adapter->outputs());
        motor.set_inputs(hold);
        motor.latch_outputs();
        missed += motor.output_values()[2];
        PhysicsInputs inputs;
        inputs.joint_drives.assign(motor.joint_drives().begin(), motor.joint_drives().end());
        adapter->set_inputs(inputs);
        adapter->step();
        lowest = std::min(lowest, adapter->outputs().joint_positions[1]);
    }
    const double lift = adapter->outputs().joint_positions[1];
    std::cout << "stepper lift hold: closed-form sag " << -sag * c << " m, measured " << lift << " m, lowest " << lowest
              << " m\n";
    CHECK(missed == 0.0);
    CHECK(lowest > -0.001);
    CHECK(relative(lift, -sag * c) < 0.1);
    const double s0h2 = (20.0 / (c * c)) * h * h / reflected_mass;
    CHECK(lowest >= -(1.0 + (1.0 / std::sqrt(1.0 - (s0h2 / 4.0)))) * sag * c * (1.0 + 1e-6));
}

// Gear pair, ratio +4 and -4 (c = 1 / ratio = +-0.25): the reflection is symmetric under q_driven -> -q_driven with
// c -> -c, so the wheel's angle, rate and the transmission force flip sign while the shaft's rate and load torque,
// both about the shaft's + axis, are unchanged. Every quantity is a product of negated exact factors: held to 1e-12.
TEST_CASE("MuJoCo negative gear ratio flips the driven side and keeps the shaft side", "[PHY-018][PHY-033]") {
    struct Result {
        double wheel_rate = 0.0;
        double wheel_quat_z = 0.0;
        double force = 0.0;
        double load = 0.0;
        double shaft_rate = 0.0;
    };
    auto run = [](const std::string& name) {
        auto adapter = loaded(name);
        for (int k = 0; k < 100; ++k) {
            adapter->set_inputs(shaft_torque(0.01));
            adapter->step();
        }
        const PhysicsOutputs& out = adapter->outputs();
        return Result{out.bodies[1].angular_velocity[2], out.bodies[1].orientation[3], out.transmissions.at(0).force,
                      out.transmissions.at(0).shaft_load_torque, out.bodies[0].angular_velocity[2]};
    };
    const Result forward = run("gear_pair.xml");
    const Result reversed = run("gear_pair_reversed.xml");
    REQUIRE(forward.wheel_rate > 0.0);
    REQUIRE(forward.force > 0.0);
    REQUIRE(forward.load > 0.0);
    CHECK(relative(reversed.wheel_rate, -forward.wheel_rate) < 1e-12);
    CHECK(relative(reversed.wheel_quat_z, -forward.wheel_quat_z) < 1e-12);
    CHECK(relative(reversed.force, -forward.force) < 1e-12);
    CHECK(relative(reversed.load, forward.load) < 1e-12);
    CHECK(relative(reversed.shaft_rate, forward.shaft_rate) < 1e-12);
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

namespace {

using mechsim::kernel::JointDrive;

PhysicsInputs driven(const JointDrive& drive, double force = 0.0) {
    PhysicsInputs inputs;
    if (force != 0.0) {
        inputs.generalized_forces = {force};
    }
    inputs.joint_drives = {drive};
    return inputs;
}

struct DriveRun {
    std::vector<double> x;       // joint coordinate after each step
    std::vector<double> torque;  // the drive torque published for each step
};

// The 1 kg slider (slider.xml, no gravity) under a constant force F on its drivable joint and a drive toward 0.
DriveRun run_slider(double stiffness, double drive_damping, double force, int steps, bool enabled = true) {
    auto adapter = loaded("slider.xml");
    DriveRun run;
    for (int k = 0; k < steps; ++k) {
        adapter->set_inputs(driven(JointDrive{0, 0.0, stiffness, drive_damping, enabled}, force));
        adapter->step();
        const PhysicsOutputs& out = adapter->outputs();
        REQUIRE(out.joint_positions.size() == 1);
        REQUIRE(out.joint_drive_torques.size() == 1);
        CHECK(out.joint_drive_torques[0].joint == 0);
        CHECK(out.joint_positions[0] == out.bodies[0].position[0]);
        run.x.push_back(out.joint_positions[0]);
        run.torque.push_back(out.joint_drive_torques[0].torque);
    }
    return run;
}

double max_deviation(const std::vector<double>& values, double about, std::size_t from, std::size_t to) {
    double worst = 0.0;
    for (std::size_t i = from; i < to; ++i) {
        worst = std::max(worst, std::abs(values[i] - about));
    }
    return worst;
}

}  // namespace

TEST_CASE("MuJoCo drive holds a load with the static sag F / K, stable past the explicit limit", "[PHY-034][reference]") {
    // m = 1 kg, F = 10 N, h = 1 ms. The drive is backward Euler (PHY-034): (m + D h + K h^2) a = F - D v - K (x + v h)
    // toward rest 0; at rest (a = 0, v = 0) x = F / K for every K: 1e-5 m at K = 1e6 N/m (omega h = 1) and 1e-7 m at
    // K = 1e8 N/m (omega h = 10, where an explicit spring would diverge: K h^2 / m = 100 > 4). Damping 1e3 and
    // 1e5 N s/m (zeta 0.5 and 5) settles it to rounding by 2 s. Without damping backward Euler never adds energy:
    // m v^2 / 2 + K (x - F / K)^2 / 2 starts at K (F / K)^2 / 2 and shrinks by the factor 1 / (1 + K h^2 / m) per step
    // or better, so the swing about F / K never exceeds F / K and is below 2^-500 of it after 1000 steps (rounding).
    struct Case {
        double stiffness;
        double drive_damping;
    };
    for (const Case c : {Case{1e6, 1e3}, Case{1e8, 1e5}}) {
        const double sag = 10.0 / c.stiffness;
        const DriveRun damped = run_slider(c.stiffness, c.drive_damping, 10.0, 2000);
        std::cout << "drive K " << c.stiffness << " N/m: closed-form sag " << sag << " m, measured " << damped.x.back()
                  << " m, drive force " << damped.torque.back() << " N (closed form -10)\n";
        CHECK(relative(-damped.x.back(), -sag) < 0.1);
        CHECK(std::abs(damped.torque.back() + 10.0) < 1e-3);
        const DriveRun free = run_slider(c.stiffness, 0.0, 10.0, 2000);
        const double early = max_deviation(free.x, sag, 0, 1000);
        const double late = max_deviation(free.x, sag, 1000, 2000);
        std::cout << "undamped swing about the sag: early " << early << " m, late " << late << " m, bound " << sag
                  << " m\n";
        CHECK(early <= sag * (1.0 + 1e-9));
        CHECK(late <= sag * 1e-9);
    }
}

TEST_CASE("MuJoCo drive on a reflected lead screw holds the lift's gravity load at tau_load / K", "[PHY-034][reference]") {
    // lift_screw.xml: J = 5.4e-6 kg m^2, 10 kg, lead 8 mm, c = 0.008 / (2 pi), gravity along the rail. Hold stiffness
    // of the gantry lift: K = pi tau_hold / (2 step) = 0.40 * 50 = 20 N m/rad, D = 1e-4 N m s/rad. Load torque
    // tau_load = m g c = 0.12490 N m, so the shaft sags theta = -tau_load / K = -6.245e-3 rad and the drive supplies
    // +tau_load, the PHY-033 load torque at rest. K_d A0 h^2 = (K / c^2) h^2 / (m + J / c^2) = 0.925 < 1: d stays at its
    // floor and the loop is the explicit spring, zeta = D_d / (2 sqrt(K_d m_eff)) = 2.4e-3, so the swing about the sag
    // decays to e^(-zeta omega 2 s) = e^(-4.6) = 1 percent by 2 s and never exceeds sag / sqrt(1 - 0.925 / 4).
    const double c = 0.008 / (2.0 * std::numbers::pi);
    const double stiffness = 20.0;
    const double tau_load = 10.0 * gravity * c;
    const double sag = -tau_load / stiffness;
    auto adapter = loaded("lift_screw.xml");
    double worst = 0.0;
    for (int k = 0; k < 2000; ++k) {
        adapter->set_inputs(driven(JointDrive{0, 0.0, stiffness, 1e-4, true}));
        adapter->step();
        worst = std::max(worst, std::abs(adapter->outputs().joint_positions[0] - sag));
    }
    const PhysicsOutputs& out = adapter->outputs();
    const double theta = out.joint_positions[0];
    const double s0h2 = (stiffness / (c * c)) * h * h / (10.0 + 5.4e-6 / (c * c));
    std::cout << "lift drive: closed-form sag " << sag << " rad, measured " << theta << " rad; drive torque "
              << out.joint_drive_torques[0].torque << " N m, load torque " << out.transmissions[0].shaft_load_torque
              << " N m (closed form " << tau_load << ")\n";
    CHECK(relative(theta, sag) < 0.1);
    CHECK(theta * c > -0.001);
    CHECK(worst <= std::abs(sag) / std::sqrt(1.0 - s0h2 / 4.0) * (1.0 + 1e-6));
    CHECK(relative(out.joint_drive_torques[0].torque, tau_load) < 0.1);
    CHECK(relative(out.transmissions[0].shaft_load_torque, tau_load) < 0.1);
    CHECK(out.joints[0].generalized_force[0] == out.transmissions[0].shaft_load_torque);
    CHECK(out.joint_positions[1] == out.bodies[1].position[2]);
}

TEST_CASE("MuJoCo drive tracks a rest moving at constant rate with the lag D w / K", "[PHY-034][reference]") {
    // m x'' + D x' + K (x - w t) = 0 has the steady solution x = w t - D w / K: the first-order lag of the
    // spring-damper. With the drive at the step's end state (backward Euler, PHY-034), the discrete steady state
    // v = w, a = 0 gives D w + K (x_k - rest_k) = 0: the same lag behind the rest of the step just taken. K = 1e6 N/m,
    // D = 1e3 N s/m (zeta 0.5), w = 0.1 m/s: lag 1e-4 m; the transient is e^(-1000) by 2 s.
    auto adapter = loaded("slider.xml");
    const double rate = 0.1;
    const double lag = 1e3 * rate / 1e6;
    for (int k = 0; k < 2000; ++k) {
        adapter->set_inputs(driven(JointDrive{0, rate * h * k, 1e6, 1e3, true}));
        adapter->step();
    }
    const double measured = (rate * h * 1999.0) - adapter->outputs().joint_positions[0];
    std::cout << "tracking lag: closed form " << lag << " m, measured " << measured << " m\n";
    CHECK(relative(measured, lag) < 0.01);
}

TEST_CASE("MuJoCo disabled or zero drive applies no force", "[PHY-034]") {
    // Free motion of the 1 kg slider under F = 10 N, semi-implicit Euler: x_n = (F / m) h^2 n (n + 1) / 2.
    for (const bool enabled : {false, true}) {
        double stiffness = 1e6;
        if (enabled) {
            stiffness = 0.0;
        }
        const DriveRun run = run_slider(stiffness, 0.0, 10.0, 100, enabled);
        const double closed = 10.0 * h * h * 100.0 * 101.0 / 2.0;
        CHECK(relative(run.x.back(), closed) < 1e-9);
        CHECK(run.torque.back() == 0.0);
    }
}

TEST_CASE("MuJoCo drive applies for its step only and publishes nothing without one", "[PHY-034][PHY-005]") {
    auto adapter = loaded("slider.xml");
    adapter->set_inputs(driven(JointDrive{0, 0.0, 1e6, 0.0, true}, 10.0));
    adapter->step();
    CHECK(adapter->outputs().joint_drive_torques.size() == 1);
    adapter->set_inputs(torque(10.0));
    adapter->step();
    CHECK(adapter->outputs().joint_drive_torques.empty());
    // Without the drive the step is free: v gains F / m h = 0.01 m/s on top of what it had.
    const double before = adapter->outputs().bodies[0].linear_velocity[0];
    adapter->set_inputs(torque(10.0));
    adapter->step();
    CHECK(std::abs(adapter->outputs().bodies[0].linear_velocity[0] - before - 0.01) < 1e-12);
}

TEST_CASE("MuJoCo adapter refuses drives it cannot apply", "[PHY-034][PHY-009]") {
    auto screw = loaded("lead_screw.xml");
    CHECK_NOTHROW(screw->set_inputs(driven(JointDrive{0, 0.0, 1.0, 0.0, true})));
    CHECK_THROWS_AS(screw->set_inputs(driven(JointDrive{1, 0.0, 1.0, 0.0, true})), AdapterError);
    CHECK_THROWS_AS(screw->set_inputs(driven(JointDrive{2, 0.0, 1.0, 0.0, true})), AdapterError);
    CHECK_THROWS_AS(screw->set_inputs(driven(JointDrive{0, 0.0, -1.0, 0.0, true})), AdapterError);
    CHECK_THROWS_AS(screw->set_inputs(driven(JointDrive{0, 0.0, 1.0, -1.0, true})), AdapterError);
    CHECK_THROWS_AS(screw->set_inputs(driven(JointDrive{0, std::nan(""), 1.0, 0.0, true})), AdapterError);
    PhysicsInputs twice = driven(JointDrive{0, 0.0, 1.0, 0.0, true});
    twice.joint_drives.push_back(JointDrive{0, 0.1, 1.0, 0.0, false});
    CHECK_THROWS_AS(screw->set_inputs(twice), AdapterError);
    // The envelope is Z = (K_d h^2 + D_d h) A0 <= 400 (past it the clamp breaks backward Euler and from Z of about 535
    // the semi-implicit map with the shifted rest diverges). Slider, 1 kg, A0 = 1 / m, h = 1 ms: K = 4e8 N/m is
    // Z = 400 exactly; K = 3.99e8 (Z = 399) runs and K = 4.01e8 (Z = 401) throws. Damping counts: K = 3.9e8 with
    // D = 1e5 N s/m is Z = 390 + 100 = 490 and throws, as does K = 1e10.
    auto below = loaded("slider.xml");
    below->set_inputs(driven(JointDrive{0, 0.0, 3.99e8, 0.0, true}));
    CHECK_NOTHROW(below->step());
    auto above = loaded("slider.xml");
    above->set_inputs(driven(JointDrive{0, 0.0, 4.01e8, 0.0, true}));
    CHECK_THROWS_AS(above->step(), AdapterError);
    auto damped = loaded("slider.xml");
    damped->set_inputs(driven(JointDrive{0, 0.0, 3.9e8, 1e5, true}));
    CHECK_THROWS_AS(damped->step(), AdapterError);
    auto slider = loaded("slider.xml");
    slider->set_inputs(driven(JointDrive{0, 0.0, 1e10, 0.0, true}));
    CHECK_THROWS_AS(slider->step(), AdapterError);
    std::string text = fixture("slider.xml");
    replace(text, R"(<text name="mechsim:drives" data="slide" />)", R"(<text name="mechsim:drives" data="rail" />)");
    MujocoAdapter unknown;
    CHECK_THROWS_AS(unknown.load(PreparedModel{"", text, "0.1.0"}, Rate(1000)), AdapterError);
}

// ===== Faults: slip, backlash, payload (PHY-006) ===========================

namespace {

// lead_screw.xml under a shaft drive: K = 5 N m/rad and critical damping D = 2 sqrt(K J_eff) = 0.0208 N m s/rad with
// the shaft-side inertia J_eff = m c^2 + J = 2.16e-5 kg m^2. With A0 at most 1 / m (armature in it or not),
// K_d A0 h^2 <= (K / c^2) h^2 / m = 0.31 < 1 keeps the drive at its impedance floor (exact dynamics, PHY-034) and the
// envelope s0 h^2 + 2 s1 h <= 0.31 + 2 (D / c^2) h / m = 2.88 < 4. The loop frequency sqrt(K / J_eff) = 481 rad/s
// makes every transient (1 + 481 t) e^(-481 t): below 1e-200 after 1 s, so states 1 s apart are steady to rounding.
constexpr double hold_stiffness = 5.0;

double hold_damping() {
    const double shaft_inertia = (carriage_mass * lead_c * lead_c) + rotor_inertia;
    return 2.0 * std::sqrt(hold_stiffness * shaft_inertia);
}

void hold(MujocoAdapter& adapter, double rest, int steps) {
    for (int k = 0; k < steps; ++k) {
        adapter.set_inputs(driven(JointDrive{0, rest, hold_stiffness, hold_damping(), true}));
        adapter.step();
    }
}

}  // namespace

TEST_CASE("MuJoCo slip loses (1 - s) of the commanded shaft travel and reports it", "[PHY-006][TST-015][reference]") {
    // The rest advances w h per step (w = 10 rad/s) for N = 2000 steps, rest_k = w h k; slip s = 0.3 from the start.
    // The first rest only starts tracking, so N - 1 increments follow: slip = s w h (N - 1) exactly (a sum of equal
    // increments). The driven joint's rest advances (1 - s) w h per step; at steady state its coordinate keeps a
    // constant lag behind that rest, so over the last 1 s (1000 steps, T = 1 s) the carriage advances c (1 - s) w T
    // and the shaft, reported as q_driven / c + slip, advances w T: each to rounding, held to 1e-9 relative. The
    // shaft rate is qd_driven / c + s w = (1 - s) w + s w = w. Without load (horizontal rail, no joint damping) the
    // steady drive force is zero: what the spring supplies (K times the lag) the damper takes back, so the published
    // drive torque stays within K times the static lag D (1 - s) w / K.
    constexpr double rate = 10.0;
    constexpr double s = 0.3;
    constexpr int n = 2000;
    auto adapter = loaded("lead_screw.xml");
    adapter->set_transmission_slip(0, s);
    double shaft_mid = 0.0;
    double carriage_mid = 0.0;
    double worst_torque = 0.0;
    for (int k = 0; k < n; ++k) {
        adapter->set_inputs(driven(JointDrive{0, rate * h * k, hold_stiffness, hold_damping(), true}));
        adapter->step();
        if (k == n / 2 - 1) {
            shaft_mid = adapter->outputs().joint_positions[0];
            carriage_mid = adapter->outputs().joint_positions[1];
        }
        if (k >= n / 2) {
            worst_torque = std::max(worst_torque, std::abs(adapter->outputs().joint_drive_torques[0].torque));
        }
    }
    const PhysicsOutputs& out = adapter->outputs();
    const int half = n / 2;
    const double window = rate * h * half;
    const double slip = s * rate * h * (n - 1);
    std::cout << "slip: closed-form slip " << slip << " rad, measured " << out.transmissions[0].slip
              << " rad; carriage advance over 1 s closed form " << lead_c * (1.0 - s) * window << " m, measured "
              << out.joint_positions[1] - carriage_mid << " m\n";
    CHECK(out.transmissions[0].slipping);
    CHECK(relative(out.transmissions[0].slip, slip) < 1e-9);
    CHECK(relative(out.joint_positions[1] - carriage_mid, lead_c * (1.0 - s) * window) < 1e-9);
    CHECK(relative(out.joint_positions[0] - shaft_mid, window) < 1e-9);
    CHECK(relative(out.bodies[0].angular_velocity[2], rate) < 1e-9);
    CHECK(relative(out.bodies[1].linear_velocity[0], lead_c * (1.0 - s) * rate) < 1e-9);
    CHECK(worst_torque <= hold_damping() * (1.0 - s) * rate);

    // Cleared, the lost travel stays lost and the driven joint follows the full increments again.
    adapter->set_transmission_slip(0, 0.0);
    const double carriage_cleared = out.joint_positions[1];
    for (int k = n; k < n + 1000; ++k) {
        adapter->set_inputs(driven(JointDrive{0, rate * h * k, hold_stiffness, hold_damping(), true}));
        adapter->step();
    }
    const PhysicsOutputs& after = adapter->outputs();
    CHECK_FALSE(after.transmissions[0].slipping);
    CHECK(after.transmissions[0].slip == out.transmissions[0].slip);
    CHECK(relative(after.bodies[1].linear_velocity[0], lead_c * rate) < 1e-9);
    CHECK(after.joint_positions[1] - carriage_cleared > lead_c * (1.0 - s) * rate * h * 1000.0);
}

TEST_CASE("MuJoCo slip scales an explicit shaft torque by (1 - s)", "[PHY-006][TST-015]") {
    // One step from rest is driving: F = (1 - s) eta tau / c on the reflected mass, v = h F / (m + J / c^2) exactly.
    constexpr double tau = 0.1;
    constexpr double s = 0.5;
    auto adapter = loaded("lead_screw.xml");
    adapter->set_transmission_slip(0, s);
    adapter->set_inputs(shaft_torque(tau));
    adapter->step();
    const double v = h * (1.0 - s) * tau / lead_c / reflected_mass;
    CHECK(relative(adapter->outputs().bodies[1].linear_velocity[0], v) < 1e-12);
    CHECK(adapter->outputs().transmissions[0].slip == 0.0);
}

TEST_CASE("MuJoCo slip refuses unknown transmissions and fractions outside [0, 1]", "[PHY-006][TST-015][PHY-009]") {
    auto adapter = loaded("lead_screw.xml");
    CHECK_NOTHROW(adapter->set_transmission_slip(0, 0.0));
    CHECK_NOTHROW(adapter->set_transmission_slip(0, 1.0));
    CHECK_THROWS_AS(adapter->set_transmission_slip(1, 0.1), AdapterError);
    CHECK_THROWS_AS(adapter->set_transmission_slip(0, -0.1), AdapterError);
    CHECK_THROWS_AS(adapter->set_transmission_slip(0, 1.1), AdapterError);
    CHECK_THROWS_AS(adapter->set_transmission_slip(0, std::nan("")), AdapterError);
    CHECK_THROWS_AS(loaded("pendulum.xml")->set_transmission_slip(0, 0.1), AdapterError);
    MujocoAdapter unloaded;
    CHECK_THROWS_AS(unloaded.set_transmission_slip(0, 0.1), AdapterError);
    mechsim::kernel::NullPhysicsAdapter null;
    CHECK_THROWS_AS(null.set_transmission_slip(0, 0.1), AdapterError);
}

TEST_CASE("MuJoCo backlash: a reversal inside the band moves nothing, past it moves c x", "[PHY-006][reference]") {
    // lead_screw_backlash.xml: 0.4 mm at the carriage, b = 0.4e-3 / c = 0.1 pi rad on the shaft. The play starts
    // centred on the first rest (0). Rest to 1 rad: the input leaves the band, the effective rest stops at 1 - b / 2,
    // and with no load the carriage settles exactly on it, x = c (1 - b / 2). Reversing by b / 2 < b keeps the input
    // inside the band: nothing moves. Reversing by b + x in all (x = 0.2 rad) drags the effective rest to the other
    // edge, 1 - (b + x) + b / 2, a move of -x: the carriage moves -c x. The shaft reports the commanded rest at rest.
    const double band = 0.0004 / lead_c;
    constexpr double x = 0.2;
    auto adapter = loaded("lead_screw_backlash.xml");
    hold(*adapter, 0.0, 1);
    hold(*adapter, 1.0, 1000);
    const double forward = adapter->outputs().joint_positions[1];
    std::cout << "backlash: closed-form carriage " << lead_c * (1.0 - band / 2.0) << " m, measured " << forward
              << " m\n";
    CHECK(relative(forward, lead_c * (1.0 - band / 2.0)) < 1e-9);
    CHECK(relative(adapter->outputs().joint_positions[0], 1.0) < 1e-9);

    hold(*adapter, 1.0 - (band / 2.0), 1000);
    CHECK(std::abs(adapter->outputs().joint_positions[1] - forward) <= 1e-9 * lead_c * band);
    CHECK(relative(adapter->outputs().joint_positions[0], 1.0 - (band / 2.0)) < 1e-9);

    hold(*adapter, 1.0 - band - x, 1000);
    const double moved = adapter->outputs().joint_positions[1] - forward;
    std::cout << "backlash: closed-form move " << -lead_c * x << " m, measured " << moved << " m\n";
    CHECK(relative(moved, -lead_c * x) < 1e-9);
    CHECK(relative(adapter->outputs().joint_positions[0], 1.0 - band - x) < 1e-9);
}

TEST_CASE("MuJoCo zero backlash and zero slip leave the drive rest exact", "[PHY-006][PHY-034]") {
    // Same commands on lead_screw.xml with slip set to 0 explicitly and left unset: bitwise identical states.
    auto plain = loaded("lead_screw.xml");
    auto cleared = loaded("lead_screw.xml");
    cleared->set_transmission_slip(0, 0.0);
    for (int k = 0; k < 200; ++k) {
        for (MujocoAdapter* adapter : {plain.get(), cleared.get()}) {
            adapter->set_inputs(driven(JointDrive{0, 0.01 * k, hold_stiffness, hold_damping(), true}));
            adapter->step();
        }
    }
    CHECK(plain->outputs().joint_positions == cleared->outputs().joint_positions);
    CHECK(plain->outputs().transmissions[0].slip == 0.0);
}

namespace {

constexpr double payload_mass = 2.0;
constexpr std::size_t payload_body = 1;
constexpr std::size_t hook_body = 0;

double payload_height(const MujocoAdapter& adapter) {
    return adapter.outputs().bodies[payload_body].position[2];
}

void coast(MujocoAdapter& adapter, int steps) {
    for (int k = 0; k < steps; ++k) {
        adapter.step();
    }
}

}  // namespace

TEST_CASE("MuJoCo payload weld carries m g and steps with a mass change", "[PHY-006][ENG-035][TST-020][reference]") {
    // payload.xml: 2 kg on a free joint welded to a fixed hook, gravity 9.81. The weld (solref 2 h, 1; MuJoCo's
    // default equality solimp 0.9) at rest: v = 0, so a_ref = -k r - b v = 0 at the first step and the solved row
    // force is f = (a_ref - a0) / (A + R) with a0 = -g, A = 1 / m (com at the body origin, the hook has no DOF) and
    // R = (1 - d) / d A at d = 0.9: f = d m g after one step. At equilibrium (a = 0, v = 0, the weld's critically
    // damped transient e^(-t / 2h) gone after 200 steps) Newton gives f = m g exactly.
    auto adapter = loaded("payload.xml");
    REQUIRE(adapter->outputs().payloads.size() == 1);
    adapter->step();
    const mechsim::kernel::PayloadState& first = adapter->outputs().payloads[0];
    CHECK(first.body == payload_body);
    CHECK(first.carrier == hook_body);
    CHECK(first.attached);
    std::cout << "payload weld after one step: closed form d m g " << 0.9 * payload_mass * gravity << " N, measured "
              << first.force[2] << " N\n";
    CHECK(relative(first.force[2], 0.9 * payload_mass * gravity) < 1e-6);
    coast(*adapter, 199);
    const mechsim::kernel::PayloadState& held = adapter->outputs().payloads[0];
    std::cout << "payload weld at rest: closed form m g " << payload_mass * gravity << " N, measured " << held.force[2]
              << " N\n";
    CHECK(relative(held.force[2], payload_mass * gravity) < 1e-6);
    CHECK(std::abs(held.force[0]) < 1e-9);
    CHECK(std::abs(held.force[1]) < 1e-9);

    adapter->set_payload_mass(payload_body, 3.0);
    coast(*adapter, 200);
    CHECK(relative(adapter->outputs().payloads[0].force[2], 3.0 * gravity) < 1e-6);
}

TEST_CASE("MuJoCo released payload falls freely and re-attaches at its carrier", "[PHY-006][ENG-035][TST-020][reference]") {
    // Released at rest, the free body under gravity alone follows semi-implicit Euler exactly: after n steps it has
    // fallen g h^2 n (n + 1) / 2 (held to 1e-6, rounding), which at t = n h = 0.2 s is g t^2 / 2 = 0.19620 m times
    // (1 + 1 / n), within the brief's 1 percent. Re-attached where it hangs (no step in between), the weld finds the
    // state it left: the payload stays within 1e-6 m. Re-attached after a short fall, the weld pulls it back to the
    // same equilibrium (the translated relative pose with its static sag).
    auto adapter = loaded("payload.xml");
    coast(*adapter, 200);
    const double hanging = payload_height(*adapter);

    adapter->set_payload_attachment(payload_body, mechsim::kernel::kWorldBody);
    adapter->set_payload_attachment(payload_body, hook_body);
    coast(*adapter, 500);
    CHECK(std::abs(payload_height(*adapter) - hanging) < 1e-6);

    adapter->release_load(payload_body);
    constexpr int n = 200;
    coast(*adapter, n);
    const double fallen = hanging - payload_height(*adapter);
    const double t = n * h;
    std::cout << "released payload: closed form g t^2 / 2 " << gravity * t * t / 2.0 << " m, scheme "
              << gravity * h * h * n * (n + 1) / 2.0 << " m, measured " << fallen << " m\n";
    CHECK(relative(fallen, gravity * h * h * n * (n + 1) / 2.0) < 1e-6);
    CHECK(relative(fallen, gravity * t * t / 2.0) < 0.01);
    const mechsim::kernel::PayloadState& released = adapter->outputs().payloads[0];
    CHECK_FALSE(released.attached);
    CHECK(released.carrier == mechsim::kernel::kWorldBody);
    CHECK(released.force == mechsim::kernel::Vec3{0.0, 0.0, 0.0});

    auto again = loaded("payload.xml");
    coast(*again, 200);
    again->release_load(payload_body);
    coast(*again, 20);
    again->set_payload_attachment(payload_body, hook_body);
    coast(*again, 1000);
    CHECK(std::abs(payload_height(*again) - hanging) < 1e-6);
    CHECK(again->outputs().payloads[0].attached);
}

TEST_CASE("MuJoCo payload operations refuse what the translation did not weld", "[PHY-006][ENG-035][TST-020][PHY-009]") {
    auto adapter = loaded("payload.xml");
    CHECK_THROWS_AS(adapter->release_load(hook_body), AdapterError);
    CHECK_THROWS_AS(adapter->release_load(2), AdapterError);
    CHECK_THROWS_AS(adapter->set_payload_attachment(hook_body, mechsim::kernel::kWorldBody), AdapterError);
    CHECK_THROWS_AS(adapter->set_payload_attachment(payload_body, 2), AdapterError);
    CHECK_THROWS_AS(adapter->set_payload_attachment(payload_body, payload_body), AdapterError);
    std::string text = fixture("payload.xml");
    replace(text, R"(data="load:hook")", R"(data="load:crane")");
    MujocoAdapter unknown;
    CHECK_THROWS_AS(unknown.load(PreparedModel{"", text, "0.1.0"}, Rate(1000)), AdapterError);
}

TEST_CASE("MuJoCo slip scales the explicit torque path only; the drive force reaches the joint whole",
          "[PHY-006][TST-015][PHY-033]") {
    // Lead screw at rest, eta = 1, s = 0.5, explicit tau = 0.1 N m plus a drive toward rest 0.01 rad (no damping).
    // Applied on the driven DOF: (1 - s) tau / c + F_d, with F_d the drive's equality force (published as F_d c).
    // The carriage alone takes m a = (1 - s) tau / c + F_d - (J / c^2) a, and the published force is that applied force
    // less the rotor's share: gain (tau_load - s tau) / c with tau_load = tau + F_d c - J a / c. One step from rest
    // (implicitfast, no damping): a = v / h, so the published force must equal m v / h to rounding.
    constexpr double tau = 0.1;
    constexpr double s = 0.5;
    auto adapter = loaded("lead_screw.xml");
    adapter->set_transmission_slip(0, s);
    PhysicsInputs inputs = shaft_torque(tau);
    inputs.joint_drives = {JointDrive{0, 0.01, hold_stiffness, 0.0, true}};
    adapter->set_inputs(inputs);
    adapter->step();
    const PhysicsOutputs& out = adapter->outputs();
    const double drive_force = out.joint_drive_torques[0].torque / lead_c;
    const double v = out.bodies[1].linear_velocity[0];
    REQUIRE(drive_force > 0.0);
    CHECK(relative(v, h * (((1.0 - s) * tau / lead_c) + drive_force) / reflected_mass) < 1e-9);
    CHECK(relative(out.transmissions[0].force, carriage_mass * v / h) < 1e-9);
}

TEST_CASE("MuJoCo drive re-enabled at a distant rest restarts the slip and play references there",
          "[PHY-006][TST-015][PHY-034]") {
    // lead_screw_backlash.xml, band b = 0.1 pi rad, s = 0.5. Ten rests 0.01 k (k = 0..9) give 9 increments:
    // slip = 0.5 * 0.09 = 0.045 rad. Three steps with the drive disabled while the rest moves to 2 rad, then enabled
    // there: no increment is counted (slip stays 0.045) and the band re-centres on the input 2 - slip, so with no load
    // the carriage settles exactly at c (2 - 0.045), and the shaft reports q_driven / c + slip = 2 rad.
    constexpr double s = 0.5;
    auto adapter = loaded("lead_screw_backlash.xml");
    adapter->set_transmission_slip(0, s);
    for (int k = 0; k < 10; ++k) {
        hold(*adapter, 0.01 * k, 1);
    }
    const double slip = adapter->outputs().transmissions[0].slip;
    CHECK(relative(slip, s * 0.09) < 1e-12);
    for (int k = 1; k <= 3; ++k) {
        adapter->set_inputs(driven(JointDrive{0, 0.5 * k, hold_stiffness, hold_damping(), false}));
        adapter->step();
    }
    hold(*adapter, 2.0, 1000);
    const PhysicsOutputs& out = adapter->outputs();
    CHECK(out.transmissions[0].slip == slip);
    CHECK(relative(out.joint_positions[1], lead_c * (2.0 - slip)) < 1e-9);
    CHECK(relative(out.joint_positions[0], 2.0) < 1e-9);
}

// ===== Implicit drive under slip and braking (brief M2-C2) ==========================================================

namespace {

// lead_screw.xml (horizontal) or lift_screw.xml (vertical) with carriage mass, coefficient c and rotor J changed:
// armature J / c^2 on the driven joint, the transmission's first number c.
std::unique_ptr<MujocoAdapter> reflected(const std::string& name, double carriage, double c, double rotor) {
    std::string text = fixture(name);
    std::ostringstream mass_text;
    mass_text << std::setprecision(17) << "mass=\"" << carriage << "\"";
    std::ostringstream armature_text;
    armature_text << std::setprecision(17) << "armature=\"" << rotor / (c * c) << "\"";
    std::ostringstream ratio_text;
    ratio_text << std::setprecision(17) << "data=\"" << c << " ";
    replace(text, "mass=\"10.0\"", mass_text.str());
    replace(text, "armature=\"3.330991485367658\"", armature_text.str());
    replace(text, "data=\"0.0012732395447351628 ", ratio_text.str());
    auto adapter = std::make_unique<MujocoAdapter>();
    adapter->load(PreparedModel{"", text, "0.1.0"}, Rate(1000));
    return adapter;
}

}  // namespace

TEST_CASE("MuJoCo drive is backward Euler: the rest of a step is reached at its end state", "[PHY-034][reference]") {
    // The 1 kg slider (A0 = 1 / m), K = 1e3 N/m, D = 0: with K A0 h^2 = 1e-3 the old start-state spring and backward
    // Euler differ by K w h on a moving joint. Pushed to w0 = F h / m = 1 m/s by F = 1000 N for one step, then with
    // the rest on the joint each step (rest_k = x_k), backward Euler gives (m + K h^2) a = -K w h, so
    // w' = w m / (m + K h^2): w decays by the factor 1 / (1 + K h^2 / m) per step, and the drive torque read back is
    // K (rest - x') = -K w' h. Held to 1e-9 (the scheme is exact; rounding).
    auto adapter = loaded("slider.xml");
    const double stiffness = 1e3;
    adapter->set_inputs(driven(JointDrive{0, 0.0, 0.0, 0.0, false}, 1000.0));
    adapter->step();
    double x = adapter->outputs().joint_positions[0];
    double w = adapter->outputs().bodies[0].linear_velocity[0];
    REQUIRE(relative(w, 1.0) < 1e-9);
    for (int k = 0; k < 10; ++k) {
        adapter->set_inputs(driven(JointDrive{0, x, stiffness, 0.0, true}));
        adapter->step();
        const PhysicsOutputs& out = adapter->outputs();
        const double expected = w / (1.0 + (stiffness * h * h));
        CHECK(relative(out.bodies[0].linear_velocity[0], expected) < 1e-9);
        CHECK(relative(out.joint_drive_torques[0].torque, -stiffness * expected * h) < 1e-9);
        x = out.joint_positions[0];
        w = out.bodies[0].linear_velocity[0];
    }
}

TEST_CASE("A stepper through a slipping belt drives the carriage at (1 - s) v and the shaft at v, bounded",
          "[SIG-033][PHY-006][PHY-034][reference]") {
    // G10's x axis: belt c = r = 0.04 / 2 pi m/rad (the gantry's 40 mm/turn pulley), 3 kg carriage, NEMA 17 class
    // stepper (1.8 deg, 0.40 N m, J = 5.4e-6 kg m^2, D = 2e-3 N m s/rad), 16 microsteps per tick through a 16x
    // driver: v = 1 full step per tick, 31.4 rad/s (300 rpm, 0.36 N m available). K_hold = 0.40 pi / (2 step)
    // = 20 N m/rad, K_d A0 h^2 = (20 / c^2) h^2 / (3 + J / c^2) = 0.157. Slip s = 0.3 for 500 ticks from tick 500.
    // Steady slip: the motor holds the reported shaft at v (the stepper locks its angle to the commanded steps), the
    // drive rest advances v per tick and the driven joint gets (1 - s) of it, (1 - s) v c. The drive torque stays
    // below K_hold step = tau_hold pi / 2 throughout (the tangent's torque is at most tau_hold at any lead below the
    // pull-out of 2 steps, and the implicit step only lowers the spring's energy). Transients decay by at least
    // 1 / sqrt(1 + K_hold h^2 / J_eff) = 0.93 per tick, so 450 ticks into the window they are below 1e-14: speeds to
    // 1e-6 relative.
    // Expected failure on the old start-state mapping (eq_data = rest c, the spring at q, v before the step): the
    // drive then applies K (rest - theta) with theta the angle the stepper sampled, not the step's end angle. The slip
    // onset (the driven joint suddenly sees (1 - s) of each increment) pushes |omega h - n' step| past one step and
    // the loss-of-synchronism rule publishes rest = theta + omega h - sign(omega) step with K = tau_avail(omega) /
    // step. At the start state that is K (omega h - step) = tau_avail(omega) (omega h / step - 1) along the motion,
    // positive whenever the shaft turns more than one step per tick: the brake drives the shaft faster, which raises
    // omega h further, until tau_avail(omega) = 0 at the curve's end, 1800 rpm = 188.5 rad/s, where K = 0 and the
    // shaft coasts (6 steps per tick, carriage (1 - s) 188.5 c = 0.84 m/s; X3 saw 0.78 m/s on G10). Measured on the
    // old mapping: carriage 0.834 m/s, shaft 186.5 rad/s, worst torque 0.82 N m, so each speed and the torque bound
    // below fail.
    const double step = std::numbers::pi / 100.0;
    const double c = 0.04 / (2.0 * std::numbers::pi);
    const double rpm = 2.0 * std::numbers::pi / 60.0;
    constexpr double s = 0.3;
    mechsim::signals::BehavioralStepperParams params;
    params.step_angle = step;
    params.rotor_inertia = rotor_inertia;
    params.holding_torque = 0.40;
    params.damping = 2e-3;
    params.torque_speed = {{0.0, 0.40}, {300.0 * rpm, 0.36}, {600.0 * rpm, 0.30}, {1200.0 * rpm, 0.18},
                           {1800.0 * rpm, 0.10}};
    mechsim::signals::BehavioralStepper motor("x_motor", params);
    motor.start(mechsim::signals::ComponentClock{Rate(1000), mechsim::kernel::Duration(1), 1});
    auto adapter = reflected("lead_screw.xml", 3.0, c, rotor_inertia);
    const std::vector<double> run{16.0 / 16.0, 1.0, 1.0, 24.0};
    const double bound = 0.40 * std::numbers::pi / 2.0;
    double worst = 0.0;
    double missed_in_window = 0.0;
    double shaft_before = 0.0;
    double shaft_rate = 0.0;
    double carriage_rate = 0.0;
    for (int k = 0; k < 1000; ++k) {
        if (k == 500) {
            adapter->set_transmission_slip(0, s);
        }
        motor.begin_tick(mechsim::kernel::SimTime(k));
        motor.sample(adapter->outputs());
        motor.set_inputs(run);
        motor.latch_outputs();
        if (k >= 500) {
            missed_in_window += motor.output_values()[2];
        }
        PhysicsInputs inputs;
        inputs.joint_drives.assign(motor.joint_drives().begin(), motor.joint_drives().end());
        shaft_before = adapter->outputs().joint_positions[0];
        adapter->set_inputs(inputs);
        adapter->step();
        const PhysicsOutputs& out = adapter->outputs();
        if (k > 0) {
            worst = std::max(worst, std::abs(out.joint_drive_torques[0].torque));
        }
        shaft_rate = (out.joint_positions[0] - shaft_before) / h;
        carriage_rate = out.bodies[1].linear_velocity[0];
    }
    const double v = step / h;
    std::cout << "stepper through slipping belt: carriage closed form " << (1.0 - s) * v * c << " m/s, measured "
              << carriage_rate << " m/s; shaft closed form " << v << " rad/s, measured " << shaft_rate
              << " rad/s; worst drive torque " << worst << " N m, bound " << bound << " N m\n";
    CHECK(adapter->outputs().transmissions[0].slipping);
    CHECK(missed_in_window == 0.0);
    CHECK(relative(carriage_rate, (1.0 - s) * v * c) < 1e-6);
    CHECK(relative(shaft_rate, v) < 1e-6);
    CHECK(worst < bound);
}

TEST_CASE("A brake catches a falling lift through the MuJoCo drive at the Coulomb deceleration, no overshoot",
          "[SIG-020][PHY-034][reference]") {
    // lift_screw.xml with a 2 kg lift on the gantry's 40 mm/turn screw, c = 0.04 / 2 pi, J = 5.4e-6 kg m^2; on the
    // shaft J_eff = m c^2 + J, so the brake's A0 = 1 / J_eff. Released, it falls at g m / (m + J / c^2) until past 0.2 m/s; the 0.5 N m brake
    // (K_b A0 h^2 = 100) then engages on the moving shaft and slips: the implicit drive at K_s = 1e-5 K_b gives
    // (m + J / c^2) a = T_b / c - m g to the apparent inertia K_s A0 h^2 = 1e-3, so the deceleration is
    // (T_b / c - m g) / (m + J / c^2) within the brief's 1 percent at every slipping step. It sticks before the
    // Coulomb torque reverses the lift; from there the implicit hold's energy about its sag position never grows and
    // the residual speed is below one step's Coulomb change a h, so the lift stays within
    // r = sqrt(J_eff (a h)^2 / K_b + (m g c / K_b)^2) (shaft rad) of the hold's equilibrium: no overshoot.
    const double c = 0.04 / (2.0 * std::numbers::pi);
    constexpr double lift = 2.0;
    const double inertia = (lift * c * c) + rotor_inertia;
    auto adapter = reflected("lift_screw.xml", lift, c, rotor_inertia);
    mechsim::signals::BrakeParams params;
    params.joint = 0;
    params.brake_torque = 0.5;
    params.inverse_inertia = 1.0 / inertia;
    mechsim::signals::Brake brake("lift_brake", params);
    brake.start(mechsim::signals::ComponentClock{Rate(1000), mechsim::kernel::Duration(1), 1});
    const double coulomb = ((params.brake_torque / c) - (lift * gravity)) / (lift + (rotor_inertia / (c * c)));
    std::vector<double> decelerations;
    std::vector<double> held;
    double rest_stuck = 0.0;
    bool engaged = false;
    bool stuck = false;
    double velocity = 0.0;
    for (int k = 0; k < 200; ++k) {
        if (!engaged && velocity < -0.2) {
            engaged = true;
        }
        brake.begin_tick(mechsim::kernel::SimTime(k));
        brake.sample(adapter->outputs());
        const std::vector<double> coil{static_cast<double>(!engaged), 24.0};
        brake.set_inputs(coil);
        brake.latch_outputs();
        PhysicsInputs inputs;
        inputs.joint_drives.assign(brake.joint_drives().begin(), brake.joint_drives().end());
        adapter->set_inputs(inputs);
        adapter->step();
        const double next = adapter->outputs().bodies[1].linear_velocity[2];
        if (engaged && !brake.joint_drives().empty()) {
            if (brake.joint_drives()[0].stiffness < brake.stiffness()) {
                REQUIRE_FALSE(stuck);
                decelerations.push_back((next - velocity) / h);
            } else {
                if (!stuck) {
                    rest_stuck = brake.joint_drives()[0].rest;
                }
                stuck = true;
                held.push_back(adapter->outputs().joint_positions[0]);
            }
        }
        velocity = next;
    }
    REQUIRE(decelerations.size() > 3);
    REQUIRE(stuck);
    std::cout << "brake catch through MuJoCo: closed-form deceleration " << coulomb << " m/s^2, measured "
              << decelerations[1] << " m/s^2\n";
    for (double rate : decelerations) {
        CHECK(std::abs(rate - coulomb) <= 0.01 * coulomb);
    }
    const double k_b = brake.stiffness();
    const double sag = lift * gravity * c / k_b;
    const double centre = rest_stuck - sag;
    const double radius = std::sqrt((inertia * (coulomb / c) * (coulomb / c) * h * h / k_b) + (sag * sag));
    for (double angle : held) {
        CHECK(std::abs(angle - centre) <= radius * (1.0 + 1e-6));
    }
    CHECK(std::abs(velocity) < 1e-6);
}
