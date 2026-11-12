#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_model.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/transmission.hpp"

using mechsim::engine::Body;
using mechsim::engine::JointType;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::Transmission;
using mechsim::engine::TransmissionType;

// Belt compliance (ENG-043 stiffness path, round 5 ruling 10): with the shaft held, the carriage of mass m on a belt
// of stiffness k is a spring-mass oscillator, omega = sqrt(k / m). "Held": shaft inertia J with r^2 m / J = 1e-6, so
// the two-mass frequency sqrt(k (r^2 / J + 1 / m)) (transmission.hpp) differs by 5e-7 relative.
//
// Scheme and envelope. The coupling enters K and D (ADR 0013), so the mode is integrated by backward Euler: stable
// for every k and h (|z| = (1 + omega^2 h^2)^-1/2 < 1, no omega h limit), with the discrete frequency
// atan(omega h) / h, error -(omega h)^2 / 3: below 1 % for omega h <= 0.17, i.e. f <= 27 Hz at 1 kHz. The gantry
// placeholder (catalog belt_pulley 100 kN/m on about 1 kg) has omega h = 0.32: stable, frequency 3.2 % low and the
// mode damped by about 5 % per step, so its transient is resolved only qualitatively at 1 ms.

namespace {

constexpr double radius = 0.01;
constexpr double carriage_mass = 1.0;

MultibodyModel belt_rig(double stiffness) {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    Body shaft;
    shaft.name = "shaft";
    shaft.joint.type = JointType::Revolute;
    shaft.mass = 1.0;
    shaft.inertia = 1e6 * carriage_mass * radius * radius * Eigen::Matrix3d::Identity();
    model.bodies.push_back(shaft);
    Body carriage;
    carriage.name = "carriage";
    carriage.joint.type = JointType::Prismatic;
    carriage.joint.axis = Eigen::Vector3d::UnitX();
    carriage.mass = carriage_mass;
    model.bodies.push_back(carriage);
    Transmission belt;
    belt.type = TransmissionType::BeltPulley;
    belt.input_body = 0;
    belt.output_body = 1;
    belt.ratio = radius;
    belt.stiffness = stiffness;
    model.transmissions.push_back(belt);
    return model;
}

double oscillation_period(double stiffness, int steps) {
    MultibodyProvider provider(belt_rig(stiffness));
    SemiImplicitEuler integrator(StepConfig{});
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    const double h = integrator.config().h;
    state.q[1] = 1e-3;
    std::vector<double> crossings;
    double previous = state.q[1];
    for (int n = 0; n < steps; ++n) {
        const double t0 = state.t;
        provider.step(integrator, state);
        const double x = state.q[1];
        if (previous < 0.0 && x >= 0.0) {
            crossings.push_back(t0 + h * previous / (previous - x));
        }
        previous = x;
    }
    REQUIRE(crossings.size() >= 3);
    return (crossings.back() - crossings.front()) / static_cast<double>(crossings.size() - 1);
}

}  // namespace

TEST_CASE("belt with the shaft held is a spring between shaft and carriage", "[ENG-043][reference]") {
    const double h = StepConfig{}.h;
    const double stiffness = 1e4;  // omega h = 0.1
    const double omega = std::sqrt(stiffness / carriage_mass);
    const double frequency = 1.0 / oscillation_period(stiffness, 1000);
    const double closed_form = omega / (2.0 * std::numbers::pi);
    const double scheme = std::atan(omega * h) / h / (2.0 * std::numbers::pi);
    std::printf("ENG-043 belt: %.9g Hz, sqrt(k / m) / (2 pi) %.9g Hz (%.3g), backward Euler %.9g Hz\n", frequency,
                closed_form, frequency / closed_form - 1.0, scheme);
    REQUIRE(std::abs(frequency / closed_form - 1.0) <= 0.01);
    REQUIRE(std::abs(frequency / scheme - 1.0) <= 1e-4);
}

// Envelope: the gantry placeholder stiffness at 1 kHz. One step from rest at x0 is backward Euler's exactly,
// x1 = x0 / (1 + omega^2 h^2), v1 = -h omega^2 x0 / (1 + omega^2 h^2) (the shaft's participation r^2 m / J = 1e-6
// changes omega^2 by that fraction, hence 1e-5), and over 1000 steps the amplitude never exceeds x0: implicit, no
// omega h limit, the mode heavily damped by the scheme (|z| = 0.95 per step).
TEST_CASE("belt at the gantry placeholder stiffness is stable at 1 kHz", "[ENG-043][reference]") {
    const double h = StepConfig{}.h;
    const double stiffness = 1e5;
    const double omega = std::sqrt(stiffness / carriage_mass);
    MultibodyProvider provider(belt_rig(stiffness));
    SemiImplicitEuler integrator(StepConfig{});
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    const double x0 = 1e-3;
    state.q[1] = x0;
    provider.step(integrator, state);
    const double wh2 = omega * omega * h * h;
    std::printf("ENG-043 belt at 100 kN/m, omega h %.3g: x1 %.12g (backward Euler %.12g), v1 %.12g (%.12g)\n",
                omega * h, state.q[1], x0 / (1.0 + wh2), state.v[1], -h * omega * omega * x0 / (1.0 + wh2));
    REQUIRE(std::abs(state.q[1] / (x0 / (1.0 + wh2)) - 1.0) <= 1e-5);
    REQUIRE(std::abs(state.v[1] / (-h * omega * omega * x0 / (1.0 + wh2)) - 1.0) <= 1e-5);
    double largest = 0.0;
    for (int n = 0; n < 1000; ++n) {
        provider.step(integrator, state);
        largest = std::max(largest, std::abs(state.q[1]));
    }
    REQUIRE(largest < x0);
}
