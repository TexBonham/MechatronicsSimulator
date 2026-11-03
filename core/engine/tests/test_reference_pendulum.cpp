#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "multibody_fixtures.hpp"

using mechsim::engine::Body;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::rod;
using mechsim::engine::test::standard_gravity;

namespace {

constexpr double rod_length = 0.5;  // m
constexpr double rod_mass = 1.0;    // kg

// Uniform rod hanging from a revolute pivot at its end: I = m L^2 / 3 about the pivot, d = L / 2.
MultibodyModel hanging_rod(double damping) {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, -standard_gravity, 0.0);
    Body body = rod("rod", -1, Eigen::Vector3d::Zero(), rod_length, rod_mass);
    body.joint.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(-0.5 * std::numbers::pi, Eigen::Vector3d::UnitZ()));
    if (damping > 0.0) {
        body.joint.damping = Eigen::VectorXd::Constant(1, damping);
    }
    model.bodies.push_back(body);
    return model;
}

double pivot_inertia() {
    return rod_mass * rod_length * rod_length / 3.0;
}

double natural_frequency() {
    return std::sqrt(rod_mass * standard_gravity * 0.5 * rod_length / pivot_inertia());
}

}  // namespace

TEST_CASE("compound pendulum small-amplitude period matches 2 pi sqrt(I / (m g d))", "[ENG-036][reference]") {
    // Closed form T0 = 2 pi sqrt(I / (m g d)). Finite amplitude: T = T0 (1 + theta0^2 / 16 + O(theta0^4)), the
    // O(theta0^4) term 11 theta0^4 / 3072 = 2e-8 at theta0 = 0.05. Scheme (gravity explicit in c, package 2a):
    // T_num / T = Omega / (2 asin(Omega / 2)), Omega = omega h, about 1 - Omega^2 / 24. Tolerance 0.1 % of period
    // on the raw period; the corrected residual is also held to 10 % of the amplitude correction.
    constexpr double theta0 = 0.05;
    MultibodyProvider stepper(hanging_rod(0.0));
    SemiImplicitEuler integrator(StepConfig{});
    const double h = integrator.config().h;
    const double omega = natural_frequency();
    const double closed_form =
        2.0 * std::numbers::pi * std::sqrt(pivot_inertia() / (rod_mass * standard_gravity * 0.5 * rod_length));
    State state{Eigen::VectorXd::Constant(1, -theta0), Eigen::VectorXd::Zero(1), 0.0};
    std::vector<double> crossings;
    const int steps = static_cast<int>(std::ceil(11.0 * closed_form / h));
    for (int n = 0; n < steps; ++n) {
        const double before = state.q[0];
        const double t0 = state.t;
        stepper.step(integrator, state);
        if (before < 0.0 && state.q[0] >= 0.0) {
            crossings.push_back(t0 + h * before / (before - state.q[0]));
        }
    }
    REQUIRE(crossings.size() >= 10);
    const auto periods = static_cast<double>(crossings.size() - 1);
    const double measured = (crossings.back() - crossings.front()) / periods;
    const double omega_h = omega * h;
    const double amplitude = theta0 * theta0 / 16.0;
    const double scheme = omega_h / (2.0 * std::asin(0.5 * omega_h));
    const double corrected = measured / (closed_form * (1.0 + amplitude) * scheme) - 1.0;
    const double raw = measured / closed_form - 1.0;
    CAPTURE(measured, closed_form, raw, corrected, amplitude, scheme - 1.0);
    WARN("ENG-036 period: closed form " << closed_form << " s, measured " << measured << " s, raw error " << raw
                                        << ", corrected " << corrected);
    CHECK(std::abs(raw) <= 1e-3);
    CHECK(std::abs(corrected) <= 0.1 * amplitude);
}

TEST_CASE("damped pendulum decays with the logarithmic decrement 2 pi zeta / sqrt(1 - zeta^2)",
          "[ENG-037][reference]") {
    // Joint damping d = 2 zeta I omega, zeta = 0.05, implicit; gravity explicit. Linearized step with a = 1 / (1 +
    // 2 zeta Omega): (I + h d) v' = I v - h I omega^2 q, q' = q + h v', so the map has det a and trace
    // 1 + a - a Omega^2: eigenvalues sqrt(a) e^(+-i theta), cos(theta) = trace / (2 sqrt(a)). Successive maxima of
    // the discrete solution are 2 pi / theta steps apart with ratio a^(-pi / theta), so the scheme's decrement is
    // delta_num = pi ln(1 + 2 zeta Omega) / theta. Tolerance 1 % of the closed form; the measurement is also held
    // to 10 % of the scheme correction delta_num / delta - 1. Peaks by parabolic interpolation of three samples;
    // amplitude theta0 = 0.01 shifts the period, hence the decrement, by at most theta0^2 / 16 = 6e-6.
    constexpr double zeta = 0.05;
    constexpr double theta0 = 0.01;
    const double omega = natural_frequency();
    MultibodyProvider provider(hanging_rod(2.0 * zeta * pivot_inertia() * omega));
    SemiImplicitEuler integrator(StepConfig{});
    const double h = integrator.config().h;
    State state{Eigen::VectorXd::Constant(1, theta0), Eigen::VectorXd::Zero(1), 0.0};
    std::vector<double> peaks;
    double previous = state.q[0];
    double current = state.q[0];
    constexpr std::size_t wanted = 8;
    const int cap = static_cast<int>(std::ceil(10.0 * 2.0 * std::numbers::pi / omega / h));
    for (int n = 0; n < cap && peaks.size() < wanted; ++n) {
        provider.step(integrator, state);
        const double next = state.q[0];
        if (n > 0 && current > previous && current >= next) {
            const double curvature = next - 2.0 * current + previous;
            peaks.push_back(current - (next - previous) * (next - previous) / (8.0 * curvature));
        }
        previous = current;
        current = next;
    }
    REQUIRE(peaks.size() == wanted);
    const double measured = std::log(peaks.front() / peaks.back()) / static_cast<double>(wanted - 1);
    const double closed_form = 2.0 * std::numbers::pi * zeta / std::sqrt(1.0 - zeta * zeta);
    const double omega_h = omega * h;
    const double a = 1.0 / (1.0 + 2.0 * zeta * omega_h);
    const double theta = std::acos((1.0 + a - a * omega_h * omega_h) / (2.0 * std::sqrt(a)));
    const double scheme = std::numbers::pi * std::log(1.0 + 2.0 * zeta * omega_h) / theta;
    const double correction = scheme / closed_form - 1.0;
    CAPTURE(measured, closed_form, scheme, correction, measured / closed_form - 1.0);
    WARN("ENG-037 decrement: closed form " << closed_form << ", measured " << measured << ", scheme " << scheme
                                           << ", raw error " << measured / closed_form - 1.0);
    CHECK(std::abs(measured / closed_form - 1.0) <= 0.01);
    CHECK(std::abs(measured / scheme - 1.0) <= 0.1 * std::abs(correction));
}
