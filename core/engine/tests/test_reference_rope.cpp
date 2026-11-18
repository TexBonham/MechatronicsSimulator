#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <numbers>
#include <vector>

#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_model.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/rope.hpp"
#include "rope_helpers.hpp"

using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::Rope;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::add_bob;
using mechsim::engine::test::bob_x;
using mechsim::engine::test::gravity;
using mechsim::engine::test::pendulum_inertia;
using mechsim::engine::test::pendulum_moment;
using mechsim::engine::test::pendulum_omega;
using mechsim::engine::test::rope_pendulum;
using mechsim::engine::test::RopePendulum;

// Rope references (ENG-047; ADR 0016, ACC-037 proposal; round 5 ruling 14).
//
// Scheme. The swing's restoring force reaches the step through the taut segments' geometric stiffness
// (T / d)(I - n n^T) in K, so the swing mode is integrated by backward Euler: per step the mode is multiplied by
// 1 / (1 + i omega h), a phase atan(omega h) (period error -(omega h)^2 / 3) and a modulus (1 + omega^2 h^2)^-1/2,
// i.e. an amplitude loss of about pi omega h per period. At h = 1 ms and omega = 3.13 rad/s that is 1 % per period;
// ACC-037's 2 % over 20 periods needs omega h <= 3.2e-4, so that check runs at h = 50 us (0.98 % predicted).
// Small amplitude theta0 = 0.02 rad: period correction theta0^2 / 16 = 2.5e-5.

namespace {

constexpr double theta0 = 0.02;

struct Swing {
    std::vector<double> crossings;  // upward zero crossings of the bob's x, s
    std::vector<double> peaks;      // positive local maxima of the bob's x after the start, m
};

Swing swing(MultibodyProvider& provider, SemiImplicitEuler& integrator, State& state, Eigen::Index bob, int steps) {
    Swing out;
    const double h = integrator.config().h;
    double previous = bob_x(provider, state, bob);
    double before_previous = previous;
    for (int n = 0; n < steps; ++n) {
        const double t0 = state.t;
        provider.step(integrator, state);
        const double x = bob_x(provider, state, bob);
        if (previous < 0.0 && x >= 0.0) {
            out.crossings.push_back(t0 + h * previous / (previous - x));
        }
        if (n >= 2 && previous > 0.0 && previous > before_previous && previous >= x) {
            out.peaks.push_back(previous);
        }
        before_previous = previous;
        previous = x;
    }
    return out;
}

double mean_period(const Swing& s) {
    return (s.crossings.back() - s.crossings.front()) / static_cast<double>(s.crossings.size() - 1);
}

State start(const MultibodyProvider& provider) {
    return State{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
}

// Exact small-oscillation frequency of the discrete chain as built (ADR 0016 cross-check): lateral displacements of
// the N - 1 nodes and the bob, masses m_r / N and M + m_r / (2 N), segment j carrying the weight below it, stiffness
// T_j / (L / N) between its ends; the lowest generalized eigenvalue of (K, M).
double chain_omega(const RopePendulum& p) {
    const int n = p.segments;
    const double node = p.rope_mass / n;
    Eigen::VectorXd mass = Eigen::VectorXd::Constant(n, node);
    mass[n - 1] = p.bob_mass + 0.5 * node;
    const double l = p.length / n;
    Eigen::MatrixXd k = Eigen::MatrixXd::Zero(n, n);
    for (int j = 0; j < n; ++j) {
        const double tension = gravity * mass.tail(n - j).sum();
        k(j, j) += tension / l;
        if (j > 0) {
            k(j - 1, j - 1) += tension / l;
            k(j, j - 1) -= tension / l;
            k(j - 1, j) -= tension / l;
        }
    }
    const Eigen::MatrixXd m = mass.asDiagonal();
    const Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> solver(k, m);
    return std::sqrt(solver.eigenvalues()[0]);
}

}  // namespace

// Brief: negligible rope mass (m_r / M = 1e-4), N = 4, the 1 ms default: period 2 pi sqrt(L / g) within 0.5 %.
// Budget: added mass ~ m_r / (6 M) = 2e-5, scheme (omega h)^2 / 3 = 3e-6, amplitude 2.5e-5.
TEST_CASE("rope pendulum with negligible rope mass has the point-mass period", "[ENG-047][reference]") {
    const RopePendulum p = rope_pendulum(4, 1.0, 1e-4, theta0, 0.0);
    MultibodyProvider provider(p.model);
    SemiImplicitEuler integrator(StepConfig{});
    State state = start(provider);
    const Swing s = swing(provider, integrator, state, p.bob, 7000);
    REQUIRE(s.crossings.size() >= 3);
    const double target = 2.0 * std::numbers::pi * std::sqrt(p.length / gravity);
    const double period = mean_period(s);
    std::printf("ENG-047 rope pendulum: period %.9g s, 2 pi sqrt(L / g) %.9g s (%.3g)\n", period, target,
                period / target - 1.0);
    REQUIRE(std::abs(period / target - 1.0) <= 0.005);
}

// ACC-037 proposal (ADR 0016): m_r / M = 0.05, N = 4, at h = 50 us (see Scheme): period within 0.1 % of
// T_N = 2 pi sqrt(I_N / (g S)) (the brief's added-mass check at 1 % is implied; the correction itself is -0.49 %
// of the period here, so a massless target would fail), amplitude after 20 periods within 2 % of the undamped
// closed form (constant L sin theta0; predicted numerical loss 20 pi omega h = 0.98 %). T_N is cross-checked against
// the chain's exact linearized eigenvalue at the same 0.1 % (Rayleigh error second order in m_r / M).
TEST_CASE("rope pendulum with rope mass keeps the chain period and amplitude over 20 periods",
          "[ENG-047][reference]") {
    const RopePendulum p = rope_pendulum(4, 1.0, 0.05, theta0, 0.0);
    MultibodyProvider provider(p.model);
    StepConfig config;
    config.h = 5e-5;
    SemiImplicitEuler integrator(config);
    State state = start(provider);
    const double omega = pendulum_omega(p);
    const double closed_form = 2.0 * std::numbers::pi / omega;
    const int steps = static_cast<int>(20.5 * closed_form / config.h);
    const Swing s = swing(provider, integrator, state, p.bob, steps);
    REQUIRE(s.crossings.size() >= 20);
    REQUIRE(s.peaks.size() >= 20);
    const double period = mean_period(s);
    const double initial = p.length * std::sin(theta0);
    const double massless = 2.0 * std::numbers::pi * std::sqrt(p.length / gravity);
    const double eigen = 2.0 * std::numbers::pi / chain_omega(p);
    std::printf("ACC-037 form: period %.9g s, T_N %.9g s (%.3g), chain eigenvalue %.9g s, massless %.9g s\n",
                period, closed_form, period / closed_form - 1.0, eigen, massless);
    std::printf("ACC-037 form: amplitude after 20 periods %.9g m of %.9g m (%.3g)\n", s.peaks[19], initial,
                s.peaks[19] / initial - 1.0);
    REQUIRE(std::abs(period / closed_form - 1.0) <= 1e-3);
    REQUIRE(std::abs(eigen / closed_form - 1.0) <= 1e-3);
    REQUIRE(std::abs(s.peaks[19] / initial - 1.0) <= 0.02);
}

// Scheme check, separate from the raw quantity: at the 1 ms default the swing loses amplitude as backward Euler
// predicts, A(t) / A0 = (1 + omega^2 h^2)^(-t / (2 h)); asserted within 10 % of the predicted loss over 10 periods
// (the chain's other modes and the start-up axial transient are not in the single-mode prediction).
TEST_CASE("rope pendulum numerical swing decay at 1 ms is the implicit scheme's", "[ENG-047][reference]") {
    const RopePendulum p = rope_pendulum(4, 1.0, 1e-3, theta0, 0.0);
    MultibodyProvider provider(p.model);
    SemiImplicitEuler integrator(StepConfig{});
    State state = start(provider);
    const double omega = pendulum_omega(p);
    const double h = integrator.config().h;
    const double period = 2.0 * std::numbers::pi / omega;
    const Swing s = swing(provider, integrator, state, p.bob, static_cast<int>(10.5 * period / h));
    REQUIRE(s.peaks.size() >= 10);
    const double initial = p.length * std::sin(theta0);
    const double predicted = 1.0 - std::pow(1.0 + omega * omega * h * h, -10.0 * period / (2.0 * h));
    const double measured = 1.0 - s.peaks[9] / initial;
    std::printf("ENG-047 scheme decay over 10 periods at 1 ms: measured %.6g, backward Euler %.6g\n", measured,
                predicted);
    REQUIRE(std::abs(measured / predicted - 1.0) <= 0.1);
}

// Ruling 14: attachment damping c at the top joint. Straight mode: I_N theta'' + c theta' + g S theta = 0,
// zeta = c / (2 sqrt(I_N g S)), logarithmic decrement per period delta = 2 pi zeta / sqrt(1 - zeta^2) between
// successive peaks. zeta = 0.02 (delta = 0.1257) at h = 0.1 ms: the scheme adds pi omega h = 9.8e-4 (0.8 % of
// delta), the rope mass (m_r / M = 1e-3) enters at second order; asserted within 2 % of delta.
TEST_CASE("rope pendulum on a damped attachment decays with the joint's logarithmic decrement",
          "[ENG-047][reference]") {
    const double zeta = 0.02;
    const RopePendulum shape = rope_pendulum(4, 1.0, 1e-3, theta0, 0.0);
    const double damping = 2.0 * zeta * std::sqrt(pendulum_inertia(shape) * gravity * pendulum_moment(shape));
    const RopePendulum p = rope_pendulum(4, 1.0, 1e-3, theta0, damping);
    MultibodyProvider provider(p.model);
    StepConfig config;
    config.h = 1e-4;
    SemiImplicitEuler integrator(config);
    State state = start(provider);
    const double period = 2.0 * std::numbers::pi / pendulum_omega(p);
    const Swing s = swing(provider, integrator, state, p.bob, static_cast<int>(4.5 * period / config.h));
    REQUIRE(s.peaks.size() >= 4);
    const double measured = std::log(s.peaks[0] / s.peaks[3]) / 3.0;
    const double closed_form = 2.0 * std::numbers::pi * zeta / std::sqrt(1.0 - zeta * zeta);
    std::printf("ENG-047 attachment damping: decrement %.6g, closed form %.6g (%.3g)\n", measured, closed_form,
                measured / closed_form - 1.0);
    REQUIRE(std::abs(measured / closed_form - 1.0) <= 0.02);
}

// Hanging rope, end mass m_e, gravity: in equilibrium each segment carries the weight below its upper end, so the
// top segment carries g (m_e + m_r - m_r / (2 N)): the end body with its added half-mass, the N - 1 interior nodes,
// but not the half-mass lumped on the world attachment. Axial damping makes the lowest axial mode about critical
// (zeta_1 = damping omega_1 / (2 EA) = 1 at damping = 2 EA / omega_1, omega_1 ~ (pi / 2) sqrt(EA / (mu L^2)));
// higher modes are overdamped and decay at about EA / damping = 78 /s or faster, so 3000 steps leave below
// exp(-200). The vertical chain has no lateral excitation. Asserted within 1e-9 relative.
TEST_CASE("a hanging rope carries its weight below the top attachment", "[ENG-047][reference]") {
    const double rest = 1.0;
    const double mu = 0.1;
    const double end_mass = 0.01;
    const double axial = 1000.0;
    const int segments = 4;
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, 0.0, -gravity);
    const Eigen::Vector3d bottom(0.0, 0.0, -rest);
    const Eigen::Index end = add_bob(model, bottom, end_mass);
    Rope rope;
    rope.to_body = end;
    rope.segments = segments;
    rope.axial_rigidity = axial;
    rope.mass_per_length = mu;
    rope.rest_length = rest;
    rope.damping = 2.0 * axial / (0.5 * std::numbers::pi * std::sqrt(axial / (mu * rest * rest)));
    static_cast<void>(add_rope(model, rope, Eigen::Vector3d::Zero(), bottom));
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(StepConfig{});
    State state = start(provider);
    for (int n = 0; n < 3000; ++n) {
        provider.step(integrator, state);
    }
    const double rope_mass = mu * rest;
    const double expected = gravity * (end_mass + rope_mass - rope_mass / (2.0 * segments));
    const double top = provider.outputs().ropes[0].tension[0];
    std::printf("ENG-047 hanging rope: top tension %.15g N, closed form %.15g N\n", top, expected);
    REQUIRE(std::abs(top - expected) <= 1e-9 * expected);
    REQUIRE(provider.outputs().ropes[0].max_tension == top);
}
