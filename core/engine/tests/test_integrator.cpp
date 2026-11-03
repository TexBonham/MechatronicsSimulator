#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <bit>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <type_traits>
#include <vector>

#include "mechsim/engine/integrator.hpp"
#include "providers.hpp"

using mechsim::engine::DynamicsProvider;
using mechsim::engine::DynamicsTerms;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::StepHealth;
using mechsim::engine::test::LinearProvider;
using mechsim::engine::test::one_dof;
using mechsim::engine::test::two_dof;

TEST_CASE("stiff mass-spring-damper at omega h = 20 overshoots at most 1 percent and settles within 100 steps",
          "[ENG-003]") {
    const StepConfig config;
    const double mass = 1.0;
    const double omega = 20.0 / config.h;
    const double zeta = 0.5;
    const double stiffness = mass * omega * omega;
    const double damping = 2.0 * zeta * mass * omega;
    const double force = 4.0e5;
    const double static_deflection = force / stiffness;
    const LinearProvider provider = one_dof(mass, stiffness, damping, force, 0.0);
    SemiImplicitEuler integrator(config);
    State state{Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), 0.0};

    // Continuous time overshoots by exp(-pi zeta / sqrt(1 - zeta^2)) = 16.3 percent; the implicit step damps the
    // stiff mode further, so the bound is 1 percent.
    double max_q = 0.0;
    int settled_at = -1;
    for (int i = 1; i <= 100; ++i) {
        integrator.step(provider, state);
        max_q = std::max(max_q, state.q(0));
        const bool settled = std::abs(state.q(0) - static_deflection) <= 1e-6 * static_deflection;
        if (settled && settled_at < 0) {
            settled_at = i;
        }
        if (!settled) {
            settled_at = -1;
        }
    }
    CAPTURE(max_q, static_deflection, settled_at);
    WARN("stiff overshoot ratio " << max_q / static_deflection << ", settled at step " << settled_at);
    CHECK(max_q <= 1.01 * static_deflection);
    CHECK(settled_at > 0);
    CHECK(std::abs(state.q(0) - static_deflection) <= 1e-6 * static_deflection);
}

TEST_CASE("undamped oscillator at omega h = 0.1 has the implicit period error and non-increasing energy",
          "[ENG-003]") {
    const StepConfig config;
    const double h = config.h;
    const double mass = 1.0;
    const double omega = 0.1 / h;
    const double stiffness = mass * omega * omega;
    const double x0 = 0.01;
    const LinearProvider provider = one_dof(mass, stiffness, 0.0, 0.0, 0.0);
    SemiImplicitEuler integrator(config);
    State state{Eigen::VectorXd::Constant(1, x0), Eigen::VectorXd::Zero(1), 0.0};
    double energy = 0.5 * stiffness * x0 * x0;
    bool energy_non_increasing = true;
    std::vector<double> upward_crossings;
    for (int i = 0; i < 10000; ++i) {
        const double q_before = state.q(0);
        const double t_before = state.t;
        integrator.step(provider, state);
        const double next = 0.5 * mass * state.v(0) * state.v(0) + 0.5 * stiffness * state.q(0) * state.q(0);
        if (next > energy) {
            energy_non_increasing = false;
        }
        energy = next;
        if (q_before < 0.0 && state.q(0) >= 0.0) {
            const double fraction = -q_before / (state.q(0) - q_before);
            upward_crossings.push_back(t_before + fraction * h);
        }
    }
    REQUIRE(upward_crossings.size() >= 100);
    const auto periods = static_cast<double>(upward_crossings.size() - 1);
    const double measured_period = (upward_crossings.back() - upward_crossings.front()) / periods;
    const double exact_period = 2.0 * std::numbers::pi / omega;
    const double period_error = measured_period / exact_period - 1.0;

    // The linear one-step map is z' = z / (1 + i omega h), so it rotates by theta = atan(omega h) per step and
    // contracts energy by 1 / (1 + (omega h)^2); the period error omega h / theta - 1 is about (omega h)^2 / 3.
    const double omega_h = omega * h;
    const double expected_error = omega_h * omega_h / 3.0;
    CAPTURE(period_error, expected_error);
    WARN("period error " << period_error << ", expected " << expected_error);
    CHECK(std::abs(period_error - expected_error) <= 0.1 * expected_error);
    CHECK(energy_non_increasing);
}

TEST_CASE("2-DOF coupled system with a non-diagonal mass matrix solves the linear system", "[ENG-001][ENG-003]") {
    static_assert(std::is_same_v<decltype(State::q), Eigen::VectorXd>);
    static_assert(std::is_same_v<decltype(DynamicsTerms::mass), Eigen::MatrixXd>);
    const StepConfig config;
    const double h = config.h;
    const LinearProvider provider = two_dof();
    SemiImplicitEuler integrator(config);
    State state{Eigen::Vector2d(0.01, -0.02), Eigen::Vector2d(0.3, -0.1), 0.0};
    const State start = state;
    integrator.step(provider, state);

    DynamicsTerms terms;
    provider.evaluate(0.0, start.q, start.v, terms);
    const double a00 = terms.mass(0, 0) + h * terms.damping(0, 0) + h * h * terms.stiffness(0, 0);
    const double a01 = terms.mass(0, 1) + h * terms.damping(0, 1) + h * h * terms.stiffness(0, 1);
    const double a10 = terms.mass(1, 0) + h * terms.damping(1, 0) + h * h * terms.stiffness(1, 0);
    const double a11 = terms.mass(1, 1) + h * terms.damping(1, 1) + h * h * terms.stiffness(1, 1);
    const double kv0 = terms.stiffness(0, 0) * start.v(0) + terms.stiffness(0, 1) * start.v(1);
    const double kv1 = terms.stiffness(1, 0) * start.v(0) + terms.stiffness(1, 1) * start.v(1);
    const double b0 = h * (terms.force(0) - terms.bias(0)) - h * h * kv0;
    const double b1 = h * (terms.force(1) - terms.bias(1)) - h * h * kv1;
    const double det = a00 * a11 - a01 * a10;
    const double dv0 = (b0 * a11 - a01 * b1) / det;
    const double dv1 = (a00 * b1 - a10 * b0) / det;
    CHECK(state.v(0) == Catch::Approx(start.v(0) + dv0).epsilon(1e-14));
    CHECK(state.v(1) == Catch::Approx(start.v(1) + dv1).epsilon(1e-14));
    CHECK(state.q(0) == Catch::Approx(start.q(0) + h * state.v(0)).epsilon(1e-14));
    CHECK(state.q(1) == Catch::Approx(start.q(1) + h * state.v(1)).epsilon(1e-14));

    for (int i = 0; i < 20000; ++i) {
        integrator.step(provider, state);
    }
    Eigen::Matrix2d stiffness;
    stiffness << 300.0, -100.0, -100.0, 200.0;
    const Eigen::Vector2d equilibrium = stiffness.inverse() * Eigen::Vector2d(4.0, -2.0);
    CHECK(state.q(0) == Catch::Approx(equilibrium(0)).epsilon(1e-9));
    CHECK(state.q(1) == Catch::Approx(equilibrium(1)).epsilon(1e-9));
}

namespace {

std::vector<std::uint64_t> run_bits(int steps) {
    const LinearProvider provider = two_dof();
    SemiImplicitEuler integrator(StepConfig{});
    State state{Eigen::Vector2d(0.01, -0.02), Eigen::Vector2d(0.3, -0.1), 0.0};
    std::vector<std::uint64_t> bits;
    for (int i = 0; i < steps; ++i) {
        integrator.step(provider, state);
        for (const double x : {state.q(0), state.q(1), state.v(0), state.v(1), state.t}) {
            bits.push_back(std::bit_cast<std::uint64_t>(x));
        }
    }
    return bits;
}

}  // namespace

TEST_CASE("two runs from the same state are bitwise identical after every step", "[ENG-023]") {
    const std::vector<std::uint64_t> first = run_bits(1000);
    const std::vector<std::uint64_t> second = run_bits(1000);
    REQUIRE(first.size() == 5000);
    CHECK(first == second);
}

TEST_CASE("a zero-DOF system steps time only", "[ENG-003][ENG-026]") {
    const LinearProvider provider(Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 0), Eigen::MatrixXd(0, 0),
                                  Eigen::VectorXd(0), Eigen::VectorXd(0));
    SemiImplicitEuler integrator(StepConfig{});
    State state{Eigen::VectorXd(0), Eigen::VectorXd(0), 0.0};
    const StepHealth health = integrator.step(provider, state);
    CHECK(state.q.size() == 0);
    CHECK(state.v.size() == 0);
    CHECK(state.t == StepConfig::default_step_s);
    CHECK(health.time == state.t);
    CHECK(health.converged);
}

namespace {

// One velocity drives the first position; the second position counts elapsed time, so nq != nv.
class ClockProvider final : public DynamicsProvider {
public:
    [[nodiscard]] Eigen::Index position_size() const override { return 2; }
    [[nodiscard]] Eigen::Index velocity_size() const override { return 1; }

    void evaluate(double /*t*/, const Eigen::VectorXd& /*q*/, const Eigen::VectorXd& /*v*/,
                  DynamicsTerms& terms) const override {
        terms.mass = Eigen::MatrixXd::Constant(1, 1, 2.0);
        terms.stiffness = Eigen::MatrixXd::Zero(1, 1);
        terms.damping = Eigen::MatrixXd::Zero(1, 1);
        terms.bias = Eigen::VectorXd::Zero(1);
        terms.force = Eigen::VectorXd::Constant(1, 4.0);
    }

    [[nodiscard]] Eigen::VectorXd integrate_positions(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                      double h) const override {
        return Eigen::Vector2d(q(0) + h * v(0), q(1) + h);
    }

    double project(Eigen::VectorXd& q, Eigen::VectorXd& /*v*/) const override {
        const double drift = std::max(0.0, q(0) - 1.0);
        q(0) = std::min(q(0), 1.0);
        return drift;
    }
};

}  // namespace

TEST_CASE("positions advance through the provider and the projection hook feeds the health record",
          "[ENG-003][ENG-026]") {
    const ClockProvider provider;
    const StepConfig config;
    SemiImplicitEuler integrator(config);
    State state{Eigen::Vector2d(0.0, 0.0), Eigen::VectorXd::Zero(1), 0.0};
    StepHealth health = integrator.step(provider, state);
    CHECK(state.q(1) == config.h);
    CHECK(state.v(0) == Catch::Approx(2.0 * config.h));
    CHECK(state.q(0) == Catch::Approx(2.0 * config.h * config.h));
    CHECK(health.max_constraint_drift == 0.0);

    state.q(0) = 1.5;
    health = integrator.step(provider, state);
    CHECK(state.q(0) == 1.0);
    CHECK(health.max_constraint_drift == Catch::Approx(0.5 + config.h * state.v(0)));
}
