#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>

#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "providers.hpp"

using mechsim::engine::InvalidConfigError;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::StepHealth;
using mechsim::engine::test::LinearProvider;
using mechsim::engine::test::one_dof;

TEST_CASE("the default step length is 1 ms", "[ENG-002]") {
    CHECK(StepConfig{}.h == 1e-3);
    CHECK(StepConfig::default_step_s == 1e-3);
    CHECK(SemiImplicitEuler(StepConfig{}).config().h == StepConfig::default_step_s);
}

TEST_CASE("each configured step length is honored", "[ENG-002]") {
    const double mass = 2.0;
    const double force = 3.0;
    const double acceleration = force / mass;
    const double end_time = 1.0;
    const LinearProvider provider = one_dof(mass, 0.0, 0.0, force, 0.0);
    const StepConfig fine{};
    const StepConfig coarse{4.0 * StepConfig::default_step_s};
    for (const StepConfig& config : {fine, coarse}) {
        SemiImplicitEuler integrator(config);
        const auto steps = static_cast<int>(std::lround(end_time / config.h));
        State state{Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), 0.0};
        for (int i = 0; i < steps; ++i) {
            const StepHealth health = integrator.step(provider, state);
            REQUIRE(health.step == config.h);
        }
        CAPTURE(config.h, steps);
        CHECK(state.t == Catch::Approx(end_time).epsilon(1e-12));
        CHECK(state.v(0) == Catch::Approx(acceleration * end_time).epsilon(1e-12));
        const double n = steps;
        const double expected_q = 0.5 * acceleration * end_time * end_time * (1.0 + 1.0 / n);
        CHECK(state.q(0) == Catch::Approx(expected_q).epsilon(1e-12));
    }
}

TEST_CASE("a step length that is not finite and positive is rejected", "[ENG-050]") {
    CHECK_THROWS_AS(SemiImplicitEuler(StepConfig{0.0}), InvalidConfigError);
    CHECK_THROWS_AS(SemiImplicitEuler(StepConfig{-StepConfig::default_step_s}), InvalidConfigError);
    CHECK_THROWS_AS(SemiImplicitEuler(StepConfig{std::numeric_limits<double>::quiet_NaN()}), InvalidConfigError);
    CHECK_THROWS_AS(SemiImplicitEuler(StepConfig{std::numeric_limits<double>::infinity()}), InvalidConfigError);
    try {
        const SemiImplicitEuler rejected(StepConfig{0.0});
        FAIL("no InvalidConfigError raised");
    } catch (const mechsim::engine::EngineError& error) {
        CHECK(error.cause() == "step length h must be finite and positive");
        CHECK(error.time() == 0.0);
    }
}

TEST_CASE("free fall of a 2 kg mass matches the closed form in SI", "[ENG-004]") {
    const double mass_kg = 2.0;
    const double gravity_m_per_s2 = 9.81;
    const double start_height_m = 10.0;
    const LinearProvider provider = one_dof(mass_kg, 0.0, 0.0, 0.0, mass_kg * gravity_m_per_s2);
    SemiImplicitEuler integrator(StepConfig{});
    State state{Eigen::VectorXd::Constant(1, start_height_m), Eigen::VectorXd::Zero(1), 0.0};
    const int steps = 1000;
    for (int i = 0; i < steps; ++i) {
        integrator.step(provider, state);
    }
    CHECK(state.t == Catch::Approx(1.0).epsilon(1e-12));
    CHECK(state.v(0) == Catch::Approx(-9.81).epsilon(1e-12));
    CHECK(state.q(0) == Catch::Approx(start_height_m - 4.905 * 1.001).epsilon(1e-12));
}

TEST_CASE("the health record reports every field after each step", "[ENG-026]") {
    const StepConfig config;
    SemiImplicitEuler integrator(config);
    const LinearProvider provider = one_dof(1.0, 10.0, 0.1, 0.0, 0.0);
    State state{Eigen::VectorXd::Constant(1, 0.1), Eigen::VectorXd::Zero(1), 0.0};
    for (int i = 1; i <= 3; ++i) {
        const StepHealth health = integrator.step(provider, state);
        CHECK(health.max_constraint_drift == 0.0);
        CHECK(health.converged);
        CHECK(health.iterations == 0);
        CHECK(health.step == config.h);
        CHECK(health.time == state.t);
        CHECK(health.time == Catch::Approx(i * config.h));
    }
}
