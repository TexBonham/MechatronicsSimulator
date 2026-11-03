#include <Eigen/Core>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <stdexcept>
#include <string>

#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "providers.hpp"

using mechsim::engine::EngineError;
using mechsim::engine::InvalidModelError;
using mechsim::engine::NonFiniteStateError;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::SolverNonConvergenceError;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::LinearProvider;
using mechsim::engine::test::one_dof;

TEST_CASE("NaN force raises a non-finite error at the time of the failing step", "[ENG-025]") {
    const StepConfig config;
    SemiImplicitEuler integrator(config);
    LinearProvider provider = one_dof(1.0, 10.0, 0.0, 0.0, 0.0);
    const double fail_time = 5.0 * config.h;
    provider.set_nan_from_time(fail_time - 0.5 * config.h);
    State state{Eigen::VectorXd::Constant(1, 0.1), Eigen::VectorXd::Zero(1), 0.0};
    for (int i = 0; i < 5; ++i) {
        integrator.step(provider, state);
    }
    const State before = state;
    try {
        integrator.step(provider, state);
        FAIL("no EngineError raised");
    } catch (const NonFiniteStateError& error) {
        CHECK(error.time() == before.t);
        CHECK(error.time() == Catch::Approx(fail_time));
        CHECK(error.cause() == "velocity increment dv is not finite");
    }
    CHECK(state.t == before.t);
    CHECK(state.q == before.q);
    CHECK(state.v == before.v);
}

TEST_CASE("an infinite velocity raises a non-finite error", "[ENG-025]") {
    SemiImplicitEuler integrator(StepConfig{});
    const LinearProvider provider = one_dof(1.0, 0.0, 0.0, 0.0, 0.0);
    State state{Eigen::VectorXd::Zero(1), Eigen::VectorXd::Constant(1, std::numeric_limits<double>::infinity()), 2.0};
    REQUIRE_THROWS_AS(integrator.step(provider, state), NonFiniteStateError);
}

TEST_CASE("every engine error carries time and cause and is an EngineError", "[ENG-025]") {
    const SolverNonConvergenceError solver(1.5, "constraint solver hit its iteration limit");
    const EngineError& base = solver;
    CHECK(base.time() == 1.5);
    CHECK(base.cause() == "constraint solver hit its iteration limit");
    CHECK(std::string(base.what()) == "engine error at t = 1.5 s: constraint solver hit its iteration limit");
    const NonFiniteStateError non_finite(0.25, "q is not finite");
    CHECK(static_cast<const EngineError&>(non_finite).time() == 0.25);
    const InvalidModelError invalid(0.0, "mass matrix is not positive definite");
    CHECK(static_cast<const std::runtime_error&>(invalid).what() != nullptr);
}

TEST_CASE("zero mass is rejected", "[ENG-049]") {
    SemiImplicitEuler integrator(StepConfig{});
    State state{Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), 0.0};
    SECTION("zero 1-DOF mass with a spring") {
        const LinearProvider provider = one_dof(0.0, 100.0, 0.0, 1.0, 0.0);
        REQUIRE_THROWS_AS(integrator.step(provider, state), InvalidModelError);
    }
    SECTION("negative 1-DOF mass") {
        const LinearProvider provider = one_dof(-1.0, 0.0, 0.0, 1.0, 0.0);
        REQUIRE_THROWS_AS(integrator.step(provider, state), InvalidModelError);
    }
    SECTION("asymmetric 2-DOF mass") {
        Eigen::MatrixXd mass(2, 2);
        mass << 2.0, 0.5, 0.4, 2.0;
        const LinearProvider provider(mass, Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Zero(2, 2),
                                      Eigen::VectorXd::Ones(2), Eigen::VectorXd::Zero(2));
        State two{Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), 0.0};
        try {
            integrator.step(provider, two);
            FAIL("no InvalidModelError raised");
        } catch (const InvalidModelError& error) {
            CHECK(error.cause() == "mass matrix is not symmetric");
        }
    }
    SECTION("singular non-diagonal 2-DOF mass") {
        Eigen::MatrixXd mass(2, 2);
        mass << 1.0, 1.0, 1.0, 1.0;
        const LinearProvider provider(mass, Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Zero(2, 2),
                                      Eigen::VectorXd::Ones(2), Eigen::VectorXd::Zero(2));
        State two{Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), 0.0};
        REQUIRE_THROWS_AS(integrator.step(provider, two), InvalidModelError);
    }
}

TEST_CASE("state sizes that disagree with the provider are rejected", "[ENG-049]") {
    SemiImplicitEuler integrator(StepConfig{});
    const LinearProvider provider = one_dof(1.0, 0.0, 0.0, 0.0, 0.0);
    State state{Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(1), 0.0};
    REQUIRE_THROWS_AS(integrator.step(provider, state), InvalidModelError);
    state.q = Eigen::VectorXd::Zero(1);
    state.v = Eigen::VectorXd::Zero(3);
    REQUIRE_THROWS_AS(integrator.step(provider, state), InvalidModelError);
}

namespace {

class MisSizedTermsProvider final : public mechsim::engine::DynamicsProvider {
public:
    explicit MisSizedTermsProvider(int which) : which_(which) {}

    [[nodiscard]] Eigen::Index position_size() const override { return 1; }
    [[nodiscard]] Eigen::Index velocity_size() const override { return 1; }

    void evaluate(double /*t*/, const Eigen::VectorXd& /*q*/, const Eigen::VectorXd& /*v*/,
                  mechsim::engine::DynamicsTerms& terms) const override {
        terms.mass = Eigen::MatrixXd::Identity(1, 1);
        terms.stiffness = Eigen::MatrixXd::Zero(1, 1);
        terms.damping = Eigen::MatrixXd::Zero(1, 1);
        terms.bias = Eigen::VectorXd::Zero(1);
        terms.force = Eigen::VectorXd::Zero(1);
        if (which_ == 0) {
            terms.mass = Eigen::MatrixXd::Identity(2, 2);
        }
        if (which_ == 1) {
            terms.stiffness = Eigen::MatrixXd::Zero(1, 2);
        }
        if (which_ == 2) {
            terms.damping = Eigen::MatrixXd::Zero(2, 1);
        }
        if (which_ == 3) {
            terms.bias = Eigen::VectorXd::Zero(2);
        }
        if (which_ == 4) {
            terms.force = Eigen::VectorXd::Zero(0);
        }
    }

    [[nodiscard]] Eigen::VectorXd integrate_positions(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                      double h) const override {
        if (which_ == 5) {
            return Eigen::VectorXd::Zero(3);
        }
        return q + h * v;
    }

private:
    int which_;
};

}  // namespace

TEST_CASE("provider terms of the wrong size are rejected", "[ENG-049]") {
    SemiImplicitEuler integrator(StepConfig{});
    for (int which = 0; which < 6; ++which) {
        const MisSizedTermsProvider provider(which);
        State state{Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), 0.0};
        CAPTURE(which);
        REQUIRE_THROWS_AS(integrator.step(provider, state), InvalidModelError);
    }
    const MisSizedTermsProvider valid(-1);
    State state{Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), 0.0};
    REQUIRE_NOTHROW(integrator.step(valid, state));
}

namespace {

class NanDriftProvider final : public mechsim::engine::DynamicsProvider {
public:
    [[nodiscard]] Eigen::Index position_size() const override { return 1; }
    [[nodiscard]] Eigen::Index velocity_size() const override { return 1; }

    void evaluate(double /*t*/, const Eigen::VectorXd& /*q*/, const Eigen::VectorXd& /*v*/,
                  mechsim::engine::DynamicsTerms& terms) const override {
        terms.mass = Eigen::MatrixXd::Identity(1, 1);
        terms.stiffness = Eigen::MatrixXd::Zero(1, 1);
        terms.damping = Eigen::MatrixXd::Zero(1, 1);
        terms.bias = Eigen::VectorXd::Zero(1);
        terms.force = Eigen::VectorXd::Zero(1);
    }

    [[nodiscard]] Eigen::VectorXd integrate_positions(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                      double h) const override {
        return q + h * v;
    }

    double project(Eigen::VectorXd& /*q*/, Eigen::VectorXd& /*v*/) const override {
        return std::numeric_limits<double>::quiet_NaN();
    }
};

}  // namespace

TEST_CASE("a non-finite drift from the projection raises a non-finite error at the step time", "[ENG-025]") {
    SemiImplicitEuler integrator(StepConfig{});
    const NanDriftProvider provider;
    State state{Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), 0.75};
    try {
        integrator.step(provider, state);
        FAIL("no NonFiniteStateError raised");
    } catch (const NonFiniteStateError& error) {
        CHECK(error.time() == 0.75);
        CHECK(error.cause() == "constraint drift is not finite");
    }
    CHECK(state.t == 0.75);
}
