#include <Eigen/Core>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

#include "beam_helpers.hpp"
#include "carried_helpers.hpp"
#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/member_loads.hpp"
#include "mechsim/engine/multibody_provider.hpp"

using mechsim::engine::BeamEnds;
using mechsim::engine::CarriedLoadOutput;
using mechsim::engine::InvalidModelError;
using mechsim::engine::lag_growth_factor;
using mechsim::engine::lag_spectral_radius;
using mechsim::engine::lag_stability_boundary;
using mechsim::engine::MultibodyConfig;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::carried_rig;
using mechsim::engine::test::CarriedRig;
using mechsim::engine::test::ParentKind;
using mechsim::engine::test::point_height;
using mechsim::engine::test::run_steps;
using mechsim::engine::test::start_state;

// The lagged pair's stability (ENG-052, brief M3-E7b; derivation in member_loads.hpp, Stability): per bending mode
// (z - r)(z - 1)^2 + 2 zeta x z^2 (z - 1) + x^2 z^3 = 0, growth exactly when r > (1 + x^2) / 2 undamped.
// With the carried mass on the member (brief M3-E7d) the same cubic holds on the modes of K and H + B, and
// |r| <= 1/2 below every boundary: the refusal is out of reach of a single mode. The E7b cases that reach and cross
// the boundary therefore run the one-way pair without the carried mass (MultibodyConfig::carried_added_mass, a test
// seam): they test the lag's recursion and the refusal, which the engine keeps as a guard.

namespace {

constexpr double step = 1e-3;

// The free-parent rig: a 5 kg parent on a world prismatic joint along y (free across the member, in its bending
// plane), a pinned-pinned steel bar of 8 segments without node damping, the carriage at midspan, gravity off.
CarriedRig free_rig(double carriage_mass) {
    CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, carriage_mass, 0.0, true, ParentKind::Sliding, 0.0);
    rig.model.gravity = Eigen::Vector3d::Zero();
    return rig;
}

MultibodyConfig one_way() {
    MultibodyConfig config;
    config.carried_added_mass = false;
    return config;
}

MultibodyConfig unchecked() {
    MultibodyConfig config = one_way();
    config.refuse_unstable_carried = false;
    return config;
}

CarriedLoadOutput lag_at(double carriage_mass, const MultibodyConfig& config = unchecked()) {
    const CarriedRig rig = free_rig(carriage_mass);
    MultibodyProvider provider(rig.model, config);
    State state = start_state(provider, rig.carriage, 0.5);
    run_steps(provider, state, 1, step);
    return provider.outputs().carried.front();
}

// The carriage mass whose undamped per-step factor (the spectral radius of the linear step, member_loads.hpp) is
// `factor`; it grows with the carried mass, so bisection.
double mass_at_factor(double factor) {
    double low = 0.01;
    double high = 100.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = std::sqrt(low * high);
        if (lag_at(mid).lag_factor < factor) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return std::sqrt(low * high);
}

// Steps per period of the critical mode: to leading order the root's phase per step is x / sqrt(1 - r).
int period_steps(const CarriedLoadOutput& lag) {
    return static_cast<int>(std::ceil(2.0 * std::numbers::pi * std::sqrt(1.0 - lag.lag_ratio) / lag.lag_omega_h));
}

// Deflection at midspan relative to the parent after a one-step 1 N pulse on the parent; the largest |u| of
// each window of `window` steps.
std::vector<double> envelope(double carriage_mass, const MultibodyConfig& config, int windows, int window) {
    const CarriedRig rig = free_rig(carriage_mass);
    MultibodyProvider provider(rig.model, config);
    State state = start_state(provider, rig.carriage, 0.5);
    SemiImplicitEuler integrator(StepConfig{step});
    provider.apply_body_wrench(rig.parent, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 1.0, 0.0));
    static_cast<void>(provider.step(integrator, state));
    std::vector<double> out;
    for (int w = 0; w < windows; ++w) {
        double largest = 0.0;
        for (int n = 0; n < window; ++n) {
            static_cast<void>(provider.step(integrator, state));
            const double parent = state.q[provider.position_offset(rig.parent)];
            largest = std::max(largest, std::abs(point_height(provider, state, 0.5) - parent));
        }
        out.push_back(largest);
    }
    return out;
}

}  // namespace

TEST_CASE("carried: the lag's growth factor against its closed forms", "[ENG-052]") {
    // r = 0, undamped: the cubic is z ((z - 1)^2 + x^2 z^2), backward Euler's |z| = 1 / sqrt(1 + x^2).
    for (const double x : {1e-3, 0.1, 1.0, 3.0}) {
        CAPTURE(x);
        CHECK(std::abs(lag_growth_factor(0.0, x, 0.0) - (1.0 / std::sqrt(1.0 + (x * x)))) <= 1e-12);
        // On the boundary a root is on the unit circle (cos theta = 1 / (1 + x^2)); 1e-9 for the eigen solve.
        const double boundary = lag_stability_boundary(x);
        CHECK(boundary == 0.5 * (1.0 + (x * x)));
        CHECK(std::abs(lag_growth_factor(boundary, x, 0.0) - 1.0) <= 1e-9);
        CHECK(lag_growth_factor(boundary - 1e-3, x, 0.0) < 1.0);
        CHECK(lag_growth_factor(boundary + 1e-3, x, 0.0) > 1.0);
        // Damping only adds decay: z^3 and z^2 terms grow with zeta.
        CHECK(lag_growth_factor(0.3, x, 0.5) < lag_growth_factor(0.3, x, 0.0));
    }
    // Leading order |z| - 1 = x^2 (2 r - 1) / (2 (1 - r)^2); the next term is O(x / sqrt(1 - r)) relative: at x = 1e-3,
    // r = 0.9 that is 3e-3, asserted within 1 %.
    const double x = 1e-3;
    const double r = 0.9;
    const double leading = x * x * (2.0 * r - 1.0) / (2.0 * (1.0 - r) * (1.0 - r));
    CHECK(std::abs((lag_growth_factor(r, x, 0.0) - 1.0) / leading - 1.0) <= 0.01);
    // One mode through the full linear step is the same cubic (the step's state (u, v, a_{n-1}) has three roots).
    for (const double zeta : {0.0, 0.2}) {
        const Eigen::MatrixXd lag = Eigen::MatrixXd::Constant(1, 1, 0.6);
        const Eigen::VectorXd omega_squared = Eigen::VectorXd::Constant(1, 150.0 * 150.0);
        const Eigen::MatrixXd damping = Eigen::MatrixXd::Constant(1, 1, 2.0 * zeta * 150.0);
        CHECK(std::abs(lag_spectral_radius(lag, omega_squared, damping, step) -
                       lag_growth_factor(0.6, 150.0 * step, zeta)) <= 1e-9);
    }
    // r = 0, x = 0: a double root at 1 (a rigid mode), the largest |z| exactly 1 up to the eigen solve.
    CHECK(std::abs(lag_growth_factor(0.0, 0.0, 0.0) - 1.0) <= 1e-7);
}

TEST_CASE("carried: a parent fixed across the member is feed-forward and publishes no lag", "[ENG-052]") {
    // A fixed parent (and the world) has no velocity across the member: J_P = 0, the inverse inertia is zero.
    for (const ParentKind kind : {ParentKind::World, ParentKind::Fixed}) {
        const CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 50.0, 0.0, true, kind, 0.0);
        MultibodyProvider provider(rig.model);
        State state = start_state(provider, rig.carriage, 0.5);
        run_steps(provider, state, 1, step);
        const CarriedLoadOutput& c = provider.outputs().carried.front();
        CHECK(c.inverse_inertia == 0.0);
        CHECK(c.lag_ratio == 0.0);
        CHECK(c.lag_factor == 0.0);
    }
}

TEST_CASE("carried: a free parent above the boundary is refused naming member, joint and r", "[ENG-052]") {
    const double mass = mass_at_factor(1.01);
    const CarriedRig rig = free_rig(mass);
    MultibodyProvider provider(rig.model, one_way());
    State state = start_state(provider, rig.carriage, 0.5);
    SemiImplicitEuler integrator(StepConfig{step});
    try {
        static_cast<void>(provider.step(integrator, state));
        FAIL("accepted");
    } catch (const InvalidModelError& error) {
        CAPTURE(error.cause());
        CHECK(error.cause().find("member 0") != std::string::npos);
        CHECK(error.cause().find("carried joint of body " + std::to_string(rig.carriage)) != std::string::npos);
        CHECK(error.cause().find("r = ") != std::string::npos);
        CHECK(error.cause().find("grows") != std::string::npos);
    }
    CHECK(state.t == 0.0);
}

TEST_CASE("carried: a free parent just below the boundary stays within its derived envelope", "[ENG-052]") {
    // Just below: the undamped factor 1 - 1e-4, so a thousand periods decay by about e^-3 and stay well above the
    // loop solve's residual.
    const double mass = mass_at_factor(1.0 - 1e-4);
    const CarriedLoadOutput lag = lag_at(mass);
    const double boundary = lag_stability_boundary(lag.lag_omega_h);
    const int period = period_steps(lag);
    const int windows = 1000;
    std::printf("carried below: m_c %.6g kg, inverse inertia %.6g, r %.6g, boundary %.6g, x %.6g, factor %.12g, "
                "period %d steps\n",
                mass, lag.inverse_inertia, lag.lag_ratio, boundary, lag.lag_omega_h, lag.lag_factor, period);
    REQUIRE(lag.lag_factor < 1.0);
    const std::vector<double> amplitude = envelope(mass, one_way(), windows, period);
    // Envelope, not a per-step inequality: window w starts (w - 1) periods after the end of window 0, so the
    // slowest eigenvector's amplitude there is at most A_0 factor^((w - 1) period), A_0 at most window 0's largest
    // sample divided by cos(theta / 2) (a sampled sinusoid with phase step theta = x / sqrt(1 - r) peaks within
    // theta / 2 of a sample). The faster eigenvectors start in window 0 and decay faster: they only raise window 0.
    const double theta = lag.lag_omega_h / std::sqrt(1.0 - lag.lag_ratio);
    const double start = amplitude.front() / std::cos(0.5 * theta);
    for (int w = 1; w < windows; ++w) {
        const double bound = start * std::pow(lag.lag_factor, static_cast<double>((w - 1) * period));
        const double measured = amplitude[static_cast<std::size_t>(w)];
        if (w % 100 == 0 || measured > bound) {
            std::printf("  window %d: %.6e, bound %.6e\n", w, measured, bound);
        }
        CHECK(measured <= bound);
    }
}

TEST_CASE("carried: above the boundary, unchecked, the member grows at the derived rate", "[ENG-052]") {
    // Undamped factor 1.01: 20 periods grow the 1 N pulse's response by about e^6, still a small deflection.
    const double mass = mass_at_factor(1.01);
    const CarriedLoadOutput lag = lag_at(mass);
    const int period = period_steps(lag);
    const int windows = 20;
    const std::vector<double> amplitude = envelope(mass, unchecked(), windows, period);
    const double predicted = std::log(lag.lag_factor);
    const int first = windows / 2;
    const double measured = std::log(amplitude[windows - 1] / amplitude[first]) /
                            static_cast<double>((windows - 1 - first) * period);
    std::printf("carried above: m_c %.6g kg, r %.6g, boundary %.6g, x %.6g, factor %.12g; growth per step "
                "predicted %.6e, measured %.6e; amplitude %.3e -> %.3e\n",
                mass, lag.lag_ratio, lag_stability_boundary(lag.lag_omega_h), lag.lag_omega_h, lag.lag_factor,
                predicted, measured, amplitude[first], amplitude[windows - 1]);
    CHECK(std::abs(measured / predicted - 1.0) <= 0.05);
}

TEST_CASE("carried: with the carried mass a free parent's lag stays below one half for any mass and decays",
          "[ENG-052]") {
    // member_loads.hpp, Stability, with the carried mass: |r| <= a b <= (a^2 + b^2) / 2 <= 1/2 on every mode of K and
    // H + B, so every published r is at most 1/2 (the bound, not a fit), below its boundary (1 + x^2) / 2, and the
    // coupled undamped step (lag_factor, no node damping here) stays below 1. Masses from 1/500 to 200 times the
    // parent's, through E7b's refusal at about one parent mass. A 20 kg carriage (four parent masses, E7b's growth
    // well past the boundary) then runs a hundred periods of the E7b envelope check (window w bounded by window 0
    // times factor^((w - 1) period), period from the root's phase).
    for (const double mass : {0.01, 1.0, 5.0, 20.0, 100.0, 1000.0}) {
        const CarriedLoadOutput lag = lag_at(mass, MultibodyConfig{});
        std::printf("carried mass %.6g kg: r %.6g, x %.6g, boundary %.6g, undamped factor %.12g\n", mass,
                    lag.lag_ratio, lag.lag_omega_h, lag_stability_boundary(lag.lag_omega_h), lag.lag_factor);
        CAPTURE(mass, lag.lag_ratio, lag.lag_factor);
        CHECK(lag.inverse_inertia > 0.0);
        CHECK(std::abs(lag.lag_ratio) <= 0.5);
        CHECK(lag.lag_ratio < lag_stability_boundary(lag.lag_omega_h));
        CHECK(lag.lag_factor < 1.0);
    }
    const double mass = 20.0;
    const CarriedLoadOutput lag = lag_at(mass, MultibodyConfig{});
    const int period = period_steps(lag);
    const int windows = 100;
    const std::vector<double> amplitude = envelope(mass, MultibodyConfig{}, windows, period);
    const double theta = lag.lag_omega_h / std::sqrt(1.0 - lag.lag_ratio);
    const double start = amplitude.front() / std::cos(0.5 * theta);
    for (int w = 1; w < windows; ++w) {
        const double bound = start * std::pow(lag.lag_factor, static_cast<double>((w - 1) * period));
        CHECK(amplitude[static_cast<std::size_t>(w)] <= bound);
    }
}
