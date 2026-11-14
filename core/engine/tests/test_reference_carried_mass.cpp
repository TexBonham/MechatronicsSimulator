#include <Eigen/Core>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

#include "beam_helpers.hpp"
#include "carried_helpers.hpp"
#include "energy_helpers.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/multibody_provider.hpp"

using mechsim::engine::BeamEnds;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::carried_rig;
using mechsim::engine::test::CarriedRig;
using mechsim::engine::test::effective_mass_share;
using mechsim::engine::test::flexural_rigidity;
using mechsim::engine::test::mechanical_energy;
using mechsim::engine::test::ParentKind;
using mechsim::engine::test::point_height;
using mechsim::engine::test::point_stiffness_factor;
using mechsim::engine::test::start_state;

// A carried mass loads the member's inertia (ENG-052, brief M3-E7d; member_loads.hpp, Carried mass).
//
// Oracle: one mode, the static deflection shape (Rayleigh). A point mass m_c at the point of static stiffness k on a
// beam of mass m_b: with the shape psi(x) of the deflection under a point load there, normalised to 1 at the point,
// T = (1/2) u'^2 (m_c + f m_b), f = integral of psi^2 dm / m_b, and omega^2 = k / (m_c + f m_b). The integrals:
//   pinned-pinned, centre load: psi = 3 s - 4 s^3 (s = x / L <= 1/2, mirrored), f = 2 [3 s^3 - 24 s^5 / 5 +
//     16 s^7 / 7] at 1/2 = 17/35 = 0.4857, k = 48 E I / L^3;
//   cantilever, tip load: psi = (3 s^2 - s^3) / 2, f = (9/5 - 1 + 1/7) / 4 = 33/140 = 0.2357, k = 3 E I / L^3;
//   clamped-clamped, centre load: psi = 3 (2 s)^2 - 2 (2 s)^3 (s <= 1/2, mirrored), f = 2 [144 s^5 / 5 - 64 s^6 +
//     256 s^7 / 7] at 1/2 = 13/35 = 0.3714, k = 192 E I / L^3
// (the standard one-mode results, e.g. Thomson, Theory of Vibration, Rayleigh's method). Error of the estimate: an
// upper bound on omega, exact as m_c / m_b grows; at m_c = 0 the continuum's exact first mode is lower by 0.7 %
// (pinned, (98.8 / 97.4)^(1/2)), 1.5 % (cantilever, 12.73 against 12.36) and 1.6 % (clamped, 517 against 501). The
// chain adds its own static error (ENG-022: midspan pinned -1.6 % in deflection at N = 8, cantilever tip
// -0.4 %, clamped -6.25 % at N = 8, -1.6 % at N = 16: +0.8 %, +0.2 %, +3.3 %, +0.8 % in omega at large m_c) and its
// first-mode error at m_c = 0 (within 2.5 %, lumped_beam.hpp). The 2 % of the brief holds for pinned-pinned and the
// cantilever at 8 and 16 segments; the clamped chain runs at 16 only (ENG-022: its 8-segment midspan stiffness alone
// is 3.3 % high in omega, the reason the clamped references run at 16).
//
// Measurement: the member starts straight under gravity (undamped), so it rings about its static deflection; the
// deflection rate at the carried point changes sign every half period. Zero crossings of the rate (linear
// interpolation) from the third period on, over at least twenty periods. Backward Euler's root 1 / (1 - i x) turns
// by atan(x) per step: the measured frequency is omega atan(x) / x; at x = omega h <= 0.042 that is within 0.06 % of
// omega, inside the 0.1 % the brief allows (h from the estimate, omega within a few percent of it).

namespace {

// Gravity scaled down so the ringing stays linear: the largest static deflection here, the cantilever's tip under
// ten member masses, m g L^3 / (3 E I) = 0.15 m at full gravity, is 0.15 mm (1.5e-4 L) at 1e-3 g (at full gravity
// that rig, rotating its tip through about 0.2 rad, reads 4 % high: a geometric nonlinearity, outside this oracle).
constexpr double small_amplitude = 1e-3;

struct Ringing {
    double measured = 0.0;  // rad/s
    double estimate = 0.0;  // rad/s
    double x = 0.0;
    int half_periods = 0;
};

Ringing ring(BeamEnds ends, int segments, double ratio) {
    const double member = 7850.0 * 0.01 * 0.01 * 1.0;
    const double carriage_mass = ratio * member;
    const bool carry = ratio > 0.0;
    double mass = carriage_mass;
    if (!carry) {
        mass = 1.0;
    }
    CarriedRig rig = carried_rig(ends, segments, mass, 0.0, carry, ParentKind::World, 0.0);
    rig.model.gravity *= small_amplitude;
    double foot = 0.5;
    if (ends == BeamEnds::Cantilever) {
        foot = 1.0;
    }
    const double k = point_stiffness_factor(ends) * flexural_rigidity(rig.spec);
    Ringing out;
    out.estimate = std::sqrt(k / (carriage_mass + (effective_mass_share(ends) * member)));
    const double h = 0.04 / out.estimate;
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, foot);
    SemiImplicitEuler integrator(StepConfig{h});
    const double period = 2.0 * std::numbers::pi / out.estimate;
    const auto steps = static_cast<int>(std::ceil(24.0 * period / h));
    std::vector<double> crossings;
    double previous = point_height(provider, state, foot);
    double rate = 0.0;
    for (int n = 0; n < steps; ++n) {
        static_cast<void>(provider.step(integrator, state));
        const double height = point_height(provider, state, foot);
        const double next = height - previous;
        previous = height;
        const double t = state.t - (0.5 * h);
        if (n > 0 && t > 2.0 * period && ((rate < 0.0 && next >= 0.0) || (rate > 0.0 && next <= 0.0))) {
            crossings.push_back(t - (h * next / (next - rate)));
        }
        rate = next;
    }
    out.half_periods = static_cast<int>(crossings.size()) - 1;
    out.measured = std::numbers::pi * out.half_periods / (crossings.back() - crossings.front());
    out.x = out.measured * h;
    return out;
}

}  // namespace

TEST_CASE("reference: a carried mass lowers the member's first frequency to k / (m_c + f m_beam)",
          "[ENG-052][ENG-022]") {
    struct Case {
        BeamEnds ends;
        int segments;
        const char* name;
    };
    const std::vector<Case> cases{{BeamEnds::SimplySupported, 8, "pinned-pinned"},
                                  {BeamEnds::SimplySupported, 16, "pinned-pinned"},
                                  {BeamEnds::Cantilever, 8, "cantilever"},
                                  {BeamEnds::Cantilever, 16, "cantilever"},
                                  {BeamEnds::Clamped, 16, "clamped-clamped"}};
    for (const Case& c : cases) {
        for (const double ratio : {0.0, 1.0, 10.0}) {
            const Ringing r = ring(c.ends, c.segments, ratio);
            std::printf("carried mass %s N = %d, m_c / m_beam = %.0f: closed form %.6f rad/s, measured %.6f rad/s "
                        "(%+.3f %%) over %d half periods, x = %.4f, atan(x) / x = %.6f\n",
                        c.name, c.segments, ratio, r.estimate, r.measured, 100.0 * (r.measured / r.estimate - 1.0),
                        r.half_periods, r.x, std::atan(r.x) / r.x);
            CAPTURE(c.name, c.segments, ratio, r.estimate, r.measured);
            CHECK(r.half_periods >= 40);
            CHECK(std::abs((std::atan(r.x) / r.x) - 1.0) <= 1e-3);
            CHECK(std::abs(r.measured / r.estimate - 1.0) <= 0.02);
        }
    }
}

// The momentum form (ENG-052, brief M3-E7e; member_loads.hpp, Carried mass): a carriage coasting from a support to
// mid-span while the member rings in its first mode.
//
// Oracle. One mode u with modal mass m(s) and stiffness k, s(t) = c t prescribed (the carriage coasts at c: nothing
// acts on it along the member, gravity off): L = (1/2) m u'^2 - (1/2) k u^2, d/dt(m u') + k u = 0, and the action
// J = E / omega, E = (1/2) m u'^2 + (1/2) k u^2, omega = sqrt(k / m), obeys dJ/dtheta = (m' / (2 m omega)) J cos 2 theta
// (m' = dm/dt): J oscillates with relative amplitude eps = abs(m') / (4 m omega) about a mean that does not drift at
// first order (the adiabatic invariant). With E = (1/2) k A^2 at the midspan amplitude A, A^2 omega is constant:
// A goes as (M_eff)^(-1/4), M_eff = k / omega^2 the loaded modal mass at midspan. (The velocity form m u'' + k u = 0
// conserves A^2 / omega instead: A as M_eff^(+1/4).)
// The chain's m(s): f m_b + m_c l(s)^2, f = 17/35, l the lever-weighted interpolation of the mode sin(pi x / L)
// (normalised at midspan) between the load nodes (0 and (j + 1/2) L / 8): the carriage's block m_c G^T G in one
// mode. l is piecewise linear, so m' jumps at each node crossed, by c m_c 2 l abs(dl'); a jump in the oscillation's
// coefficient moves J's mean by at most the jump in eps, dJ / J <= c m_c 2 l abs(dl') / (4 m omega) per node.
// Tolerance on the action ratio: 2 eps_max (the two ends' oscillation; each end is averaged over one period, but
// bounded, not averaged out) + the sum of the node jumps, both from the one-mode m(s) with k = 48 E I / L^3 (10 %
// slack: the one-mode omega is within 2 % of the chain's, ENG-052 row), + x_max (backward Euler's energy differs
// from its decaying quadratic form by O(x) within a period).
// Scheme. Backward Euler on a mode: z = (1 + i x) / (1 + x^2), the energy falls by (1 + x^2) per step; the oracle
// divides it out: S = prod (1 + x_n^2)^-1 between the two windows' centres, x_n = omega(s_n) h, omega the chain's own
// loaded first frequency (lumped_first_frequency at the carriage's position, a grid of 81 points, linear between).
// The raw ratio is printed beside it.
// Amplitude. A from the midspan deflection's mean square over the window (A^2 = 2 mean u^2), against
// (M_b / M_a)^(-1/4) S^(1/2) with the one-mode M_eff: tolerance half the action's plus 4 % (the one-mode M_eff and
// the chain's frequency and mode at midspan each within 2 % in omega, ENG-052 row; A goes as their square root).
// Energy (the m_c = m_b run). E = (1/2) v^T (H + B) v + V changes by the work of the prescribed travel W = sum
// -(1/2) v_n^T (B_n - B_{n-1}) v_n less backward Euler's loss D = sum (1/2) dv^T (H + B) dv + (1/2) h^2 v_{n+1}^T K
// v_{n+1} (the step's exact energy balance with the block changing: (H + B_n)(v_{n+1} - v_n) = -h K q_{n+1} -
// (B_n - B_{n-1}) v_n gives E_{n+1} - E_n = -(1/2) v_n^T dB v_n - dv^T dB v_n - D), so the residual is at most
// sum abs(dv^T dB v_n), B_{n-1} here at the previous configuration (the engine's at today's Jacobians, a difference
// of second order in the member's rotation, at 1e-4 m amplitude inside 1e-6 of E).

namespace {

struct Adiabatic {
    double action = 0.0;     // J_b / (J_a S)
    double raw = 0.0;        // J_b / J_a
    double scheme = 0.0;     // S
    double amplitude = 0.0;  // A_b / A_a
    double predicted = 0.0;  // (M_b / M_a)^(-1/4) S^(1/2)
    double velocity_form = 0.0;
    double tolerance = 0.0;
    double energy_residual = 0.0;  // abs(E_N - E_0 - (W - D))
    double energy_bound = 0.0;
    double work = 0.0;
    double loss = 0.0;
};

Adiabatic coast(double ratio, double speed, double h, bool energy) {
    const double member = 7850.0 * 0.01 * 0.01 * 1.0;
    CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, ratio * member, 0.0, true, ParentKind::World, 0.0);
    rig.model.gravity = Eigen::Vector3d::Zero();
    const double carried = ratio * member;
    MultibodyProvider provider(rig.model);
    const Eigen::Index slot = provider.position_offset(rig.carriage);
    const auto omega_grid = [&]() {
        std::vector<double> out;
        for (int j = 0; j <= 80; ++j) {
            Eigen::VectorXd q = provider.neutral_positions();
            q[slot] = 0.5 * j / 80.0;
            out.push_back(mechsim::engine::test::lumped_first_frequency(provider, q));
        }
        return out;
    }();
    const auto omega_at = [&](double s) {
        const double u = std::clamp(s / 0.5 * 80.0, 0.0, 80.0);
        const auto j = static_cast<std::size_t>(std::min(std::floor(u), 79.0));
        const double t = u - static_cast<double>(j);
        return ((1.0 - t) * omega_grid[j]) + (t * omega_grid[j + 1]);
    };
    const double k = 48.0 * flexural_rigidity(rig.spec);
    const std::vector<double> nodes{0.0, 0.0625, 0.1875, 0.3125, 0.4375, 0.5625};
    const auto lever = [&](double s, double& slope) {
        std::size_t j = 0;
        while (j + 2 < nodes.size() && nodes[j + 1] <= s) {
            ++j;
        }
        const double a = std::sin(std::numbers::pi * nodes[j]);
        const double b = std::sin(std::numbers::pi * nodes[j + 1]);
        slope = (b - a) / (nodes[j + 1] - nodes[j]);
        return a + (slope * (s - nodes[j]));
    };
    const auto modal_mass = [&](double s) {
        double slope = 0.0;
        const double l = lever(s, slope);
        return ((17.0 / 35.0) * member) + (carried * l * l);
    };
    double eps = 0.0;
    for (int j = 0; j <= 5000; ++j) {
        const double s = 0.5 * j / 5000.0;
        double slope = 0.0;
        const double l = lever(s, slope);
        const double m = modal_mass(s);
        eps = std::max(eps, speed * carried * 2.0 * std::abs(l * slope) / (4.0 * m * std::sqrt(k / m)));
    }
    double jumps = 0.0;
    for (std::size_t j = 1; j + 1 < nodes.size(); ++j) {
        double below = 0.0;
        double above = 0.0;
        static_cast<void>(lever(nodes[j] - 1e-9, below));
        const double l = lever(nodes[j], above);
        const double m = modal_mass(nodes[j]);
        jumps += speed * carried * 2.0 * l * std::abs(above - below) / (4.0 * m * std::sqrt(k / m));
    }

    State state = start_state(provider, rig.carriage, 0.0);
    const mechsim::engine::test::FirstMode mode = mechsim::engine::test::lumped_first_mode(provider, state.q);
    state.v = 0.01 * mode.shape;
    state.v[provider.velocity_offset(rig.carriage)] = speed;
    const auto steps = static_cast<int>(std::lround(0.5 / speed / h));
    const auto first = static_cast<int>(std::lround(2.0 * std::numbers::pi / omega_at(0.0) / h));
    const auto last = static_cast<int>(std::lround(2.0 * std::numbers::pi / omega_at(0.5) / h));
    const int centre_a = first / 2;
    const int centre_b = steps - (last / 2);
    const double carriage_energy = 0.5 * carried * speed * speed;
    const auto member_energy = [&](const State& at) {
        return mechanical_energy(provider, at) - carriage_energy;
    };
    SemiImplicitEuler integrator(StepConfig{h});
    double energy_a = 0.0;
    double energy_b = 0.0;
    double square_a = 0.0;
    double square_b = 0.0;
    double log_scheme = 0.0;
    double x_max = 0.0;
    Adiabatic out;
    const double start_energy = mechanical_energy(provider, state);
    Eigen::MatrixXd previous_block = provider.carried_mass_matrix(state.q);
    for (int n = 0; n < steps; ++n) {
        const double s = state.q[slot];
        const double x = omega_at(s) * h;
        x_max = std::max(x_max, x);
        if (n >= centre_a && n < centre_b) {
            log_scheme += std::log1p(x * x);
        }
        mechsim::engine::DynamicsTerms terms;
        Eigen::MatrixXd block;
        const State before = state;
        if (energy) {
            provider.evaluate(state.t, state.q, state.v, terms);
            block = provider.carried_mass_matrix(state.q);
        }
        static_cast<void>(provider.step(integrator, state));
        if (energy) {
            const Eigen::VectorXd dv = state.v - before.v;
            const Eigen::MatrixXd change = block - previous_block;
            out.loss += (0.5 * dv.dot(terms.mass * dv)) + (0.5 * h * h * state.v.dot(terms.stiffness * state.v));
            out.work += -0.5 * before.v.dot(change * before.v);
            out.energy_bound += std::abs(dv.dot(change * before.v));
            previous_block = block;
        }
        const double height = point_height(provider, state, 0.5);
        if (n < first) {
            energy_a += member_energy(state) / first;
            square_a += height * height / first;
        }
        if (n >= steps - last) {
            energy_b += member_energy(state) / last;
            square_b += height * height / last;
        }
    }
    const double s_a = speed * h * centre_a;
    const double s_b = speed * h * centre_b;
    out.scheme = std::exp(-log_scheme);
    out.raw = (energy_b / omega_at(s_b)) / (energy_a / omega_at(s_a));
    out.action = out.raw / out.scheme;
    out.amplitude = std::sqrt(square_b / square_a);
    const double mass_ratio = modal_mass(s_b) / modal_mass(s_a);
    out.predicted = std::pow(mass_ratio, -0.25) * std::sqrt(out.scheme);
    out.velocity_form = std::pow(mass_ratio, 0.25) * std::sqrt(out.scheme);
    out.tolerance = (1.1 * ((2.0 * eps) + jumps)) + x_max;
    if (energy) {
        const double end_energy = mechanical_energy(provider, state);
        out.energy_residual = std::abs(end_energy - start_energy - (out.work - out.loss));
        out.energy_bound += 1e-6 * start_energy;
    }
    return out;
}

}  // namespace

TEST_CASE("reference: a carriage coasting to mid-span keeps the member's action E / omega; the amplitude goes as "
          "M_eff^(-1/4)",
          "[ENG-052]") {
    struct Case {
        double ratio;
        double speed;  // m/s
        double h;      // s
        bool energy;
    };
    for (const Case& c : {Case{1.0, 0.5, 1e-4, true}, Case{10.0, 0.2, 2e-4, false}}) {
        const Adiabatic run = coast(c.ratio, c.speed, c.h, c.energy);
        std::printf("adiabatic m_c / m_b = %.0f at %.2f m/s: action ratio %.6f (raw %.6f, scheme S %.6f) against 1 "
                    "within %.4f; amplitude ratio %.6f against M_eff^(-1/4) S^(1/2) = %.6f (velocity form %.6f) within "
                    "%.4f\n",
                    c.ratio, c.speed, run.action, run.raw, run.scheme, run.tolerance, run.amplitude, run.predicted,
                    run.velocity_form, (run.tolerance / 2.0) + 0.04);
        CHECK(std::abs(run.action - 1.0) <= run.tolerance);
        CHECK(std::abs((run.amplitude / run.predicted) - 1.0) <= (run.tolerance / 2.0) + 0.04);
        if (c.energy) {
            std::printf("adiabatic energy: work of the travel W %.6e J, scheme loss D %.6e J, residual %.3e J within "
                        "%.3e J\n",
                        run.work, run.loss, run.energy_residual, run.energy_bound);
            CHECK(run.work < 0.0);
            CHECK(run.energy_residual <= run.energy_bound);
        }
    }
}
