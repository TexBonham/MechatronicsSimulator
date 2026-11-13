#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

#include "beam_helpers.hpp"
#include "carried_helpers.hpp"
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
