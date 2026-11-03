#include <Eigen/Core>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <utility>
#include <vector>

#include "beam_helpers.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/multibody_provider.hpp"

using mechsim::engine::BeamEnds;
using mechsim::engine::BeamPoint;
using mechsim::engine::BeamSpec;
using mechsim::engine::LumpedBeam;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::apply_point_force;
using mechsim::engine::test::flexural_rigidity;
using mechsim::engine::test::frequency_scale;
using mechsim::engine::test::lumped_first_frequency;
using mechsim::engine::test::steel_bar;
using mechsim::engine::test::upward_crossings;
using mechsim::engine::test::world_point;

// Closed forms and the chain's own derived values are in mechsim/engine/lumped_beam.hpp. Tolerances: 2 % of
// deflection, 3 % of frequency (docs/spec/eng.md, ADR 0013).

namespace {

constexpr double deflection_tolerance = 0.02;           // ENG-038, ENG-040
constexpr double frequency_tolerance = 0.03;            // ENG-039, ENG-041
constexpr double cantilever_root = 1.8751040687119611;  // beta_1 L, first root of cos(x) cosh(x) = -1
constexpr double cantilever_length = 1.0;               // m
constexpr double supported_length = 1.5;                // m

struct Built {
    MultibodyModel model;
    LumpedBeam beam;
};

Built build(BeamSpec spec) {
    Built out;
    out.model.gravity = Eigen::Vector3d::Zero();
    out.beam = add_lumped_beam(out.model, spec);
    return out;
}

struct Settled {
    double deflection = 0.0;   // world y of the point, m
    double last_change = 0.0;  // |y change| over the final 100 steps, m
};

// Static load held every step; node damping D = (2 / omega_c) K gives every mode zeta_i = omega_i / omega_c, about
// critical for the first and overdamped above it, so all modes decay at least at omega_c / 2. 3000 steps of
// 1 ms at omega_c >= 50 rad/s leave exp(-75) of the transient. The fixed point of the step has f = c, so the
// settled state is the chain's static solution.
Settled settle(BeamSpec spec, double closed_form_omega, double x_load, double force) {
    spec.node_damping = 2.0 / closed_form_omega * flexural_rigidity(spec) / (spec.length / spec.segments);
    const Built built = build(spec);
    MultibodyProvider provider(built.model);
    SemiImplicitEuler integrator(StepConfig{});
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    const BeamPoint point = beam_point(built.beam, x_load);
    const int steps = 3000;
    double marker = 0.0;
    for (int n = 0; n < steps; ++n) {
        if (n == steps - 100) {
            marker = world_point(provider, state, point).y();
        }
        apply_point_force(provider, state, point, Eigen::Vector3d(0.0, force, 0.0));
        provider.step(integrator, state);
    }
    Settled out;
    out.deflection = world_point(provider, state, point).y();
    out.last_change = std::abs(out.deflection - marker);
    return out;
}

// Upward crossings from the given initial shape, undamped; returns the measured first-mode frequency, rad/s. The
// first two crossings are dropped. A mode at omega_k contracts by (1 + (omega_k h)^2)^(-1/2) per step under the
// implicit step. Cantilever: mode 2 (omega_2 about 6.27 omega_1, omega_2 h about 0.32) falls below 1e-4 of its start
// within two periods of mode one. Pinned span: the symmetric midspan-load shape has zero content in the
// antisymmetric mode 2 (omega_2 = 4 omega_1), and mode 3 (omega_3 = 9 omega_1, omega_3 h about 0.58) falls below 1e-4
// within about 65 steps. Later crossings carry the first mode alone.
double measured_frequency(const Built& built, Eigen::VectorXd q, double x_measure, double lumped_omega, int periods) {
    MultibodyProvider provider(built.model);
    SemiImplicitEuler integrator(StepConfig{});
    State state{std::move(q), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    provider.project_positions(0.0, state.q, state.v);
    const double h = integrator.config().h;
    const int steps = static_cast<int>(std::ceil((periods + 3.0) * 2.0 * std::numbers::pi / lumped_omega / h));
    const std::vector<double> crossings =
        upward_crossings(provider, integrator, state, beam_point(built.beam, x_measure), steps);
    constexpr std::size_t dropped = 2;
    REQUIRE(crossings.size() >= static_cast<std::size_t>(periods) + dropped);
    const auto counted = static_cast<double>(crossings.size() - dropped - 1);
    return 2.0 * std::numbers::pi * counted / (crossings.back() - crossings[dropped]);
}

// K implicit, no damping: the step rotates each mode by atan(omega h) (package 1), so the simulated frequency is
// atan(omega h) / h, low by about (omega h)^2 / 3. Asserted within 10 % of that correction, as in packages 1 and 2a.
void check_dispersion(double measured, double lumped_omega) {
    const double h = StepConfig{}.h;
    const double omega_h = lumped_omega * h;
    const double expected = std::atan(omega_h) / h;
    const double correction = omega_h * omega_h / 3.0;
    CAPTURE(measured, expected, correction, measured / lumped_omega - 1.0);
    CHECK(std::abs(measured / expected - 1.0) <= 0.1 * correction);
}

}  // namespace

TEST_CASE("cantilever tip deflection under a static tip load matches P L^3 / (3 E I)", "[ENG-038][reference]") {
    // Chain value P L^3 / (3 E I) (1 - 1 / (4 N^2)): N = 4 is -1.5625 %, N = 8 is -0.390625 %. The load gives a
    // tip deflection of 1e-4 L, so geometric nonlinearity is O(1e-8) and the chain value holds to 1e-6.
    const int segments = GENERATE(4, 8);
    const BeamSpec spec = steel_bar(cantilever_length, BeamEnds::Cantilever, segments);
    const double length = spec.length;
    const double load = 3.0 * flexural_rigidity(spec) * 1e-4 / (length * length);
    const double closed_form = load * length * length * length / (3.0 * flexural_rigidity(spec));
    const double chain = closed_form * (1.0 - 1.0 / (4.0 * segments * segments));
    const double omega_c = cantilever_root * cantilever_root * frequency_scale(spec);
    const Settled settled = settle(spec, omega_c, length, -load);
    const double measured = -settled.deflection;
    CAPTURE(segments, measured, closed_form, chain, measured / closed_form - 1.0);
    WARN("ENG-038 N = " << segments << ": closed form " << closed_form << " m, measured " << measured << " m, error "
                        << measured / closed_form - 1.0);
    CHECK(settled.last_change <= 1e-9 * closed_form);
    CHECK(std::abs(measured / chain - 1.0) <= 1e-6);
    CHECK(std::abs(measured / closed_form - 1.0) <= deflection_tolerance);
}

struct Midspan {
    double closed_form = 0.0;  // P L^3 / (48 E I), m
    double chain = 0.0;        // closed_form (1 - 1 / N^2), m
    double measured = 0.0;     // m
    double last_change = 0.0;  // m
};

Midspan midspan_deflection(int segments) {
    const BeamSpec spec = steel_bar(supported_length, BeamEnds::SimplySupported, segments);
    const double length = spec.length;
    const double load = 48.0 * flexural_rigidity(spec) * 1e-4 / (length * length);
    Midspan out;
    out.closed_form = load * length * length * length / (48.0 * flexural_rigidity(spec));
    out.chain = out.closed_form * (1.0 - 1.0 / (segments * segments));
    const double omega_c = std::numbers::pi * std::numbers::pi * frequency_scale(spec);
    const Settled settled = settle(spec, omega_c, 0.5 * length, -load);
    out.measured = -settled.deflection;
    out.last_change = settled.last_change;
    return out;
}

TEST_CASE("simply supported midspan deflection at N = 8 matches P L^3 / (48 E I)", "[ENG-040][reference]") {
    // Chain value P L^3 / (48 E I) (1 - 1 / N^2) for even N, intrinsic to the midpoint-spring chain: N = 4 is
    // -6.25 %, outside the 2 % tolerance, so pinned-pinned members and this reference run at N = 8 (-1.5625 %);
    // refining the segmentation is the remedy ADR 0013 names before a higher integrator tier (docs/spec/eng.md).
    const Midspan result = midspan_deflection(8);
    CAPTURE(result.measured, result.closed_form, result.chain);
    WARN("ENG-040 N = 8: closed form " << result.closed_form << " m, measured " << result.measured << " m, error "
                                       << result.measured / result.closed_form - 1.0);
    CHECK(result.last_change <= 1e-9 * result.closed_form);
    CHECK(std::abs(result.measured / result.chain - 1.0) <= 1e-6);
    CHECK(std::abs(result.measured / result.closed_form - 1.0) <= deflection_tolerance);
}

TEST_CASE("simply supported midspan deflection at N = 4 equals the chain value (1 - 1 / 16) P L^3 / (48 E I)",
          "[ENG-040][reference]") {
    // Consistency check only: the N = 4 chain is -6.25 % by derivation and is not held to the 2 % tolerance.
    const Midspan result = midspan_deflection(4);
    CAPTURE(result.measured, result.closed_form, result.chain);
    WARN("ENG-040 N = 4: closed form " << result.closed_form << " m, chain " << result.chain << " m, measured "
                                       << result.measured << " m");
    CHECK(result.last_change <= 1e-9 * result.closed_form);
    CHECK(std::abs(result.measured / result.chain - 1.0) <= 1e-6);
}

TEST_CASE("cantilever first mode matches (1.875^2 / (2 pi)) sqrt(E I / (rho A L^4))", "[ENG-039][reference]") {
    // Lumped eigenfrequency from K phi = omega^2 H phi, then the simulated tip frequency from a static-shape
    // deflection of 1e-3 L (higher-mode content decays under the implicit step) against atan(omega h) / h.
    const int segments = GENERATE(4, 8);
    const BeamSpec spec = steel_bar(cantilever_length, BeamEnds::Cantilever, segments);
    const Built built = build(spec);
    const double omega_c = cantilever_root * cantilever_root * frequency_scale(spec);
    const double lumped = lumped_first_frequency(MultibodyProvider(built.model));
    CAPTURE(segments, lumped, omega_c, lumped / omega_c - 1.0);
    WARN("ENG-039 N = " << segments << ": closed form " << omega_c << " rad/s, lumped " << lumped << " rad/s, error "
                        << lumped / omega_c - 1.0);
    CHECK(std::abs(lumped / omega_c - 1.0) <= frequency_tolerance);

    const double length = spec.length;
    const double load = 3.0 * flexural_rigidity(spec) * 1e-3 / (length * length);
    const MultibodyProvider shape(built.model);
    Eigen::VectorXd q = shape.neutral_positions();
    for (std::size_t j = 0; j < built.beam.bodies.size(); ++j) {
        q[shape.position_offset(built.beam.bodies[j])] =
            -load * (length - built.beam.starts[j]) / built.beam.node_stiffness;
    }
    const double measured = measured_frequency(built, q, length, lumped, 12);
    check_dispersion(measured, lumped);
    WARN("ENG-039 N = " << segments << ": closed form " << omega_c << " rad/s, simulated " << measured
                        << " rad/s, error " << measured / omega_c - 1.0);
    CHECK(std::abs(measured / omega_c - 1.0) <= frequency_tolerance);
}

TEST_CASE("simply supported first mode matches (pi / (2 L^2)) sqrt(E I / (rho A))", "[ENG-041][reference]") {
    // As ENG-039 with the eigenproblem on the null space of the roller constraint; the initial shape is the
    // linearized static midspan-load shape with the pin angle closing the far end, projected onto the constraint.
    const int segments = GENERATE(4, 8);
    const BeamSpec spec = steel_bar(supported_length, BeamEnds::SimplySupported, segments);
    const Built built = build(spec);
    const double omega_c = std::numbers::pi * std::numbers::pi * frequency_scale(spec);
    const double lumped = lumped_first_frequency(MultibodyProvider(built.model));
    CAPTURE(segments, lumped, omega_c, lumped / omega_c - 1.0);
    WARN("ENG-041 N = " << segments << ": closed form " << omega_c << " rad/s, lumped " << lumped << " rad/s, error "
                        << lumped / omega_c - 1.0);
    CHECK(std::abs(lumped / omega_c - 1.0) <= frequency_tolerance);

    const double length = spec.length;
    const double load = 48.0 * flexural_rigidity(spec) * 1e-3 / (length * length);
    const MultibodyProvider shape(built.model);
    Eigen::VectorXd q = shape.neutral_positions();
    double end_offset = 0.0;
    for (std::size_t j = 1; j < built.beam.bodies.size(); ++j) {
        const double x = built.beam.starts[j];
        const double angle = -0.5 * load * std::min(x, length - x) / built.beam.node_stiffness;
        q[shape.position_offset(built.beam.bodies[j])] = angle;
        end_offset += angle * (length - x);
    }
    q[shape.position_offset(built.beam.bodies[0])] = -end_offset / length;
    const double measured = measured_frequency(built, q, 0.5 * length, lumped, 12);
    check_dispersion(measured, lumped);
    WARN("ENG-041 N = " << segments << ": closed form " << omega_c << " rad/s, simulated " << measured
                        << " rad/s, error " << measured / omega_c - 1.0);
    CHECK(std::abs(measured / omega_c - 1.0) <= frequency_tolerance);
}
