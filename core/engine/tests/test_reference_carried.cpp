#include <Eigen/Core>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>

#include "beam_helpers.hpp"
#include "carried_helpers.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/member_loads.hpp"
#include "mechsim/engine/multibody_provider.hpp"

using mechsim::engine::BeamEnds;
using mechsim::engine::CarriedLoadOutput;
using mechsim::engine::MemberOutput;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::carried_rig;
using mechsim::engine::test::CarriedRig;
using mechsim::engine::test::flexural_rigidity;
using mechsim::engine::test::pinned_chain_compliance;
using mechsim::engine::test::point_height;
using mechsim::engine::test::settling_steps;
using mechsim::engine::test::start_state;

// Load-path coupling (ENG-052, ruling 63), static and quasi-static references. The carriage's load is P = 0.05 N
// along world -y (a constant body force on it, gravity off): the member's own weight would deflect it by
// 5 rho A g L^4 / (384 E I) = 0.6 mm, and geometric nonlinearity enters the static quantities at (delta / L)^2;
// with P the largest deflection is P L^3 / (48 E I) = 6.25e-6 m, delta / L <= 1e-5, so it stays below 1e-10, under
// every tolerance here. Settling: carried_helpers.hpp (40 beta after the load starts).
//
// Closed forms (Euler-Bernoulli, load P at a, b = L - a):
//   pinned-pinned deflection under the load P a^2 b^2 / (3 E I L); support shear P b / L (x = 0), P a / L (x = L);
//   clamped-clamped at midspan P L^3 / (192 E I), end moments P L / 8; cantilever with the load at the tip
//   P L^3 / (3 E I).
// The chain's own values (lumped_beam.hpp; the lever rule puts P on the two load nodes around a, and the member
// point at a is the same interpolation of the node points, so by reciprocity the deflection under the load is the
// unit-load sum over the springs with both moment diagrams from the lever-distributed load):
//   pinned-pinned, N = 8: a = L / 2 -1.5625 % (both load nodes at 3.5 l and 4.5 l, symmetric), a = L / 4 -2.083 %,
//   a = 0.1 L -3.067 %; N = 16: -0.391 %, -0.521 %, +0.911 % (pinned_chain_compliance, the same sum). The 2 % row
//   (docs/spec/eng.md, ENG-022) therefore holds at eight segments only at midspan: L / 4 and 0.1 L are asserted
//   against the row at sixteen segments and against the chain's own value at eight.
//   clamped-clamped, N = 16, a = L / 2 (load nodes 7.5 l and 8.5 l, half each): the far rotation lock sum M(x_j) = 0
//   gives the end moment -P L / 8 exactly and the midspan deflection P L^3 / (192 E I) (1 - 4 / N^2), -1.5625 %.
//   cantilever, N = 8, the load on the tip node (weight 1): P L^3 / (3 E I) (1 - 1 / (4 N^2)), -0.39 %.
// Support shear is statics: with weights w and 1 - w at x_k and x_{k+1}, the far support takes
// P (w x_k + (1 - w) x_{k+1}) / L = P a / L for every N; the tolerance 1e-9 relative (ENG-030's unit tolerance) holds
// once the transient is below it.

namespace {

constexpr double load = 0.05;
constexpr double row = 0.02;
constexpr double chain_tolerance = 1e-6;  // ruling 57: against the chain's own value
constexpr double statics_tolerance = 1e-9;

struct Settled {
    double deflection = 0.0;  // downward, at a, m
    MemberOutput member;
    CarriedLoadOutput carried;
};

Settled settle(BeamEnds ends, int segments, double a) {
    CarriedRig rig = carried_rig(ends, segments, 1.0, 0.0, true);
    rig.model.gravity = Eigen::Vector3d::Zero();
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, a);
    SemiImplicitEuler integrator(StepConfig{1e-3});
    const int steps = settling_steps(rig);
    for (int n = 0; n < steps; ++n) {
        provider.apply_body_wrench(rig.carriage, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, -load, 0.0));
        static_cast<void>(provider.step(integrator, state));
    }
    const State before = state;
    provider.apply_body_wrench(rig.carriage, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, -load, 0.0));
    static_cast<void>(provider.step(integrator, state));
    Settled out;
    out.deflection = -point_height(provider, before, a);
    out.member = provider.outputs().members.front();
    out.carried = provider.outputs().carried.front();
    return out;
}

double relative(double measured, double target) { return std::abs(measured / target - 1.0); }

}  // namespace

TEST_CASE("reference: a carried load on a pinned-pinned member deflects and loads its supports as the closed forms",
          "[ENG-052][ENG-022][ENG-030]") {
    const double ei = flexural_rigidity(mechsim::engine::test::steel_bar(1.0, BeamEnds::SimplySupported, 8));
    for (const double a : {0.5, 0.25, 0.1}) {
        for (const int segments : {8, 16}) {
            const Settled s = settle(BeamEnds::SimplySupported, segments, a);
            const double b = 1.0 - a;
            const double continuum = load * a * a * b * b / (3.0 * ei);
            const double chain = load * pinned_chain_compliance(segments, a, 1.0) / ei;
            std::printf("carried pinned-pinned N=%d a=%.2f: deflection %.9e, chain %.9e, continuum %.9e (%+.4f %%)\n",
                        segments, a, s.deflection, chain, continuum, 100.0 * (s.deflection / continuum - 1.0));
            std::printf("  shear start %.12e (P b / L %.12e), end %.12e (P a / L %.12e)\n", s.member.start_force.y(),
                        load * b, s.member.end_force.y(), load * a);
            CAPTURE(a, segments);
            CHECK(relative(s.deflection, chain) <= chain_tolerance);
            if (segments == 16 || a == 0.5) {
                CHECK(relative(s.deflection, continuum) <= row);
            }
            CHECK(relative(s.member.start_force.y(), load * b) <= statics_tolerance);
            CHECK(relative(s.member.end_force.y(), load * a) <= statics_tolerance);
            CHECK(relative(-s.carried.deflection, s.deflection) <= chain_tolerance);
            CHECK(s.carried.force == -load);
            CHECK(s.carried.position == a);
        }
    }
}

TEST_CASE("reference: a carried load at midspan of a clamped-clamped member, sixteen segments",
          "[ENG-052][ENG-022][ENG-030]") {
    const Settled s = settle(BeamEnds::Clamped, 16, 0.5);
    const double ei = flexural_rigidity(mechsim::engine::test::steel_bar(1.0, BeamEnds::Clamped, 16));
    const double continuum = load / (192.0 * ei);
    const double chain = continuum * (1.0 - 4.0 / 256.0);
    const double moment = load / 8.0;
    std::printf("carried clamped-clamped N=16: deflection %.9e, chain %.9e, continuum %.9e; end moments %.9e %.9e "
                "(P L / 8 %.9e)\n",
                s.deflection, chain, continuum, s.member.start_moment.z(), s.member.end_moment.z(), moment);
    CHECK(relative(s.deflection, continuum) <= row);
    CHECK(relative(s.deflection, chain) <= chain_tolerance);
    CHECK(relative(std::abs(s.member.start_moment.z()), moment) <= row);
    CHECK(relative(std::abs(s.member.end_moment.z()), moment) <= row);
    CHECK(relative(std::abs(s.member.start_moment.z()), moment) <= chain_tolerance);
    CHECK(relative(std::abs(s.member.end_moment.z()), moment) <= chain_tolerance);
}

TEST_CASE("reference: a carriage at the tip of a cantilever member", "[ENG-052][ENG-022]") {
    const Settled s = settle(BeamEnds::Cantilever, 8, 1.0);
    const double ei = flexural_rigidity(mechsim::engine::test::steel_bar(1.0, BeamEnds::Cantilever, 8));
    const double continuum = load / (3.0 * ei);
    const double chain = continuum * (1.0 - 1.0 / 256.0);
    std::printf("carried cantilever tip N=8: deflection %.9e, chain %.9e, continuum %.9e\n", s.deflection, chain,
                continuum);
    CHECK(relative(s.deflection, continuum) <= row);
    CHECK(relative(s.deflection, chain) <= chain_tolerance);
    CHECK(s.carried.weight == 0.0);
}

TEST_CASE("reference: an axial load through a locked carriage above the axis reaches the supports as a couple",
          "[ENG-052][ENG-013][ENG-030]") {
    // The carriage is locked at a = L / 4 (lower == upper, ENG-013), its origin d = 0.05 m above the member axis, and
    // pushed along +x by F = 1 N. Its load on the member is F x at height d: the axial force stays on the parent,
    // the couple about the foot point M_z = (d y) x (F x) . z = -d F is rerouted. A pinned-pinned member under a
    // couple M at any a has support shears R_L = -M / L at x = L (moments about the root: R_L L + M = 0) and
    // R_0 = -R_L = M / L at x = 0, here -d F / L at the root and +d F / L at the roller: statics, 1e-9 relative. The load reaches the carriage's joint
    // only through the lock rows, so this also shows the rerouted reaction carries the limit impulses.
    const double d = 0.05;
    const double push = 1.0;
    CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1.0, d, true);
    rig.model.gravity = Eigen::Vector3d::Zero();
    rig.model.bodies.back().joint.lower = Eigen::VectorXd::Constant(1, 0.25);
    rig.model.bodies.back().joint.upper = Eigen::VectorXd::Constant(1, 0.25);
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, 0.25);
    SemiImplicitEuler integrator(StepConfig{1e-3});
    const int steps = settling_steps(rig) + 1;
    for (int n = 0; n < steps; ++n) {
        provider.apply_body_wrench(rig.carriage, Eigen::Vector3d::Zero(), Eigen::Vector3d(push, 0.0, 0.0));
        static_cast<void>(provider.step(integrator, state));
    }
    const MemberOutput& member = provider.outputs().members.front();
    const CarriedLoadOutput& carried = provider.outputs().carried.front();
    const double shear = d * push;
    std::printf("carried couple: moment %.12e (-d F %.12e), shear start %.12e end %.12e (d F / L %.12e), axial %.3e\n",
                carried.moment, -shear, member.start_force.y(), member.end_force.y(), shear,
                member.start_force.x());
    CHECK(relative(carried.moment, -shear) <= statics_tolerance);
    CHECK(carried.force == 0.0);
    CHECK(relative(member.start_force.y(), -shear) <= statics_tolerance);
    CHECK(relative(member.end_force.y(), shear) <= statics_tolerance);
    CHECK(std::abs(member.start_force.x()) <= statics_tolerance * push);
}

TEST_CASE("reference: a carriage crossing a pinned-pinned member slowly follows the static deflection",
          "[ENG-052][ENG-022]") {
    // Quasi-static response. Each mode of the chain obeys q'' + beta omega^2 q' + omega^2 q = omega^2 q_s(t) (Rayleigh
    // damping, carried_helpers.hpp); backward Euler's steady response to a ramp q_s = r t is q = q_s - beta r exactly
    // (v = r in v' = v + h(-c v' - omega^2 (q' - q_s')) gives omega^2 (q - q_s) = -c r, c = beta omega^2), the same
    // beta in every mode, so the deflection under a load moving at speed v lags the chain's static value by
    //   beta v |d delta / da|  plus the curvature term v^2 |delta''(a)| / omega_1^2,
    // delta''(a) <= 2 P L / (3 E I) (the continuum's largest, at the ends; the chain's is the same piecewise). The
    // load's own value is constant (a body force on the carriage), so the one-step reaction lag adds nothing.
    // The slope is the chain's: finite differences of the unit-load sum on a 1 mm grid, the largest over the span
    // (a kink at each load node: the lever rule is piecewise linear in a, the chain's deflection piecewise quadratic).
    // Assert from a = 0.1 L (0.8 s, over 50 beta after the start, the start transient e^(-50)) to 0.9 L: the chain's
    // value within the dynamic term, the continuum within the 2 % row (N = 16, the chain within 1.45 % on [0.1 L,
    // 0.9 L]) plus the dynamic term. Node force jumps: node k carries w_k F_y; per step w changes by at most
    // v h / l_k (l_k >= l / 2, the end intervals), so no node's force may change by more than |F| v h / (l / 2) plus
    // the change of F (zero here) and round-off.
    const int segments = 16;
    const double speed = 0.1;
    const double h = 1e-3;
    CarriedRig rig = carried_rig(BeamEnds::SimplySupported, segments, 1.0, 0.0, true);
    rig.model.gravity = Eigen::Vector3d::Zero();
    const double ei = flexural_rigidity(rig.spec);
    const double omega = mechsim::engine::test::first_mode_factor(BeamEnds::SimplySupported) *
                         mechsim::engine::test::frequency_scale(rig.spec);
    double slope = 0.0;
    for (int i = 0; i < 1000; ++i) {
        const double x0 = i * 1e-3;
        const double x1 = x0 + 1e-3;
        const double d0 = pinned_chain_compliance(segments, x0, 1.0);
        const double d1 = pinned_chain_compliance(segments, x1, 1.0);
        slope = std::max(slope, std::abs(d1 - d0) / 1e-3);
    }
    slope *= load / ei;
    const double dynamic = (rig.beta * speed * slope) + (speed * speed * 2.0 * load / (3.0 * ei * omega * omega));
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, 0.02);
    state.v[provider.velocity_offset(rig.carriage)] = speed;
    SemiImplicitEuler integrator(StepConfig{h});
    const double l = 1.0 / segments;
    const double jump_bound = (load * speed * h / (0.5 * l)) * (1.0 + 1e-9);
    std::vector<double> previous;
    double worst_chain = 0.0;
    double worst_continuum = 0.0;
    double worst_jump = 0.0;
    while (state.q[provider.position_offset(rig.carriage)] < 0.9) {
        const State before = state;
        provider.apply_body_wrench(rig.carriage, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, -load, 0.0));
        static_cast<void>(provider.step(integrator, state));
        const CarriedLoadOutput& c = provider.outputs().carried.front();
        std::vector<double> nodes(static_cast<std::size_t>(segments) + 2, 0.0);
        nodes[static_cast<std::size_t>(c.node)] = c.weight * c.force;
        nodes[static_cast<std::size_t>(c.node) + 1] = (1.0 - c.weight) * c.force;
        if (!previous.empty()) {
            for (std::size_t k = 0; k < nodes.size(); ++k) {
                worst_jump = std::max(worst_jump, std::abs(nodes[k] - previous[k]));
            }
        }
        previous = nodes;
        const double a = before.q[provider.position_offset(rig.carriage)];
        if (a < 0.1) {
            continue;
        }
        const double deflection = -point_height(provider, before, a);
        const double chain = load * pinned_chain_compliance(segments, a, 1.0) / ei;
        const double b = 1.0 - a;
        const double continuum = load * a * a * b * b / (3.0 * ei);
        worst_chain = std::max(worst_chain, std::abs(deflection - chain));
        worst_continuum = std::max(worst_continuum, std::abs(deflection - continuum) - (row * continuum));
    }
    std::printf("carried moving N=16 v=%.2f: worst |delta - chain| %.3e (dynamic term %.3e), worst excess over the "
                "row %.3e, worst node jump %.3e (bound %.3e)\n",
                speed, worst_chain, dynamic, worst_continuum, worst_jump, jump_bound);
    CHECK(worst_chain <= dynamic);
    CHECK(worst_continuum <= dynamic);
    CHECK(worst_jump <= jump_bound);
}
