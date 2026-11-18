#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <cmath>
#include <cstdio>
#include <numbers>

#include "beam_helpers.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/member_loads.hpp"
#include "mechsim/engine/multibody_provider.hpp"

using mechsim::engine::BeamEnds;
using mechsim::engine::BeamPoint;
using mechsim::engine::BeamSpec;
using mechsim::engine::LumpedBeam;
using mechsim::engine::MemberOutput;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::apply_point_force;
using mechsim::engine::test::flexural_rigidity;
using mechsim::engine::test::frequency_scale;
using mechsim::engine::test::steel_bar;
using mechsim::engine::test::world_point;

// Member end-loads (ENG-030) and mid-run spring changes (ENG-031, ENG-032), static references.
//
// Settling. Node damping D = (2 / omega_c) K with omega_c the closed-form first frequency gives every mode
// zeta_i = omega_i / omega_c (about critical for the first, overdamped above), so the transient decays at least as
// exp(-omega_c t / 2); 6000 steps of 1 ms at omega_c >= 50 rad/s leave below exp(-150), far under 1e-9 of any
// static quantity. At the settled fixed point the step's force balance is the chain's static equilibrium.
//
// Geometry. Loads are kept so that the deflection delta satisfies delta / L <= 1e-5: geometric nonlinearity enters
// the static quantities used here at (delta / L)^2 <= 1e-10 relative, below the 1e-9 tolerance (brief, ENG-030 to
// ENG-032 unit tolerance 1e-9 relative, docs/spec/eng.md transmission and lumped tolerances).

namespace {

constexpr double tolerance = 1e-9;
constexpr int settle_steps = 6000;

struct Rig {
    MultibodyModel model;
    LumpedBeam beam;
};

Rig rig(BeamSpec spec, double omega_c) {
    spec.node_damping = 2.0 / omega_c * flexural_rigidity(spec) / (spec.length / spec.segments);
    Rig out;
    out.model.gravity = Eigen::Vector3d::Zero();
    out.beam = add_lumped_beam(out.model, spec);
    return out;
}

double cantilever_omega(const BeamSpec& spec) {
    const double root = 1.8751040687119611;
    return root * root * frequency_scale(spec);
}

void settle(MultibodyProvider& provider, SemiImplicitEuler& integrator, State& state, const BeamPoint& point,
            double force) {
    for (int n = 0; n < settle_steps; ++n) {
        if (force != 0.0) {
            apply_point_force(provider, state, point, Eigen::Vector3d(0.0, force, 0.0));
        }
        provider.step(integrator, state);
    }
}

}  // namespace

// Cantilever, tip load P along world -y, gravity off. Statics of the whole member: the support exerts +P y and the
// moment that balances P about the root, M_z = P x_tip (x_tip the deformed tip abscissa, the step's start pose);
// x_tip = L (1 - O((delta / L)^2)), so M_z = P L within 1e-10 relative. The remaining components vanish.
TEST_CASE("cantilever root end-load is the tip load's shear and moment", "[ENG-030][reference]") {
    const BeamSpec spec = steel_bar(1.0, BeamEnds::Cantilever, 4);
    const Rig r = rig(spec, cantilever_omega(spec));
    MultibodyProvider provider(r.model);
    SemiImplicitEuler integrator(StepConfig{});
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    const double length = spec.length;
    const double load = 1e-5 * 3.0 * flexural_rigidity(spec) / (length * length);  // delta / L ~ 1e-5
    const BeamPoint tip = beam_point(r.beam, length);
    settle(provider, integrator, state, tip, -load);
    const double x_tip = world_point(provider, state, tip).x();
    apply_point_force(provider, state, tip, Eigen::Vector3d(0.0, -load, 0.0));
    provider.step(integrator, state);
    REQUIRE(provider.outputs().members.size() == 1);
    const MemberOutput& loads = provider.outputs().members[0];
    std::printf("ENG-030 cantilever: root moment %.15g (P L %.15g, P x_tip %.15g), shear %.15g (P %.15g)\n",
                loads.start_moment.z(), load * length, load * x_tip, loads.start_force.y(), load);
    REQUIRE(std::abs(loads.start_moment.z() - load * length) <= tolerance * load * length);
    REQUIRE(std::abs(loads.start_moment.z() - load * x_tip) <= tolerance * load * length);
    REQUIRE(std::abs(loads.start_force.y() - load) <= tolerance * load);
    REQUIRE(std::abs(loads.start_force.x()) <= tolerance * load);
    REQUIRE(std::abs(loads.start_force.z()) <= tolerance * load);
    REQUIRE(loads.start_moment.head<2>().norm() <= tolerance * load * length);
    REQUIRE(loads.end_force.norm() == 0.0);
    REQUIRE(loads.end_moment.norm() == 0.0);
}

// Simply supported, midspan load P along -y. Moments about the pin: R_end x_end = P x_load; the deformed shape is
// symmetric about the loaded material point and the roller end moves in by the shortening s while the load point
// moves by s / 2, so x_load / x_end = (L / 2 - s / 2) / (L - s) = 1 / 2 exactly: R_start = R_end = P / 2. The pin
// carries no moment (no node spring at x = 0) and the roller point constraint none.
TEST_CASE("simply supported end-loads are half the midspan load with zero end moments", "[ENG-030][reference]") {
    const BeamSpec spec = steel_bar(1.5, BeamEnds::SimplySupported, 4);
    const Rig r = rig(spec, std::numbers::pi * std::numbers::pi * frequency_scale(spec));
    MultibodyProvider provider(r.model);
    SemiImplicitEuler integrator(StepConfig{});
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    const double length = spec.length;
    const double load = 1e-5 * 48.0 * flexural_rigidity(spec) / (length * length);
    const BeamPoint middle = beam_point(r.beam, 0.5 * length);
    settle(provider, integrator, state, middle, -load);
    apply_point_force(provider, state, middle, Eigen::Vector3d(0.0, -load, 0.0));
    provider.step(integrator, state);
    const MemberOutput& loads = provider.outputs().members[0];
    std::printf("ENG-030 simply supported: start %.15g, end %.15g (P / 2 %.15g), start moment %.3g\n",
                loads.start_force.y(), loads.end_force.y(), 0.5 * load, loads.start_moment.z());
    REQUIRE(std::abs(loads.start_force.y() - 0.5 * load) <= tolerance * load);
    REQUIRE(std::abs(loads.end_force.y() - 0.5 * load) <= tolerance * load);
    REQUIRE(std::abs(loads.start_force.x()) <= tolerance * load);
    REQUIRE(std::abs(loads.end_force.x()) <= tolerance * load);
    REQUIRE(loads.start_moment.norm() <= tolerance * load * length);
    REQUIRE(loads.end_moment.norm() == 0.0);
}

// Yield offset phi on the root node (x_0 = l / 2), no load: the springs' rest state is the outboard part rotated
// rigidly by phi about node 0, so the settled tip is at y = (L - l / 2) sin(phi), x = l / 2 + (L - l / 2) cos(phi).
// (The brief's "offset times L" is this lever about the node, L - l / 2, for the midpoint-node chain.) With a tip
// load as well, the shift is the same lever times sin(phi) to (delta / L)^2 + delta phi / (2 L) <= 1e-9 relative
// (delta / L = 1e-5, phi = 1e-4: the loaded deflection rotated by phi changes y by delta (cos(phi) - 1)).
TEST_CASE("a root spring offset rotates the cantilever about its node", "[ENG-031][reference]") {
    const BeamSpec spec = steel_bar(1.0, BeamEnds::Cantilever, 4);
    const double length = spec.length;
    const double lever = length - 0.5 * length / spec.segments;
    const Rig r = rig(spec, cantilever_omega(spec));
    const BeamPoint tip = beam_point(r.beam, length);

    SECTION("unloaded: exact rigid rotation") {
        const double offset = 1e-3;
        MultibodyProvider provider(r.model);
        SemiImplicitEuler integrator(StepConfig{});
        State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
        provider.step(integrator, state);
        provider.set_spring_offset(0, 0, offset);
        settle(provider, integrator, state, tip, 0.0);
        const Eigen::Vector3d at = world_point(provider, state, tip);
        std::printf("ENG-031 offset: tip y %.15g (closed form %.15g)\n", at.y(), lever * std::sin(offset));
        REQUIRE(std::abs(at.y() - lever * std::sin(offset)) <= tolerance * lever * offset);
        REQUIRE(std::abs(at.x() - (length - lever + lever * std::cos(offset))) <= tolerance * lever * offset);
    }

    SECTION("loaded: the static deflection shifts by the lever times the offset") {
        const double offset = 1e-4;
        const double load = 1e-5 * 3.0 * flexural_rigidity(spec) / (length * length);
        MultibodyProvider provider(r.model);
        SemiImplicitEuler integrator(StepConfig{});
        State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
        settle(provider, integrator, state, tip, -load);
        const double before = world_point(provider, state, tip).y();
        provider.set_spring_offset(0, 0, offset);
        settle(provider, integrator, state, tip, -load);
        const double shift = world_point(provider, state, tip).y() - before;
        std::printf("ENG-031 loaded offset: shift %.15g (closed form %.15g)\n", shift, lever * std::sin(offset));
        REQUIRE(std::abs(shift - lever * std::sin(offset)) <= tolerance * lever * offset);
    }
}

// Halving node j's stiffness. Unit-load method on the chain (lumped_beam.hpp): the tip deflection under a tip load P
// is sum_j P (L - x_j)^2 / k_j to (delta / L)^2 relative; changing k_j to k_j / 2 adds P (L - x_j)^2 / k_j, so the
// tip moves by -P (L - x_j)^2 / k (load along -y), x_j = (j + 1/2) l.
TEST_CASE("halving one node's stiffness adds that node's share of the deflection", "[ENG-032][reference]") {
    const BeamSpec spec = steel_bar(1.0, BeamEnds::Cantilever, 4);
    const double length = spec.length;
    const double l = length / spec.segments;
    const auto node = GENERATE(0, 1, 2);
    const Rig r = rig(spec, cantilever_omega(spec));
    MultibodyProvider provider(r.model);
    SemiImplicitEuler integrator(StepConfig{});
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    const double load = 1e-5 * 3.0 * flexural_rigidity(spec) / (length * length);
    const BeamPoint tip = beam_point(r.beam, length);
    settle(provider, integrator, state, tip, -load);
    const double before = world_point(provider, state, tip).y();
    const double k = r.beam.node_stiffness;
    provider.set_spring_stiffness(0, node, 0.5 * k);
    settle(provider, integrator, state, tip, -load);
    const double arm = length - (node + 0.5) * l;
    const double expected = -load * arm * arm / k;
    const double change = world_point(provider, state, tip).y() - before;
    std::printf("ENG-032 node %d: change %.15g (closed form %.15g)\n", node, change, expected);
    REQUIRE(std::abs(change - expected) <= tolerance * std::abs(expected));
}
