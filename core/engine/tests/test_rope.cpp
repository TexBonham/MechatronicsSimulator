#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <numbers>
#include <vector>

#include "mechsim/engine/dynamics_provider.hpp"
#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_model.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/rope.hpp"
#include "rope_helpers.hpp"

using mechsim::engine::DynamicsTerms;
using mechsim::engine::InvalidModelError;
using mechsim::engine::JointType;
using mechsim::engine::LoadSource;
using mechsim::engine::ModelChange;
using mechsim::engine::ModelChangeEvent;
using mechsim::engine::MultibodyConfig;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::Rope;
using mechsim::engine::rope_segment;
using mechsim::engine::RopeSegment;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::add_bob;
using mechsim::engine::test::gravity;

// Rope chain unit behaviour (ENG-047, ADR 0016) and its load release (ENG-034). Exact or 1e-9 relative values, each
// derived where it is asserted.

namespace {

Rope basic_rope(Eigen::Index to_body, int segments) {
    Rope rope;
    rope.to_body = to_body;
    rope.segments = segments;
    rope.axial_rigidity = 100.0;
    rope.mass_per_length = 0.1;
    rope.rest_length = 1.0;
    return rope;
}

State rest_state(const MultibodyProvider& provider) {
    return State{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
}

}  // namespace

// ADR 0016: N - 1 interior Translation nodes of mu l, evenly placed; end half-masses on the end bodies, none on the
// world, so the built mass is mu rest_length less the world end's mu l / 2.
TEST_CASE("add_rope builds N - 1 nodes and conserves the rope mass", "[ENG-047]") {
    MultibodyModel model;
    model.bodies.emplace_back();
    model.bodies[0].joint.type = JointType::Translation;
    model.bodies[0].mass = 3.0;
    const Eigen::Index bob = add_bob(model, Eigen::Vector3d(0.0, 0.0, -1.0), 1.0);
    Rope rope = basic_rope(bob, 4);
    rope.from_body = 0;
    rope.from_point = Eigen::Vector3d(0.0, 0.0, 0.0);
    const Eigen::Index index = add_rope(model, rope, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, -1.0));
    REQUIRE(index == 0);
    REQUIRE(model.ropes.size() == 1);
    const Rope& built = model.ropes[0];
    REQUIRE(built.nodes.size() == 3);
    double nodes = 0.0;
    for (std::size_t i = 0; i < built.nodes.size(); ++i) {
        const auto& body = model.bodies[static_cast<std::size_t>(built.nodes[i])];
        REQUIRE(body.joint.type == JointType::Translation);
        REQUIRE(body.parent == -1);
        REQUIRE(std::abs(body.joint.position.z() + 0.25 * static_cast<double>(i + 1)) <= 1e-15);
        nodes += body.mass;
    }
    const double l = 0.25;
    REQUIRE(std::abs(nodes - 3.0 * 0.1 * l) <= 1e-15);
    REQUIRE(std::abs(model.bodies[0].mass - (3.0 + 0.05 * l)) <= 1e-15);
    REQUIRE(std::abs(model.bodies[static_cast<std::size_t>(bob)].mass - (1.0 + 0.05 * l)) <= 1e-15);
    REQUIRE(std::abs(nodes + 0.1 * l - 0.1) <= 1e-15);
    const MultibodyProvider provider(model);
    REQUIRE(provider.velocity_size() == 3 + 3 + 3 * 3);
}

TEST_CASE("a one-segment rope has no nodes", "[ENG-047]") {
    MultibodyModel model;
    const Eigen::Index bob = add_bob(model, Eigen::Vector3d(0.0, 0.0, -1.0), 1.0);
    static_cast<void>(add_rope(model, basic_rope(bob, 1), Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, -1.0)));
    REQUIRE(model.ropes[0].nodes.empty());
    REQUIRE(model.bodies.size() == 1);
}

TEST_CASE("invalid rope parameters are rejected", "[ENG-047]") {
    MultibodyModel model;
    const Eigen::Index bob = add_bob(model, Eigen::Vector3d(0.0, 0.0, -1.0), 1.0);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const Eigen::Vector3d a = Eigen::Vector3d::Zero();
    const Eigen::Vector3d b(0.0, 0.0, -1.0);
    Rope rope = basic_rope(bob, 2);
    SECTION("segments") { rope.segments = 0; }
    SECTION("EA zero") { rope.axial_rigidity = 0.0; }
    SECTION("EA infinite") { rope.axial_rigidity = inf; }
    SECTION("damping negative") { rope.damping = -1.0; }
    SECTION("damping nan") { rope.damping = nan; }
    SECTION("mass per length zero") { rope.mass_per_length = 0.0; }
    SECTION("rest length negative") { rope.rest_length = -1.0; }
    SECTION("rest length nan") { rope.rest_length = nan; }
    SECTION("breaking tension zero") { rope.breaking_tension = 0.0; }
    SECTION("breaking tension nan") { rope.breaking_tension = nan; }
    SECTION("attachment damping negative") { rope.attachment_damping = -0.1; }
    SECTION("attachment friction infinite") { rope.attachment_friction = inf; }
    SECTION("from point") { rope.from_point.x() = nan; }
    SECTION("missing to body") { rope.to_body = 5; }
    SECTION("missing from body") { rope.from_body = -2; }
    SECTION("self") { rope.from_body = bob; }
    REQUIRE_THROWS_AS(add_rope(model, rope, a, b), InvalidModelError);
}

TEST_CASE("a rope whose nodes are not built by add_rope is rejected by the provider", "[ENG-047]") {
    MultibodyModel model;
    const Eigen::Index bob = add_bob(model, Eigen::Vector3d(0.0, 0.0, -1.0), 1.0);
    static_cast<void>(add_rope(model, basic_rope(bob, 3), Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, -1.0)));
    SECTION("node count") { model.ropes[0].nodes.pop_back(); }
    SECTION("node joint") {
        model.bodies[static_cast<std::size_t>(model.ropes[0].nodes[0])].joint.type = JointType::Prismatic;
        model.bodies[static_cast<std::size_t>(model.ropes[0].nodes[0])].joint.axis = Eigen::Vector3d::UnitZ();
    }
    SECTION("node missing") { model.ropes[0].nodes[0] = 7; }
    SECTION("missing body") { model.ropes[0].to_body = 9; }
    SECTION("parameter") { model.ropes[0].rest_length = 0.0; }
    REQUIRE_THROWS_AS(MultibodyProvider(model), InvalidModelError);
}

// T = k (d - l) + c d_dot when d > l and positive, else 0.
TEST_CASE("a rope segment pulls only when stretched", "[ENG-047]") {
    const Eigen::Vector3d a = Eigen::Vector3d::Zero();
    const Eigen::Vector3d still = Eigen::Vector3d::Zero();
    const RopeSegment taut = rope_segment(a, still, Eigen::Vector3d(0.0, 0.0, -1.1), Eigen::Vector3d(0.0, 0.0, -0.5),
                                          1.0, 100.0, 2.0);
    REQUIRE(taut.taut);
    REQUIRE(std::abs(taut.tension - (100.0 * 0.1 + 2.0 * 0.5)) <= 1e-12);
    REQUIRE(taut.direction == Eigen::Vector3d(0.0, 0.0, -1.0));
    const RopeSegment slack = rope_segment(a, still, Eigen::Vector3d(0.0, 0.0, -0.9), still, 1.0, 100.0, 2.0);
    REQUIRE_FALSE(slack.taut);
    REQUIRE(slack.tension == 0.0);
    const RopeSegment exact = rope_segment(a, still, Eigen::Vector3d(0.0, 0.0, -1.0), still, 1.0, 100.0, 2.0);
    REQUIRE_FALSE(exact.taut);
    const RopeSegment closing = rope_segment(a, still, Eigen::Vector3d(0.0, 0.0, -1.01), Eigen::Vector3d(0.0, 0.0, 1.0),
                                             1.0, 100.0, 2.0);
    REQUIRE_FALSE(closing.taut);
    REQUIRE(closing.tension == 0.0);
    const RopeSegment coincident = rope_segment(a, still, a, still, 1.0, 100.0, 2.0);
    REQUIRE(coincident.tension == 0.0);
    REQUIRE(coincident.direction == Eigen::Vector3d::Zero());
}

// A taut single segment from the world to a bob displaced sideways: K's bob block is k n n^T + (T / d)(I - n n^T)
// and D's is c n n^T (ADR 0016), with T from the stretch.
TEST_CASE("a taut segment adds axial and geometric stiffness", "[ENG-047]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    const Eigen::Vector3d at(0.6, 0.0, -0.8);
    const Eigen::Index bob = add_bob(model, at, 1.0);
    Rope rope = basic_rope(bob, 1);
    rope.rest_length = 0.9;
    rope.damping = 3.0;
    static_cast<void>(add_rope(model, rope, Eigen::Vector3d::Zero(), at));
    const MultibodyProvider provider(model);
    DynamicsTerms terms;
    const State state = rest_state(provider);
    provider.evaluate(0.0, state.q, state.v, terms);
    const Eigen::Vector3d n = at.normalized();
    const double k = 100.0 / 0.9;
    const double tension = k * 0.1;
    const Eigen::Matrix3d expected =
        k * n * n.transpose() + tension * (Eigen::Matrix3d::Identity() - n * n.transpose());
    REQUIRE((terms.stiffness - expected).cwiseAbs().maxCoeff() <= 1e-12);
    REQUIRE((terms.damping - (3.0 / 0.9) * n * n.transpose()).cwiseAbs().maxCoeff() <= 1e-12);
    REQUIRE((terms.force + tension * n).cwiseAbs().maxCoeff() <= 1e-12);
}

// Ruling 14: bob at (0, 0, -L) moving at u along x on a slack single segment. omega_rel = n x v / L = (0, -u / L, 0),
// tau = -(c + f / max(|u| / L, w)) omega_rel with w the friction slip threshold (MultibodyConfig, 1e-4 rad/s): above
// w the force on the bob (tau x n) / L is -(c u / L^2 + f sign(u) / L) along x, the full Coulomb value, and D_xx gets
// (c + f L / |u|) / L^2; at zero rate the force is 0 and D_xx is the bounded (c + f / w) / L^2; exactly at the
// threshold both forms agree.
TEST_CASE("the top attachment damps and rubs the swing", "[ENG-047]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    const double length = 0.5;
    const Eigen::Vector3d at(0.0, 0.0, -length);
    const Eigen::Index bob = add_bob(model, at, 1.0);
    Rope rope = basic_rope(bob, 1);
    rope.rest_length = length;
    rope.attachment_damping = 0.2;
    rope.attachment_friction = 0.05;
    static_cast<void>(add_rope(model, rope, Eigen::Vector3d::Zero(), at));
    const MultibodyProvider provider(model);
    const double u = -0.3;
    Eigen::VectorXd v = Eigen::VectorXd::Zero(provider.velocity_size());
    v[0] = u;
    DynamicsTerms terms;
    provider.evaluate(0.0, provider.neutral_positions(), v, terms);
    const double coefficient = 0.2 + 0.05 * length / std::abs(u);
    REQUIRE(std::abs(terms.force[0] - (-coefficient * u / (length * length))) <= 1e-12);
    REQUIRE(std::abs(terms.force[1]) <= 1e-15);
    REQUIRE(std::abs(terms.force[2]) <= 1e-15);
    REQUIRE(std::abs(terms.damping(0, 0) - coefficient / (length * length)) <= 1e-12);
    REQUIRE(std::abs(terms.damping(1, 1) - coefficient / (length * length)) <= 1e-12);
    REQUIRE(std::abs(terms.damping(2, 2)) <= 1e-15);
    const double threshold = provider.config().friction_slip_threshold;
    REQUIRE(threshold == 1e-4);
    const double regularised = (0.2 + 0.05 / threshold) / (length * length);
    provider.evaluate(0.0, provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), terms);
    REQUIRE(std::abs(terms.damping(0, 0) / regularised - 1.0) <= 1e-12);
    REQUIRE(terms.force.norm() == 0.0);
    v[0] = threshold * length;
    provider.evaluate(0.0, provider.neutral_positions(), v, terms);
    REQUIRE(std::abs(terms.damping(0, 0) / regularised - 1.0) <= 1e-12);
    const double coulomb = -(0.2 * v[0] / (length * length) + 0.05 / length);
    REQUIRE(std::abs(terms.force[0] / coulomb - 1.0) <= 1e-12);
    v[0] = 0.5 * threshold * length;
    provider.evaluate(0.0, provider.neutral_positions(), v, terms);
    REQUIRE(std::abs(terms.force[0] / (-regularised * v[0]) - 1.0) <= 1e-12);
}

TEST_CASE("a non-positive friction slip threshold is rejected", "[ENG-047]") {
    const MultibodyModel model{};
    mechsim::engine::MultibodyConfig config;
    config.friction_slip_threshold = 0.0;
    REQUIRE_THROWS(MultibodyProvider(model, config));
    config.friction_slip_threshold = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_THROWS(MultibodyProvider(model, config));
}

// Gravity off, ends 0.9 m apart on a 1 m rope: every segment is slack, nothing moves, every tension is exactly 0.
// Winching to 0.8 m makes the chain taut; at rest the series tension is EA (d - r) / r = 12.5 N in every segment.
// Settling: c_seg = damping / l ~ 2 sqrt(k_seg m_node), the slowest (overdamped) axial decay about k / c = 50 /s,
// 3000 steps leave below exp(-150); the nodes start on the line, so no lateral motion. 1e-9 relative.
TEST_CASE("a slack rope carries zero tension and a winch makes it taut", "[ENG-047]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    Rope rope = basic_rope(-1, 3);
    rope.to_point = Eigen::Vector3d(0.9, 0.0, 0.0);
    rope.damping = 1.9;
    static_cast<void>(add_rope(model, rope, Eigen::Vector3d::Zero(), rope.to_point));
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(StepConfig{});
    State state = rest_state(provider);
    for (int n = 0; n < 50; ++n) {
        provider.step(integrator, state);
    }
    REQUIRE(state.q.norm() == 0.0);
    for (const double tension : provider.outputs().ropes[0].tension) {
        REQUIRE(tension == 0.0);
    }
    REQUIRE(provider.outputs().ropes[0].rest_length == 1.0);
    REQUIRE(provider.outputs().ropes[0].snatch_omega_h == 0.0);
    REQUIRE_FALSE(provider.outputs().ropes[0].snatch_underestimated);
    provider.set_rope_rest_length(0, 0.8);
    for (int n = 0; n < 3000; ++n) {
        provider.step(integrator, state);
    }
    const double expected = 100.0 * (0.9 - 0.8) / 0.8;
    REQUIRE(provider.outputs().ropes[0].tension.size() == 3);
    for (const double tension : provider.outputs().ropes[0].tension) {
        REQUIRE(std::abs(tension - expected) <= 1e-9 * expected);
    }
    REQUIRE(provider.outputs().ropes[0].rest_length == 0.8);
    REQUIRE(provider.outputs().events.empty());
    REQUIRE_THROWS_AS(provider.set_rope_rest_length(0, 0.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_rope_rest_length(0, -1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_rope_rest_length(0, std::numeric_limits<double>::infinity()), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_rope_rest_length(1, 1.0), InvalidModelError);
}

// Snap within one step. Bob M = 1 kg on a 1 m, N = 2 rope, EA = 1e8 N, hung at its static stretch: tension g M.
// A step load F = 10 N down: backward Euler on the bob-on-rope mode (omega h = sqrt(EA / (L M)) h = 10) moves the bob
// (omega h)^2 / (1 + (omega h)^2) = 100 / 101 of the way to the new static stretch in one step, so the end-of-step
// tension is at least g M + 0.99 F = 19.7 N > breaking 14.81 N > g M: the first loaded step snaps, the event lists it
// and from the next step the bob falls free, dv / h = -(g + F / M_bob) exactly. omega_s h = 10 > 0.2: the snatch
// flag is set (ruling 15).
TEST_CASE("a load above the breaking tension snaps the rope within one step", "[ENG-034][ENG-047]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, 0.0, -gravity);
    const double axial = 1e8;
    const double mu = 1e-4;
    const double bob_mass = 1.0;
    const double l = 0.5;
    const double length = 1.0 + gravity * (bob_mass + 0.5 * mu * l) / axial;
    const Eigen::Vector3d at(0.0, 0.0, -length);
    const Eigen::Index bob = add_bob(model, at, bob_mass);
    Rope rope = basic_rope(bob, 2);
    rope.axial_rigidity = axial;
    rope.mass_per_length = mu;
    rope.breaking_tension = gravity * bob_mass + 5.0;
    static_cast<void>(add_rope(model, rope, Eigen::Vector3d::Zero(), at));
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(StepConfig{});
    State state = rest_state(provider);
    const double h = integrator.config().h;
    provider.step(integrator, state);
    REQUIRE(provider.outputs().ropes[0].attached);
    REQUIRE(provider.outputs().ropes[0].max_tension < rope.breaking_tension);
    REQUIRE(provider.outputs().ropes[0].snatch_underestimated);
    const double bob_total = bob_mass + 0.5 * mu * l;
    REQUIRE(std::abs(provider.outputs().ropes[0].snatch_omega_h - std::sqrt(axial / bob_total) * h) <= 1e-12);
    const Eigen::Vector3d load(0.0, 0.0, -10.0);
    provider.apply_body_wrench(bob, Eigen::Vector3d::Zero(), load);
    provider.step(integrator, state);
    const auto& out = provider.outputs();
    REQUIRE(out.ropes[0].max_tension > gravity * bob_mass + 0.99 * 10.0);
    REQUIRE_FALSE(out.ropes[0].attached);
    REQUIRE_FALSE(provider.rope_attached(0));
    REQUIRE(out.events.size() == 2);
    REQUIRE(out.events[1].kind == ModelChange::LoadReleased);
    REQUIRE(out.events[1].source == LoadSource::Rope);
    REQUIRE(out.events[1].time == state.t);
    REQUIRE(out.events[0].kind == ModelChange::RopeSnapped);
    REQUIRE(out.events[0].target == 0);
    REQUIRE(out.events[0].value == out.ropes[0].max_tension);
    REQUIRE(out.events[0].limit == rope.breaking_tension);
    REQUIRE(out.events[0].time == state.t);
    const double before = state.v[provider.velocity_offset(bob) + 2];
    provider.apply_body_wrench(bob, Eigen::Vector3d::Zero(), load);
    provider.step(integrator, state);
    const double acceleration = (state.v[provider.velocity_offset(bob) + 2] - before) / h;
    REQUIRE(std::abs(acceleration + gravity + 10.0 / bob_total) <= 1e-12 * gravity);
    REQUIRE(provider.outputs().ropes[0].load_end_tension == 0.0);
    REQUIRE(provider.outputs().ropes[0].tension.back() == 0.0);
    REQUIRE(provider.outputs().events.empty());
    REQUIRE_THROWS_AS(provider.release_rope(0), InvalidModelError);
}

// The snap's evaluation start (INT-031, ruling 75; rope.hpp, Snap). Bob M = 1 kg on a 1 m, N = 2 rope, EA = 1e4 N
// (omega h = sqrt(EA / (L M)) h = 0.1 at 1 ms), mu = 1e-4 kg/m, released from straight (the rope unstretched,
// tension 0) under gravity. Backward Euler's step response of the bob-on-rope mode peaks at n theta = pi, theta =
// atan x: the tension reaches g M (1 + (1 + x^2)^(-pi / (2 atan x))) = 1.85 g M (the rope's own mass, 1e-4 of M,
// shifts it below the 2 percent asserted) and stays below 2 g (M + mu L) (the step bound). Breaking at 1.5 g M:
//   start 0.5 s (eight periods of the mode, its ringing then e^(-8 pi x) = 0.08 of the 0.85 g M overshoot, below
// breaking, asserted): no snap, the load stays attached, held_tension reads the peak; a 10 N step load after the
// start adds 10 (1 - cos(n theta)) N under the scheme, past g M + 10 N > breaking + 4 N at n theta = pi / 2 less the
// ringing left (0.08 of 0.85 g M = 0.7 N), so it snaps within ceil(pi / (2 atan x)) = 16 steps, not before 0.5 s;
//   start 0: the same run snaps in the first step whose tension exceeds breaking, the first step held above; and a
// default config steps bit-identically to start 0.
TEST_CASE("a rope does not snap before the snap's evaluation start and snaps from it on", "[ENG-047][ENG-034]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, 0.0, -gravity);
    const Eigen::Vector3d at(0.0, 0.0, -1.0);
    const Eigen::Index bob = add_bob(model, at, 1.0);
    Rope rope = basic_rope(bob, 2);
    rope.axial_rigidity = 1e4;
    rope.mass_per_length = 1e-4;
    rope.breaking_tension = 1.5 * gravity;
    static_cast<void>(add_rope(model, rope, Eigen::Vector3d::Zero(), at));
    const double x = 0.1;
    const double peak = gravity * (1.0 + std::pow(1.0 + (x * x), -std::numbers::pi / (2.0 * std::atan(x))));
    const auto run = [&](const MultibodyConfig& config, int steps, std::vector<double>& path, double& held,
                         double& first, bool& snapped, double& snap_time, State& state) {
        MultibodyProvider provider(model, config);
        SemiImplicitEuler integrator(StepConfig{1e-3});
        state = rest_state(provider);
        for (int n = 0; n < steps; ++n) {
            provider.step(integrator, state);
            const auto& out = provider.outputs();
            if (first < 0.0 && out.ropes[0].max_tension > rope.breaking_tension) {
                first = state.t;
            }
            held = out.ropes[0].held_tension;
            for (const ModelChangeEvent& event : out.events) {
                if (event.kind == ModelChange::RopeSnapped && !snapped) {
                    snapped = true;
                    snap_time = event.time;
                }
            }
            path.push_back(state.q[provider.position_offset(bob) + 2]);
        }
        return provider.rope_attached(0);
    };
    MultibodyConfig late;
    late.rope_snap_evaluation_start = 0.5;
    std::vector<double> held_path;
    double held = 0.0;
    double first = -1.0;
    bool snapped = false;
    double snap_time = -1.0;
    State held_state;
    const bool attached = run(late, 499, held_path, held, first, snapped, snap_time, held_state);
    MultibodyConfig zero;
    zero.rope_snap_evaluation_start = 0.0;
    std::vector<double> zero_path;
    double zero_held = 0.0;
    double zero_first = -1.0;
    bool zero_snapped = false;
    double zero_snap_time = -1.0;
    State zero_state;
    static_cast<void>(run(zero, 499, zero_path, zero_held, zero_first, zero_snapped, zero_snap_time, zero_state));
    std::vector<double> default_path;
    double default_held = 0.0;
    double default_first = -1.0;
    bool default_snapped = false;
    double default_snap_time = -1.0;
    State default_state;
    static_cast<void>(run(MultibodyConfig{}, 499, default_path, default_held, default_first, default_snapped,
                          default_snap_time, default_state));
    std::printf("rope snap start: peak tension before the start %.6f N (derived %.6f N, breaking %.6f N, bound %.6f "
                "N), first above breaking at %.4f s; start 0 snapped at %.4f s\n",
                held, peak, rope.breaking_tension, 2.0 * gravity * (1.0 + 1e-4), first, zero_snap_time);
    REQUIRE(attached);
    REQUIRE_FALSE(snapped);
    REQUIRE(held > rope.breaking_tension);
    REQUIRE(std::abs(held / peak - 1.0) <= 0.02);
    REQUIRE(held < 2.0 * gravity * (1.0 + 1e-4));
    REQUIRE(zero_snapped);
    REQUIRE(zero_snap_time == first);
    REQUIRE(zero_held == 0.0);
    REQUIRE(default_path == zero_path);
    REQUIRE(default_snap_time == zero_snap_time);

    MultibodyProvider provider(model, late);
    SemiImplicitEuler integrator(StepConfig{1e-3});
    State state = rest_state(provider);
    while (state.t + 5e-4 < 0.5) {
        provider.step(integrator, state);
    }
    REQUIRE(provider.rope_attached(0));
    provider.step(integrator, state);
    REQUIRE(provider.outputs().ropes[0].max_tension < rope.breaking_tension);
    REQUIRE(provider.rope_attached(0));
    const auto limit = static_cast<int>(std::ceil(std::numbers::pi / (2.0 * std::atan(x))));
    int loaded = 0;
    bool released = false;
    while (loaded < limit && !released) {
        provider.apply_body_wrench(bob, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, -10.0));
        provider.step(integrator, state);
        ++loaded;
        for (const ModelChangeEvent& event : provider.outputs().events) {
            released = released || event.kind == ModelChange::RopeSnapped;
        }
    }
    std::printf("rope snap start: after the start a 10 N step load snaps in step %d of at most %d\n", loaded, limit);
    REQUIRE(released);
    REQUIRE_FALSE(provider.rope_attached(0));
    REQUIRE(provider.outputs().ropes[0].held_tension == held);
}

// The snap's evaluation start by step count (rope.hpp, Snap): the first judged step is n = ceil(start / h - 1e-9). A
// bob hung at its static stretch with a breaking tension below g M exceeds it in every step, so the rope snaps in step
// n exactly and stays attached for the n steps before: start / h for 0.01 s and 8 s at 1 ms and 0.1 ms, 2 for
// 0.15 ms at 0.1 ms (rounded up), 0 for start 0.
TEST_CASE("a rope's snap evaluation start is a whole number of steps, rounded up", "[ENG-047][ENG-034]") {
    struct Case {
        double h;
        double start;
        long expected;
    };
    for (const Case c : {Case{1e-3, 0.01, 10}, Case{1e-4, 0.01, 100}, Case{1e-3, 8.0, 8000}, Case{1e-4, 8.0, 80000},
                         Case{1e-4, 1.5e-4, 2}, Case{1e-3, 0.0, 0}}) {
        MultibodyModel model;
        model.gravity = Eigen::Vector3d(0.0, 0.0, -gravity);
        const double axial = 1e4;
        const double length = 1.0 + gravity * (1.0 + 0.5 * 1e-4 * 0.5) / axial;
        const Eigen::Vector3d at(0.0, 0.0, -length);
        const Eigen::Index bob = add_bob(model, at, 1.0);
        Rope rope = basic_rope(bob, 2);
        rope.axial_rigidity = axial;
        rope.mass_per_length = 1e-4;
        rope.breaking_tension = 0.5 * gravity;
        static_cast<void>(add_rope(model, rope, Eigen::Vector3d::Zero(), at));
        MultibodyConfig config;
        config.rope_snap_evaluation_start = c.start;
        MultibodyProvider provider(model, config);
        SemiImplicitEuler integrator(StepConfig{c.h});
        State state = rest_state(provider);
        long attached = 0;
        while (provider.rope_attached(0) && attached <= c.expected) {
            provider.step(integrator, state);
            if (provider.rope_attached(0)) {
                ++attached;
            }
        }
        std::printf("rope snap start %.6g s at h %.0e: %ld steps held (expected %ld)\n", c.start, c.h, attached,
                    c.expected);
        CHECK(attached == c.expected);
        CHECK_FALSE(provider.rope_attached(0));
    }
}

TEST_CASE("a rope released on request lets its load go and lists the release", "[ENG-034][ENG-047]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d(0.0, 0.0, -gravity);
    const Eigen::Vector3d at(0.0, 0.0, -1.0);
    const Eigen::Index bob = add_bob(model, at, 1.0);
    static_cast<void>(add_rope(model, basic_rope(bob, 1), Eigen::Vector3d::Zero(), at));
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(StepConfig{});
    State state = rest_state(provider);
    const double h = integrator.config().h;
    provider.step(integrator, state);
    // Ruling 21: the swing's pi omega h, omega = sqrt(g / d), d the segment length (the bob's distance).
    const Eigen::Vector3d bob_at = provider.outputs().bodies[0].position;
    const double swing = std::numbers::pi * std::sqrt(gravity / bob_at.norm()) * h;
    REQUIRE(std::abs(provider.outputs().ropes[0].scheme_decay / swing - 1.0) <= 1e-12);
    provider.release_rope(0);
    const double before = state.v[2];
    provider.step(integrator, state);
    REQUIRE(std::abs((state.v[2] - before) / h + gravity) <= 1e-12 * gravity);
    REQUIRE(provider.outputs().events.size() == 1);
    REQUIRE(provider.outputs().events[0].kind == ModelChange::LoadReleased);
    REQUIRE(provider.outputs().events[0].source == LoadSource::Rope);
    REQUIRE(provider.outputs().events[0].target == 0);
    REQUIRE(provider.outputs().events[0].time == h);
    REQUIRE(provider.outputs().ropes[0].tension.size() == 1);
    REQUIRE(provider.outputs().ropes[0].max_tension == 0.0);
    REQUIRE_THROWS_AS(provider.release_rope(0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.release_rope(1), InvalidModelError);
    REQUIRE_THROWS_AS(static_cast<void>(provider.rope_attached(-1)), InvalidModelError);
}

// ENG-035 with ENG-047: a payload hanging on a rope changes mass mid-run. The bob carries its own mass plus the rope's
// end half-segment mu l / 2 = 0.025 kg (l = 0.5 m), before and after, so the load-end tension settles to (m + 0.025) g.
// Axial damping c / l = 40 N s/m against k = EA / l = 200 N/m holds the slowest axial mode near critical at m = 2 kg
// (decay rate about 5 / s): 10 s leaves e^-50 of each transient, so 1e-9 relative (the rope's static tolerance).
TEST_CASE("a rope payload's mid-run mass change keeps the rope's end share", "[ENG-035][ENG-047]") {
    MultibodyModel model;
    const Eigen::Index bob = add_bob(model, Eigen::Vector3d(0.0, 0.0, -1.0), 1.0);
    Rope rope = basic_rope(bob, 2);
    rope.damping = 20.0;
    static_cast<void>(add_rope(model, rope, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, -1.0)));
    MultibodyProvider provider(model);
    State state = rest_state(provider);
    SemiImplicitEuler integrator{StepConfig{}};
    for (int n = 0; n < 10000; ++n) {
        provider.step(integrator, state);
    }
    const double share = 0.5 * 0.1 * 0.5;
    CHECK(std::abs(provider.outputs().ropes[0].load_end_tension / ((1.0 + share) * gravity) - 1.0) <= 1e-9);
    provider.set_body_mass(bob, 2.0);
    REQUIRE(std::abs(provider.model().bodies[static_cast<std::size_t>(bob)].mass - (2.0 + share)) <= 1e-15);
    for (int n = 0; n < 10000; ++n) {
        provider.step(integrator, state);
    }
    CHECK(std::abs(provider.outputs().ropes[0].load_end_tension / ((2.0 + share) * gravity) - 1.0) <= 1e-9);
}
