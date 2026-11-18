#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>
#include <string>

#include "beam_helpers.hpp"
#include "carried_helpers.hpp"
#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/member_loads.hpp"
#include "mechsim/engine/multibody_provider.hpp"

using mechsim::engine::BeamEnds;
using mechsim::engine::Body;
using mechsim::engine::CarriedLoadOutput;
using mechsim::engine::InvalidModelError;
using mechsim::engine::LumpedBeam;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::carried_rig;
using mechsim::engine::test::CarriedRig;
using mechsim::engine::test::flexural_rigidity;
using mechsim::engine::test::ParentKind;
using mechsim::engine::test::rig_gravity;
using mechsim::engine::test::run_steps;
using mechsim::engine::test::settling_steps;
using mechsim::engine::test::start_state;

// Load-path coupling (ENG-052, ruling 63): the rerouted pair is internal, so the parent's reaction from the world
// is the same with and without the member carrying the joint; node and end placement; validation.

namespace {

double total_mass(const MultibodyModel& model) {
    double m = 0.0;
    for (const Body& body : model.bodies) {
        m += body.mass;
    }
    return m;
}

// The deflection of every load node, world y, m.
double largest_node_deflection(const MultibodyProvider& provider, const State& state) {
    const LumpedBeam& beam = provider.model().members.front();
    double largest = 0.0;
    for (const mechsim::engine::LoadNode& node : mechsim::engine::load_nodes(beam)) {
        mechsim::engine::BeamPoint point;
        point.body = node.body;
        point.point = node.point;
        largest = std::max(largest, std::abs(mechsim::engine::test::world_point(provider, state, point).y()));
    }
    return largest;
}

}  // namespace

TEST_CASE("carried: the parent's reaction from the world is the sum of the weights, carried or not",
          "[ENG-052][ENG-030]") {
    // Newton on the whole tree at rest: the world's force on the fixed parent is sum m g upward. The pair adds
    // nothing to it (equal and opposite at the same points). The settled transient (carried_helpers.hpp) is below
    // e^(-40) of the load, and the loop solve leaves |J v| <= 1e-9 m/s (ENG-025), an acceleration error of at most
    // 1e-6 m/s^2 on the 0.8 kg member: 1e-6 N, 4e-8 of the 25 N total. Asserted 1e-9 relative between the carried and
    // the uncarried run (the same rows, the same residuals) and against sum m g.
    for (const BeamEnds ends : {BeamEnds::SimplySupported, BeamEnds::Clamped, BeamEnds::Cantilever}) {
        int segments = 8;
        if (ends == BeamEnds::Clamped) {
            segments = 16;
        }
        std::array<double, 2> reactions{0.0, 0.0};
        double weight = 0.0;
        for (const bool carry : {false, true}) {
            const CarriedRig rig = carried_rig(ends, segments, 2.0, 0.05, carry, ParentKind::Fixed);
            weight = total_mass(rig.model) * rig_gravity;
            MultibodyProvider provider(rig.model);
            State state = start_state(provider, rig.carriage, 0.4);
            run_steps(provider, state, settling_steps(rig) + 1);
            reactions[static_cast<std::size_t>(carry)] = provider.outputs().joints.front().reaction_wrench[4];
        }
        std::printf("carried parent reaction: uncarried %.12e, carried %.12e, sum m g %.12e\n", reactions[0],
                    reactions[1], weight);
        CAPTURE(static_cast<int>(ends));
        CHECK(std::abs(reactions[1] / reactions[0] - 1.0) <= 1e-9);
        CHECK(std::abs(reactions[1] / weight - 1.0) <= 1e-9);
    }
}

TEST_CASE("carried: a carriage exactly at a node loads that node alone; at a supported end it bends nothing",
          "[ENG-052]") {
    // Simply supported, N = 8, l = 1 / 8: node 3 is the spring node at 2.5 l. At x = 0 the pin segment's origin is the
    // pin itself and at x = L the last segment's tip is the roller point: the load goes straight into the support
    // and every node stays on the axis, up to the loop solve's residual (|J v| <= 1e-9 m/s for 1 ms per step,
    // projected back below 1e-6 m by ENG-024 and in practice far below): asserted below 1e-6 of the midspan
    // deflection the same load would give, P L^3 / (48 E I).
    const double load = 10.0;
    for (const double a : {2.5 / 8.0, 0.0, 1.0}) {
        CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1.0, 0.0, true);
        rig.model.gravity = Eigen::Vector3d::Zero();
        MultibodyProvider provider(rig.model);
        State state = start_state(provider, rig.carriage, a);
        SemiImplicitEuler integrator(StepConfig{1e-3});
        const int steps = settling_steps(rig);
        for (int n = 0; n < steps; ++n) {
            provider.apply_body_wrench(rig.carriage, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, -load, 0.0));
            static_cast<void>(provider.step(integrator, state));
        }
        const CarriedLoadOutput& c = provider.outputs().carried.front();
        CAPTURE(a, c.node, c.weight);
        if (a == 2.5 / 8.0) {
            CHECK(c.node == 3);
            CHECK(c.weight == 1.0);
            continue;
        }
        const double midspan = load / (48.0 * flexural_rigidity(rig.spec));
        const double largest = largest_node_deflection(provider, state);
        std::printf("carried at x = %.1f: largest node deflection %.3e (midspan scale %.3e)\n", a, largest, midspan);
        CHECK(largest <= 1e-6 * midspan);
        CHECK(provider.outputs().members.front().carried_deflection <= 1e-6 * midspan);
    }
}

TEST_CASE("carried: the first step already carries a resting carriage's static weight", "[ENG-052]") {
    const CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 2.0, 0.05, true);
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, 0.5);
    run_steps(provider, state, 1);
    const CarriedLoadOutput& c = provider.outputs().carried.front();
    CHECK(c.force == -2.0 * rig_gravity);
    CHECK(c.moment == 0.0);
    CHECK(c.member == 0);
    CHECK(c.body == rig.carriage);
}

TEST_CASE("carried: the parent's motion with and without rerouting differs only by the member's deflection",
          "[ENG-052]") {
    // A 5 kg parent slides across the member (world y) under a constant 50 N while the carriage accelerates along
    // it under 2 N; gravity off. The pair is internal, so in the parent's equation (its row of the mass matrix is the
    // whole tree's y momentum) it cancels: both runs have the same y momentum and the same y centre of mass at every
    // step. With y_n = y_P + u_n for each member body (u the deflection relative to the parent) and the carriage
    // riding y_P: M y_com = M y_P + sum m_n u_n, so
    //   |y_P - y_P'| = |sum m_n (u_n - u_n')| / M <= (m_b / M) max |u - u'|.
    // u - u' is the deflection under the rerouted load m_c a_P (a_P = F / M): at most its static value at
    // midspan, P L^3 / (48 E I) (Maxwell: the largest of any point under a load anywhere), times 2 for a suddenly
    // applied load. The one-step lag cannot break the identity: each step's pair is internal whatever load it uses.
    const double force = 50.0;
    const double push = 2.0;
    const double carriage_mass = 2.0;
    std::array<double, 2> y{0.0, 0.0};
    double bound = 0.0;
    for (const bool carry : {false, true}) {
        CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, carriage_mass, 0.0, carry, ParentKind::Sliding);
        rig.model.gravity = Eigen::Vector3d::Zero();
        const double total = total_mass(rig.model);
        const double member = rig.spec.density * rig.spec.area * rig.spec.length;
        const double load = carriage_mass * force / total;
        bound = 2.0 * (member / total) * load / (48.0 * flexural_rigidity(rig.spec));
        MultibodyProvider provider(rig.model);
        State state = start_state(provider, rig.carriage, 0.1);
        SemiImplicitEuler integrator(StepConfig{1e-3});
        for (int n = 0; n < 500; ++n) {
            provider.apply_body_wrench(rig.parent, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, force, 0.0));
            provider.apply_body_wrench(rig.carriage, Eigen::Vector3d::Zero(), Eigen::Vector3d(push, 0.0, 0.0));
            static_cast<void>(provider.step(integrator, state));
        }
        y[static_cast<std::size_t>(carry)] = state.q[provider.position_offset(rig.parent)];
    }
    std::printf("carried sliding parent: y %.12e vs %.12e, difference %.3e, bound %.3e\n", y[1], y[0],
                std::abs(y[1] - y[0]), bound);
    CHECK(std::abs(y[1] - y[0]) <= bound);
}

TEST_CASE("carried: the pair does no work on a rigid motion", "[ENG-052]") {
    // With everything at rest the pair's generalized force has no component along the parent's coordinate or the
    // carriage's (the rigid motions of this tree): its halves act at the same points. Round-off: the components are
    // sums of products of the 2 kg weight's 19.6 N with unit vectors and 1 m arms, 1e-15 relative each.
    const CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 2.0, 0.05, true, ParentKind::Sliding);
    const MultibodyProvider provider(rig.model);
    const State state = start_state(provider, rig.carriage, 0.3);
    const Eigen::VectorXd q = provider.carried_generalized_forces(state.q);
    CHECK(std::abs(q[provider.velocity_offset(rig.parent)]) <= 1e-12);
    CHECK(q[provider.velocity_offset(rig.carriage)] == 0.0);
    CHECK(q.norm() > 1.0);
}

TEST_CASE("carried: validation refuses a wrong parent, a joint carried twice, a member body and a left span",
          "[ENG-052]") {
    const auto refuses = [](const MultibodyModel& model, const std::string& fragment) {
        try {
            const MultibodyProvider provider(model);
            FAIL("accepted");
        } catch (const InvalidModelError& error) {
            CAPTURE(error.cause());
            CHECK(error.cause().find(fragment) != std::string::npos);
        }
    };
    {
        CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1.0, 0.0, true, ParentKind::Fixed);
        rig.model.bodies.back().parent = -1;
        refuses(rig.model, "not a child of the member's parent");
    }
    {
        CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1.0, 0.0, true);
        static_cast<void>(add_lumped_beam(rig.model, rig.spec, -1, Eigen::Vector3d(0.0, 1.0, 0.0)));
        rig.model.members.back().carried.push_back(rig.carriage);
        refuses(rig.model, "more than one member");
    }
    {
        CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1.0, 0.0, false);
        rig.model.members.back().carried.push_back(rig.model.members.back().bodies.front());
        refuses(rig.model, "a member's own body");
    }
    {
        CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1.0, 0.0, false);
        rig.model.members.back().carried.push_back(99);
        refuses(rig.model, "is missing");
    }
    {
        // Past the end by 1 mm (beyond the 1e-6 m drift threshold): the run ends with the excursion named.
        const CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1.0, 0.0, true);
        MultibodyProvider provider(rig.model);
        State state = start_state(provider, rig.carriage, 1.001);
        SemiImplicitEuler integrator(StepConfig{1e-3});
        try {
            static_cast<void>(provider.step(integrator, state));
            FAIL("accepted");
        } catch (const InvalidModelError& error) {
            CHECK(error.cause().find("left the span by") != std::string::npos);
        }
        CHECK(state.t == 0.0);
    }
}
