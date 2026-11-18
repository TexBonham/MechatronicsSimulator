#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <limits>

#include "mechsim/engine/dynamics_provider.hpp"
#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/member_loads.hpp"
#include "mechsim/engine/multibody_model.hpp"
#include "mechsim/engine/multibody_provider.hpp"

using mechsim::engine::BeamEnds;
using mechsim::engine::BeamSpec;
using mechsim::engine::Body;
using mechsim::engine::ConstraintType;
using mechsim::engine::DynamicsTerms;
using mechsim::engine::InvalidModelError;
using mechsim::engine::JointType;
using mechsim::engine::LoopConstraint;
using mechsim::engine::LumpedBeam;
using mechsim::engine::ModelChange;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;

// Mid-run model changes (ENG-031 to ENG-034, PHY-008) and member registration (ENG-030): unit behaviour, exact
// values (the changes write the spring arrays and constraint rows directly; no tolerance beyond rounding).

namespace {

BeamSpec beam_spec(BeamEnds ends) {
    BeamSpec spec;
    spec.youngs_modulus = 2.0e11;
    spec.area_moment = 1e-10;
    spec.density = 7850.0;
    spec.area = 1e-4;
    spec.length = 1.0;
    spec.segments = 4;
    spec.ends = ends;
    return spec;
}

// Body 0 fixed to the world at the origin; body 1 on a vertical prismatic joint held to body 0 by a point constraint.
MultibodyModel held_load() {
    MultibodyModel model;
    Body anchor;
    anchor.name = "anchor";
    anchor.joint.type = JointType::Fixed;
    model.bodies.push_back(anchor);
    Body load;
    load.name = "load";
    load.joint.type = JointType::Prismatic;
    load.joint.axis = Eigen::Vector3d::UnitZ();
    load.mass = 2.0;
    model.bodies.push_back(load);
    LoopConstraint hold;
    hold.type = ConstraintType::Point;
    hold.body_a = 1;
    hold.body_b = 0;
    model.constraints.push_back(hold);
    return model;
}

State rest_state(const MultibodyProvider& provider) {
    return State{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
}

}  // namespace

TEST_CASE("every lumped beam is registered as a member and publishes end-loads each step", "[ENG-030]") {
    MultibodyModel model;
    const LumpedBeam cantilever = add_lumped_beam(model, beam_spec(BeamEnds::Cantilever));
    const LumpedBeam supported = add_lumped_beam(model, beam_spec(BeamEnds::SimplySupported));
    REQUIRE(model.members.size() == 2);
    REQUIRE(model.members[0].bodies == cantilever.bodies);
    REQUIRE(member_nodes(cantilever) == cantilever.bodies);
    const auto nodes = member_nodes(supported);
    REQUIRE(nodes.size() == 4);
    REQUIRE(nodes.front() == supported.bodies[1]);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(StepConfig{});
    State state = rest_state(provider);
    REQUIRE(provider.outputs().members.empty());
    provider.step(integrator, state);
    REQUIRE(provider.outputs().members.size() == 2);
    provider.step(integrator, state);
    REQUIRE(provider.outputs().members.size() == 2);
    REQUIRE(provider.outputs().members[0].start_force.allFinite());
}

TEST_CASE("a member referencing a missing body or constraint is rejected", "[ENG-030]") {
    MultibodyModel model;
    static_cast<void>(add_lumped_beam(model, beam_spec(BeamEnds::Cantilever)));
    SECTION("body") {
        model.members[0].bodies.push_back(99);
        REQUIRE_THROWS_AS(MultibodyProvider(model), InvalidModelError);
    }
    SECTION("constraint") {
        model.members[0].roller_constraint = 3;
        REQUIRE_THROWS_AS(MultibodyProvider(model), InvalidModelError);
    }
    SECTION("empty") {
        model.members[0].bodies.clear();
        REQUIRE_THROWS_AS(MultibodyProvider(model), InvalidModelError);
    }
}

TEST_CASE("a spring offset sets the node's rest angle and is listed at the next step", "[ENG-031]") {
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    const LumpedBeam beam = add_lumped_beam(model, beam_spec(BeamEnds::Cantilever));
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(StepConfig{});
    State state = rest_state(provider);
    provider.step(integrator, state);
    const double offset = 0.01;
    provider.set_spring_offset(0, 2, offset);
    provider.set_spring_offset(0, 2, 2.0 * offset);
    DynamicsTerms terms;
    const Eigen::VectorXd q = Eigen::VectorXd::Zero(provider.position_size());
    provider.evaluate(0.0, q, Eigen::VectorXd::Zero(provider.velocity_size()), terms);
    const Eigen::Index dof = provider.velocity_offset(beam.bodies[2]);
    REQUIRE(terms.force[dof] == beam.node_stiffness * 2.0 * offset);
    REQUIRE(terms.force[provider.velocity_offset(beam.bodies[1])] == 0.0);
    provider.step(integrator, state);
    const auto& events = provider.outputs().events;
    REQUIRE(events.size() == 2);
    REQUIRE(events[1].kind == ModelChange::SpringOffset);
    REQUIRE(events[1].target == 0);
    REQUIRE(events[1].index == 2);
    REQUIRE(events[1].value == 2.0 * offset);
    REQUIRE(events[1].time == StepConfig{}.h);
    provider.step(integrator, state);
    REQUIRE(provider.outputs().events.empty());
}

TEST_CASE("a spring offset on a missing member or node or not finite is rejected", "[ENG-031]") {
    MultibodyModel model;
    static_cast<void>(add_lumped_beam(model, beam_spec(BeamEnds::SimplySupported)));
    MultibodyProvider provider(model);
    REQUIRE_THROWS_AS(provider.set_spring_offset(1, 0, 0.1), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_spring_offset(-1, 0, 0.1), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_spring_offset(0, 4, 0.1), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_spring_offset(0, -1, 0.1), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_spring_offset(0, 0, std::numeric_limits<double>::quiet_NaN()),
                      InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_spring_offset(0, 0, std::numeric_limits<double>::infinity()), InvalidModelError);
}

TEST_CASE("a spring stiffness change writes K and is listed at the next step", "[ENG-032]") {
    MultibodyModel model;
    const LumpedBeam beam = add_lumped_beam(model, beam_spec(BeamEnds::SimplySupported));
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(StepConfig{});
    State state = rest_state(provider);
    const auto nodes = member_nodes(beam);
    provider.set_spring_stiffness(0, 0, 0.0);
    provider.set_spring_stiffness(0, 3, 0.5 * beam.node_stiffness);
    DynamicsTerms terms;
    provider.evaluate(0.0, state.q, state.v, terms);
    REQUIRE(terms.stiffness(provider.velocity_offset(nodes[0]), provider.velocity_offset(nodes[0])) == 0.0);
    REQUIRE(terms.stiffness(provider.velocity_offset(nodes[3]), provider.velocity_offset(nodes[3])) ==
            0.5 * beam.node_stiffness);
    REQUIRE(terms.stiffness(provider.velocity_offset(nodes[1]), provider.velocity_offset(nodes[1])) ==
            beam.node_stiffness);
    provider.step(integrator, state);
    REQUIRE(provider.outputs().events.size() == 2);
    REQUIRE(provider.outputs().events[0].kind == ModelChange::SpringStiffness);
    REQUIRE(provider.outputs().events[0].time == 0.0);
    REQUIRE(provider.outputs().events[1].value == 0.5 * beam.node_stiffness);
    REQUIRE_THROWS_AS(provider.set_spring_stiffness(0, 0, -1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_spring_stiffness(0, 0, std::numeric_limits<double>::quiet_NaN()),
                      InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_spring_stiffness(0, 0, std::numeric_limits<double>::infinity()),
                      InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_spring_stiffness(0, 4, 1.0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_spring_stiffness(2, 0, 1.0), InvalidModelError);
}

// Held, the load does not move; after the disable its step is free fall, dv = -g h exactly (the only force).
TEST_CASE("a disabled constraint stops acting from the next step", "[ENG-033]") {
    MultibodyProvider provider(held_load());
    SemiImplicitEuler integrator(StepConfig{});
    State state = rest_state(provider);
    const double h = integrator.config().h;
    provider.step(integrator, state);
    REQUIRE(std::abs(state.v[0]) <= 1e-9);
    REQUIRE(std::abs(provider.outputs().constraints[0].force.z() - 2.0 * 9.81) <= 1e-9);
    REQUIRE(provider.constraint_enabled(0));
    provider.disable_constraint(0);
    REQUIRE_FALSE(provider.constraint_enabled(0));
    const double before = state.v[0];
    provider.step(integrator, state);
    REQUIRE(std::abs((state.v[0] - before) / h + 9.81) <= 1e-12);
    REQUIRE(provider.outputs().constraints[0].force.norm() == 0.0);
    REQUIRE(provider.outputs().constraints[0].translational_error == 0.0);
    REQUIRE(provider.outputs().events.size() == 1);
    REQUIRE(provider.outputs().events[0].kind == ModelChange::ConstraintDisabled);
    REQUIRE(provider.outputs().events[0].target == 0);
    for (int n = 0; n < 100; ++n) {
        provider.step(integrator, state);
    }
    REQUIRE(state.q[0] < -0.05);
    REQUIRE_THROWS_AS(provider.disable_constraint(0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.disable_constraint(1), InvalidModelError);
    REQUIRE_THROWS_AS(provider.disable_constraint(-1), InvalidModelError);
    REQUIRE_THROWS_AS(static_cast<void>(provider.constraint_enabled(1)), InvalidModelError);
}

TEST_CASE("a released load falls free and the release is listed", "[ENG-034]") {
    MultibodyProvider provider(held_load());
    SemiImplicitEuler integrator(StepConfig{});
    State state = rest_state(provider);
    const double h = integrator.config().h;
    provider.step(integrator, state);
    provider.release_load(0);
    const double before = state.v[0];
    provider.step(integrator, state);
    REQUIRE(std::abs((state.v[0] - before) / h + 9.81) <= 1e-12);
    REQUIRE(provider.outputs().events.size() == 1);
    REQUIRE(provider.outputs().events[0].kind == ModelChange::LoadReleased);
    REQUIRE(provider.outputs().events[0].time == h);
    REQUIRE_THROWS_AS(provider.release_load(0), InvalidModelError);
    REQUIRE_THROWS_AS(provider.release_load(5), InvalidModelError);
}

TEST_CASE("a disabled hinge publishes zero force on all five rows", "[ENG-033]") {
    MultibodyModel model = held_load();
    model.constraints[0].type = ConstraintType::Hinge;
    model.bodies[1].joint.type = JointType::Prismatic;
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator(StepConfig{});
    State state = rest_state(provider);
    provider.disable_constraint(0);
    provider.step(integrator, state);
    REQUIRE(provider.outputs().constraints[0].force.size() == 5);
    REQUIRE(provider.outputs().constraints[0].force.norm() == 0.0);
    REQUIRE(provider.outputs().constraints[0].rotational_error == 0.0);
}
