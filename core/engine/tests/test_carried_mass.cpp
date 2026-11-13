#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>
#include <vector>

#include "carried_helpers.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/multibody_model.hpp"
#include "mechsim/engine/multibody_provider.hpp"

using mechsim::engine::BeamEnds;
using mechsim::engine::Body;
using mechsim::engine::MultibodyConfig;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::carried_rig;
using mechsim::engine::test::CarriedRig;
using mechsim::engine::test::ParentKind;
using mechsim::engine::test::start_state;

// The carried mass on the member (ENG-052, brief M3-E7d; member_loads.hpp, Carried mass): B = m_c G^T G, inertia
// only, nothing along a rigid motion of the parent, and nothing at all without carried joints.

namespace {

double model_mass(const CarriedRig& rig) {
    double m = 0.0;
    for (const Body& body : rig.model.bodies) {
        m += body.mass;
    }
    return m;
}

}  // namespace

TEST_CASE("carried mass: the block is m_c G^T G, symmetric, rank one, zero along the parent's and the carriage's motion",
          "[ENG-052]") {
    // G v = u'(p): the parent's coordinate (a translation across the member, every node moving with it) and the
    // carriage's (it moves no node) give G e = 0, so B e = 0, to the Jacobians' round-off for the parent (the node
    // rotations applied and undone, 1e-12 of the trace) and exactly for the carriage. m_c G^T G has the
    // one eigenvalue m_c |G|^2, the trace; every other eigenvalue is zero to round-off and none is negative.
    const double mass = 2.0;
    const CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, mass, 0.05, true, ParentKind::Sliding);
    const MultibodyProvider provider(rig.model);
    const State state = start_state(provider, rig.carriage, 0.37);
    const Eigen::MatrixXd block = provider.carried_mass_matrix(state.q);
    const Eigen::Index parent = provider.velocity_offset(rig.parent);
    const Eigen::Index carriage = provider.velocity_offset(rig.carriage);
    CHECK((block - block.transpose()).norm() == 0.0);
    CHECK(block.col(parent).norm() <= 1e-12 * block.trace());
    CHECK(block.col(carriage).norm() == 0.0);
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(block);
    const Eigen::VectorXd& values = solver.eigenvalues();
    const double largest = values.maxCoeff();
    CHECK(largest > 0.0);
    CHECK(std::abs(largest - block.trace()) <= 1e-12 * largest);
    for (Eigen::Index k = 0; k + 1 < values.size(); ++k) {
        CHECK(std::abs(values[k]) <= 1e-12 * largest);
    }
    // G depends on the geometry only: the block scales with m_c.
    CarriedRig heavier = carried_rig(BeamEnds::SimplySupported, 8, 2.0 * mass, 0.05, true, ParentKind::Sliding);
    const MultibodyProvider doubled(heavier.model);
    const Eigen::MatrixXd twice = doubled.carried_mass_matrix(start_state(doubled, heavier.carriage, 0.37).q);
    CHECK((twice - (2.0 * block)).norm() <= 1e-15 * block.norm());
}

TEST_CASE("carried mass: nothing without carried joints", "[ENG-052]") {
    // A model whose members carry nothing never enters the block: its mass matrix, steps and outputs are those of
    // the engine before the carried mass, bit for bit, whatever the test seam says.
    const CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 2.0, 0.05, false, ParentKind::Sliding);
    std::vector<State> ends;
    for (const bool added : {true, false}) {
        MultibodyConfig config;
        config.carried_added_mass = added;
        MultibodyProvider provider(rig.model, config);
        CHECK(provider.carried_mass_matrix(provider.neutral_positions()).norm() == 0.0);
        State state = start_state(provider, rig.carriage, 0.3);
        SemiImplicitEuler integrator(StepConfig{1e-3});
        for (int n = 0; n < 300; ++n) {
            provider.apply_body_wrench(rig.parent, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 20.0, 0.0));
            static_cast<void>(provider.step(integrator, state));
        }
        ends.push_back(state);
    }
    CHECK(ends[0].q == ends[1].q);
    CHECK(ends[0].v == ends[1].v);
}

TEST_CASE("carried mass: on a rigid member the parent accelerates as m_p + m_c, not m_p + 2 m_c", "[ENG-052]") {
    // A two-segment cantilever on the sliding parent with every node joint locked (lower == upper == 0; two lock
    // rows, the sweep cap raised so the coupled rows meet the tolerance) is rigid: G v = 0 on the
    // constrained motion, so B adds nothing and the tree accelerates as one body, a = F / (m_p + m_c + m_b) along y,
    // gravity off. Each step's lock rows leave |J v| <= 1e-9 rad/s (ENG-025), a velocity error the next step carries:
    // over N steps at most N 1e-9 against F t / M = 1.28 m/s, 1.6e-7 relative here. The uncarried run (no pair, no
    // block) has the same rows and the same sweep, asserted 1e-9 relative (measured equal to that). The double
    // count m_p + 2 m_c would be 25 % off.
    const double force = 50.0;
    const double h = 1e-3;
    const int steps = 200;
    std::vector<double> velocity;
    double total = 0.0;
    for (const bool carry : {false, true}) {
        CarriedRig rig = carried_rig(BeamEnds::Cantilever, 2, 2.0, 0.0, carry, ParentKind::Sliding);
        rig.model.gravity = Eigen::Vector3d::Zero();
        for (const Eigen::Index b : rig.model.members.front().bodies) {
            Body& body = rig.model.bodies[static_cast<std::size_t>(b)];
            body.joint.lower = Eigen::VectorXd::Zero(1);
            body.joint.upper = Eigen::VectorXd::Zero(1);
        }
        total = model_mass(rig);
        MultibodyConfig config;
        config.iteration_cap = 1000;
        MultibodyProvider provider(rig.model, config);
        State state = start_state(provider, rig.carriage, 0.6);
        SemiImplicitEuler integrator(StepConfig{h});
        for (int n = 0; n < steps; ++n) {
            provider.apply_body_wrench(rig.parent, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, force, 0.0));
            static_cast<void>(provider.step(integrator, state));
        }
        velocity.push_back(state.v[provider.velocity_offset(rig.parent)]);
    }
    const double expected = force * h * steps / total;
    std::printf("carried mass, rigid member: parent velocity %.15e (carried) %.15e (uncarried), F t / (m_p + m_c + "
                "m_b) %.15e\n",
                velocity[1], velocity[0], expected);
    CHECK(std::abs(velocity[1] - expected) <= steps * 1e-9);
    CHECK(std::abs(velocity[1] / velocity[0] - 1.0) <= 1e-9);
}

TEST_CASE("carried mass: a loaded run is deterministic", "[ENG-052]") {
    std::vector<State> ends;
    for (int run = 0; run < 2; ++run) {
        const CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 2.0, 0.05, true, ParentKind::Sliding);
        MultibodyProvider provider(rig.model);
        State state = start_state(provider, rig.carriage, 0.3);
        state.v[provider.velocity_offset(rig.carriage)] = 0.5;
        SemiImplicitEuler integrator(StepConfig{1e-3});
        for (int n = 0; n < 300; ++n) {
            static_cast<void>(provider.step(integrator, state));
        }
        ends.push_back(state);
    }
    CHECK(ends[0].q == ends[1].q);
    CHECK(ends[0].v == ends[1].v);
}
