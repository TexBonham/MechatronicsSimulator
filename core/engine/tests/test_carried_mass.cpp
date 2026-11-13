#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>
#include <vector>

#include "carried_helpers.hpp"
#include "energy_helpers.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/member_loads.hpp"
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
    const CarriedRig heavier = carried_rig(BeamEnds::SimplySupported, 8, 2.0 * mass, 0.05, true, ParentKind::Sliding);
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

namespace {

struct MomentumRun {
    State state;
    mechsim::engine::MemberOutput member;
};

// A loaded pinned member under gravity from straight, undamped (it rings), the carriage at 0.3 m with speed c.
MomentumRun momentum_run(bool momentum, double speed, int steps) {
    CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1.0, 0.0, true, ParentKind::World, 0.0);
    MultibodyConfig config;
    config.carried_momentum = momentum;
    MultibodyProvider provider(rig.model, config);
    State state = start_state(provider, rig.carriage, 0.3);
    state.v[provider.velocity_offset(rig.carriage)] = speed;
    SemiImplicitEuler integrator(StepConfig{1e-4});
    for (int n = 0; n < steps; ++n) {
        static_cast<void>(provider.step(integrator, state));
    }
    return MomentumRun{state, provider.outputs().members.front()};
}

}  // namespace

TEST_CASE("carried mass: a standing carriage steps bit-identically with and without the momentum form; a moving one "
          "does not",
          "[ENG-052]") {
    // member_loads.hpp, Carried mass, the momentum form: the force -(B_n - B'_{n-1}) v_n / h is skipped when s_n =
    // s_{n-1} exactly, so a standing carriage is the block of brief M3-E7d to the last bit; a moving one changes B and
    // carries its momentum, a different step.
    const MomentumRun with = momentum_run(true, 0.0, 2000);
    const MomentumRun without = momentum_run(false, 0.0, 2000);
    CHECK(with.state.q == without.state.q);
    CHECK(with.state.v == without.state.v);
    CHECK(with.member.start_force == without.member.start_force);
    CHECK(with.member.end_force == without.member.end_force);
    CHECK(with.member.carried_work == without.member.carried_work);
    const MomentumRun moving = momentum_run(true, 0.2, 2000);
    const MomentumRun velocity_form = momentum_run(false, 0.2, 2000);
    CHECK(moving.state.v != velocity_form.state.v);
}

TEST_CASE("carried mass: the member's end loads include the carried mass's inertial force", "[ENG-052]") {
    // A free body of the member (its bodies root to tip, the pin segment included; supports at both ends; the pair's
    // force along member y = world y): the support forces' y sum is S = (m_member + m_c) g + sum_i m_i a_i + m_c a_p,
    // the static weights (the carriage's through the pair, its reaction m_c g: it does not move across the member) and
    // the inertial forces, m_c a_p the carried block's (a_p the lever-weighted acceleration of the two load nodes, the
    // parent the world). Measured: a_i = J(q_n)(v_{n+1} - v_n) / h at each body's centre of mass and node point, the
    // scheme's q'' at the start configuration the reactions use; the reactions also carry the velocity-product
    // accelerations J' v, which the measurement leaves out: at most (l / 2) |w_i|^2 per segment (the centripetal
    // acceleration of its centre about its joint), so the tolerance is sum m_i (l / 2) |w_i|^2 + 1e-9 of the weight.
    // A carriage of 10 m_beam at midspan, undamped, released straight under gravity: it rings at full amplitude.
    const CarriedRig probe = carried_rig(BeamEnds::SimplySupported, 8, 1.0, 0.0, true, ParentKind::World, 0.0);
    const double member_mass = probe.spec.density * probe.spec.area * probe.spec.length;
    const CarriedRig rig =
        carried_rig(BeamEnds::SimplySupported, 8, 10.0 * member_mass, 0.0, true, ParentKind::World, 0.0);
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, 0.5);
    const mechsim::engine::LumpedBeam& beam = rig.model.members.front();
    const std::vector<mechsim::engine::LoadNode> nodes = mechsim::engine::load_nodes(beam);
    const double carried = rig.model.bodies[static_cast<std::size_t>(rig.carriage)].mass;
    double bodies_mass = 0.0;
    for (const Eigen::Index b : beam.bodies) {
        bodies_mass += rig.model.bodies[static_cast<std::size_t>(b)].mass;
    }
    const double g = -rig.model.gravity.y();
    const double h = 1e-4;
    const double l = beam.length / 8.0;
    SemiImplicitEuler integrator(StepConfig{h});
    double worst = 0.0;
    double largest_inertial = 0.0;
    for (int n = 0; n < 600; ++n) {
        const State before = state;
        static_cast<void>(provider.step(integrator, state));
        const std::vector<mechsim::engine::BodyOutput> at_start = provider.body_states(before.q, before.v);
        const std::vector<mechsim::engine::BodyOutput> at_end = provider.body_states(before.q, state.v);
        double inertial = 0.0;
        double centripetal = 0.0;
        for (const Eigen::Index b : beam.bodies) {
            const auto i = static_cast<std::size_t>(b);
            const Eigen::Vector3d com = rig.model.bodies[i].com;
            const double dv = mechsim::engine::test::point_velocity(at_end[i], com).y() -
                              mechsim::engine::test::point_velocity(at_start[i], com).y();
            inertial += rig.model.bodies[i].mass * dv / h;
            const double spin = at_start[i].spatial_velocity.head<3>().squaredNorm();
            centripetal += rig.model.bodies[i].mass * 0.5 * l * spin;
        }
        const mechsim::engine::CarriedLoadOutput& pair = provider.outputs().carried.front();
        double foot = 0.0;
        for (Eigen::Index a = 0; a < 2; ++a) {
            const mechsim::engine::LoadNode& node = nodes[static_cast<std::size_t>(pair.node + a)];
            double weight = pair.weight;
            if (a == 1) {
                weight = 1.0 - pair.weight;
            }
            if (node.body < 0) {
                continue;
            }
            const auto i = static_cast<std::size_t>(node.body);
            foot += weight * (mechsim::engine::test::point_velocity(at_end[i], node.point).y() -
                              mechsim::engine::test::point_velocity(at_start[i], node.point).y()) /
                    h;
        }
        const mechsim::engine::MemberOutput& member = provider.outputs().members.front();
        const double supports = member.start_force.y() + member.end_force.y();
        const double expected = ((bodies_mass + carried) * g) + inertial + (carried * foot);
        const double tolerance = centripetal + (1e-9 * (bodies_mass + carried) * g);
        worst = std::max(worst, std::abs(supports - expected) / tolerance);
        largest_inertial = std::max(largest_inertial, std::abs(carried * foot));
    }
    std::printf("end loads: largest m_c a_p %.6e N; worst |S - expected| over its tolerance %.4f\n", largest_inertial,
                worst);
    CHECK(worst <= 1.0);
    CHECK(largest_inertial > 0.5 * carried * g);
}
