#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/rope.hpp"
#include "mechsim/engine/transmission.hpp"

using mechsim::engine::Body;
using mechsim::engine::BodyOutput;
using mechsim::engine::JointType;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::Rope;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::Transmission;
using mechsim::engine::TransmissionOutput;
using mechsim::engine::TransmissionType;

// Energy oracle (brief M3-E6): with no drive and no input the scheme is backward Euler on a conservative system
// (gravity, the transmission's spring, the rope segments' springs) plus dissipation (dampers, the efficiency loss as
// an implicit shaft damping c >= 0 acting on the end-of-step rate, ruling 47) and inelastic velocity rows (limits,
// ruling 43; the self-locking row, ruling 48; both land a binding row's velocity on its target with impulse >= 0 and
// do no positive work). For the linear part the backward Euler step gives
//   E(q', v') - E(q, v) = -(dv^T M dv + dq^T K dq) / 2 - h v'^T D v' <= 0,
// so the total mechanical energy E (kinetic, gravitational, elastic) never rises from step to step; the nonlinear
// rest (rotation of rope segments, quaternion updates) is absent from these models or second order and dissipative
// for the same reason. A joint resting on its lower bound under a load that pushes into the bound is held by the
// anticipatory row with gap 0, s v' >= 0, so its coordinate is unchanged bit for bit.
// Round-off: energies here are at most about 10 J and each step's evaluation carries about 1e-15 relative, so a rise
// of 1e-12 J per step is beyond round-off and far below any injection the hand-off defect made (kJ).

namespace {

constexpr double gravity = 9.81;
constexpr double round_off = 1e-12;

Body shaft(double inertia) {
    Body body;
    body.parent = -1;
    body.joint.type = JointType::Revolute;
    body.joint.axis = Eigen::Vector3d::UnitZ();
    body.mass = 0.1;
    body.inertia = Eigen::Vector3d(inertia, inertia, inertia).asDiagonal();
    return body;
}

Body carriage(double mass) {
    Body body;
    body.parent = -1;
    body.joint.type = JointType::Prismatic;
    body.joint.axis = Eigen::Vector3d::UnitZ();
    body.joint.position = Eigen::Vector3d(0.5, 0.0, 0.0);
    body.joint.lower = Eigen::VectorXd::Constant(1, 0.0);
    body.joint.upper = Eigen::VectorXd::Constant(1, 0.5);
    body.mass = mass;
    body.inertia = 1e-2 * Eigen::Matrix3d::Identity();
    return body;
}

Transmission belt(double efficiency) {
    Transmission t;
    t.type = TransmissionType::BeltPulley;
    t.ratio = 0.01;
    t.efficiency = efficiency;
    t.back_efficiency = efficiency;
    t.stiffness = 1e5;
    t.damping = 0.0;
    return t;
}

Transmission screw() {
    Transmission t;
    t.type = TransmissionType::LeadScrew;
    t.ratio = 0.008;
    t.efficiency = 0.4;
    t.back_efficiency = 0.4;
    t.self_locking = true;
    t.stiffness = 1e6;
    t.damping = 0.0;
    return t;
}

Eigen::Vector3d world_point(const BodyOutput& body, const Eigen::Vector3d& local) {
    return body.position + (body.orientation * local);
}

Eigen::Vector3d point_velocity(const BodyOutput& body, const Eigen::Vector3d& local) {
    const Eigen::Vector3d omega = body.spatial_velocity.head<3>();
    return body.origin_velocity + omega.cross(body.orientation * local);
}

// Total mechanical energy: kinetic and gravitational of every body (centre of mass), the transmissions' springs
// k d^2 / 2 (d the deflection beyond the backlash, in contact), the rope segments' tension-only springs
// (EA / l)(d - l)^2 / 2 when taut (l = rest / N), and a one-DOF model joint spring K (q - rest)^2 / 2.
double mechanical_energy(const MultibodyProvider& provider, const State& state) {
    const MultibodyModel& model = provider.model();
    const std::vector<BodyOutput> bodies = provider.body_states(state.q, state.v);
    double energy = 0.0;
    for (std::size_t b = 0; b < bodies.size(); ++b) {
        const Body& body = model.bodies[b];
        const BodyOutput& out = bodies[b];
        const Eigen::Vector3d omega = out.spatial_velocity.head<3>();
        const Eigen::Vector3d com = world_point(out, body.com);
        const Eigen::Vector3d v = point_velocity(out, body.com);
        const Eigen::Matrix3d r = out.orientation.toRotationMatrix();
        const Eigen::Matrix3d inertia = r * body.inertia * r.transpose();
        energy += (0.5 * body.mass * v.squaredNorm()) + (0.5 * omega.dot(inertia * omega)) -
                  (body.mass * model.gravity.dot(com));
    }
    for (std::size_t b = 0; b < bodies.size(); ++b) {
        const Body& body = model.bodies[b];
        if (body.joint.stiffness.size() == 1) {
            const double q = state.q[provider.position_offset(static_cast<Eigen::Index>(b))];
            double rest = 0.0;
            if (body.joint.rest.size() == 1) {
                rest = body.joint.rest[0];
            }
            energy += 0.5 * body.joint.stiffness[0] * (q - rest) * (q - rest);
        }
    }
    const std::vector<TransmissionOutput> transmissions = provider.transmission_states(state.q, state.v);
    for (std::size_t i = 0; i < transmissions.size(); ++i) {
        const TransmissionOutput& t = transmissions[i];
        if (t.in_contact) {
            const double deflection = t.stretch - t.backlash_position;
            energy += 0.5 * model.transmissions[i].stiffness * deflection * deflection;
        }
    }
    for (const Rope& rope : model.ropes) {
        std::vector<Eigen::Vector3d> points;
        if (rope.from_body < 0) {
            points.push_back(rope.from_point);
        } else {
            points.push_back(world_point(bodies[static_cast<std::size_t>(rope.from_body)], rope.from_point));
        }
        for (const Eigen::Index node : rope.nodes) {
            points.push_back(bodies[static_cast<std::size_t>(node)].position);
        }
        points.push_back(world_point(bodies[static_cast<std::size_t>(rope.to_body)], rope.to_point));
        const double rest = rope.rest_length / rope.segments;
        const double k = rope.axial_rigidity / rest;
        for (std::size_t s = 0; s + 1 < points.size(); ++s) {
            const double stretch = (points[s + 1] - points[s]).norm() - rest;
            if (stretch > 0.0) {
                energy += 0.5 * k * stretch * stretch;
            }
        }
    }
    return energy;
}

// Belt (or screw) shaft and a carriage on its lower limit at 0, optionally with a 1 kg bob on a 2-segment rope 0.5 m
// below the carriage, placed at its loaded length so the rope stays taut (static stretch m g l / EA per segment, far
// above the swing of the axial mode started by the carriage's few-micrometre motion: it never moves at all).
struct Rig {
    MultibodyModel model;
    Eigen::Index bob = -1;
};

Rig rig(const Transmission& transmission, bool rope_load) {
    Rig out;
    out.model.gravity = Eigen::Vector3d(0.0, 0.0, -gravity);
    out.model.bodies.push_back(shaft(1e-4));
    out.model.bodies.push_back(carriage(2.0));
    out.model.transmissions.push_back(transmission);
    if (rope_load) {
        const double rest = 0.5;
        const double axial = 1e5;
        const double bob_mass = 1.0;
        const double loaded = rest + (gravity * rest * bob_mass / axial);
        Body bob;
        bob.parent = -1;
        bob.joint.type = JointType::Translation;
        bob.joint.position = Eigen::Vector3d(0.5, 0.0, -loaded);
        bob.mass = bob_mass;
        bob.inertia = 1e-3 * Eigen::Matrix3d::Identity();
        out.model.bodies.push_back(bob);
        out.bob = static_cast<Eigen::Index>(out.model.bodies.size()) - 1;
        Rope rope;
        rope.from_body = 1;
        rope.to_body = out.bob;
        rope.segments = 2;
        rope.axial_rigidity = axial;
        rope.damping = 0.0;
        rope.mass_per_length = 1e-3;
        rope.rest_length = rest;
        static_cast<void>(
            mechsim::engine::add_rope(out.model, rope, Eigen::Vector3d(0.5, 0.0, 0.0), bob.joint.position));
    }
    return out;
}

struct EnergyRun {
    double worst_rise = -1.0;  // largest E(n + 1) - E(n) over the steps
    bool seated = true;        // the carriage coordinate stayed exactly 0
};

EnergyRun run_energy(MultibodyProvider& provider, State& state, double h, int steps) {
    SemiImplicitEuler integrator(StepConfig{h});
    EnergyRun run;
    run.worst_rise = -std::numeric_limits<double>::infinity();
    double before = mechanical_energy(provider, state);
    for (int n = 0; n < steps; ++n) {
        static_cast<void>(provider.step(integrator, state));
        const double after = mechanical_energy(provider, state);
        run.worst_rise = std::max(run.worst_rise, after - before);
        before = after;
        if (state.q[1] != 0.0) {
            run.seated = false;
        }
    }
    return run;
}

}  // namespace

TEST_CASE("energy: a limit and a compliant lossy belt with no drive never gain energy", "[ENG-013][ENG-043]") {
    // The belt starts stretched by -1e-4 m (n theta = -1e-4, the carriage at its bound), pressing the carriage into
    // the limit with k 1e-4 = 10 N; released, the free shaft swings the stretch through +-1e-4 m, so the belt never
    // pulls up harder than 10 N < m g = 19.6 N and the carriage stays seated. The swing passes zero speed and
    // reverses the power flow every half period, at h = 1 ms and 0.1 ms (the adapter's 10 substeps).
    for (const double h : {1e-3, 1e-4}) {
        CAPTURE(h);
        const Rig r = rig(belt(0.8), false);
        MultibodyProvider provider(r.model);
        State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
        state.q[0] = -1e-4 / 0.01;
        const EnergyRun run = run_energy(provider, state, h, 2000);
        CAPTURE(run.worst_rise);
        CHECK(run.worst_rise <= round_off);
        CHECK(run.seated);
    }
}

TEST_CASE("energy: a limit and a self-locking screw with no drive never gain energy", "[ENG-013][ENG-044]") {
    // The shaft starts turning at -2 rad/s, driving the carriage down into its bound: the screw compresses until the
    // shaft stops; back-driving is then held by the lock row (no own torque). Nothing may rise, the carriage stays
    // seated (screw force and gravity both push down or hold).
    for (const double h : {1e-3, 1e-4}) {
        CAPTURE(h);
        const Rig r = rig(screw(), false);
        MultibodyProvider provider(r.model);
        State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
        state.v[0] = -2.0;
        const EnergyRun run = run_energy(provider, state, h, 2000);
        CAPTURE(run.worst_rise);
        CHECK(run.worst_rise <= round_off);
        CHECK(run.seated);
    }
}

TEST_CASE("energy: a drive cleared mid-motion against a limit adds no energy, with and without a rope load",
          "[ENG-013][ENG-043][ENG-047][ENG-051]") {
    // A drive (K 1 N m/rad, rest -0.1 rad, D 0.01) presses the carriage into its bound through the lossy belt (the
    // gantry's stepper preload, G12): shaft torque 0.1 N m, belt force 10 N < m g. While it moves the drive is
    // cleared (clear_joint_drive); from then on there is no drive and no input and E never rises; the stored belt
    // energy swings the free shaft with |F| <= 10 N, so the carriage stays seated. With the rope load the bob's
    // 9.81 N adds to the push into the bound.
    for (const bool rope_load : {false, true}) {
        for (const double h : {1e-3, 1e-4}) {
            CAPTURE(rope_load, h);
            const Rig r = rig(belt(0.8), rope_load);
            MultibodyProvider provider(r.model);
            State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
            SemiImplicitEuler integrator(StepConfig{h});
            const int pressing = static_cast<int>(0.0123 / h);
            for (int n = 0; n < pressing; ++n) {
                provider.set_joint_drive(0, 0, -0.1, 1.0, 0.01);
                static_cast<void>(provider.step(integrator, state));
                REQUIRE(state.q[1] == 0.0);
            }
            REQUIRE(state.v[0] != 0.0);
            provider.clear_joint_drive(0, 0);
            const EnergyRun run = run_energy(provider, state, h, 2000);
            CAPTURE(run.worst_rise);
            CHECK(run.worst_rise <= round_off);
            CHECK(run.seated);
        }
    }
}

TEST_CASE("energy: a limit, a lossy belt and a stiff shaft spring with no drive never gain energy",
          "[ENG-013][ENG-043][ENG-051]") {
    // A held brake's law as a model joint spring (K = 30 N m/rad, rest -0.02 rad: no drive, no input) against the
    // belt (k n^2 = 10 N m/rad): the shaft swings at sqrt(40 / 1e-4) = 632 rad/s about theta* = K rest / (K + k n^2)
    // = -0.015 from theta = 0, so the belt force k n theta stays in [-30 N, 0] (into the bound; the carriage stays
    // seated) while the shaft speed reverses every half period under a non-zero force: the power flow reverses at
    // speed with the load on, the case where an efficiency gain on the end-of-step force put energy in.
    for (const double h : {1e-3, 1e-4}) {
        CAPTURE(h);
        Rig r = rig(belt(0.8), false);
        r.model.bodies[0].joint.stiffness = Eigen::VectorXd::Constant(1, 30.0);
        r.model.bodies[0].joint.damping = Eigen::VectorXd::Constant(1, 0.0);
        r.model.bodies[0].joint.rest = Eigen::VectorXd::Constant(1, -0.02);
        MultibodyProvider provider(r.model);
        State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
        const EnergyRun run = run_energy(provider, state, h, 2000);
        CAPTURE(run.worst_rise);
        CHECK(run.worst_rise <= round_off);
        CHECK(run.seated);
    }
}
