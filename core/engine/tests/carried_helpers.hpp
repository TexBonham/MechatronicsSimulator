#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

#include "beam_helpers.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/state.hpp"
#include "mechsim/engine/step_config.hpp"

namespace mechsim::engine::test {

// Carried-joint rigs (ENG-052): a 1 m steel bar (beam_helpers) along world x with gravity -y, in its bending plane
// (member frame y = world y, ruling 65), and a carriage on a prismatic joint along x on the same parent, its origin
// offset by `offset` along y from the member axis.
//
// Settling. The node dampers are Rayleigh stiffness-proportional: every hinge has spring k = E I / l and damper c, so
// D = (c / k) K = beta K and the chain's modes stay decoupled with damping ratio zeta_i = beta omega_i / 2. With
// beta = 2 / omega_1 the first mode is critical and every higher mode overdamped; an overdamped mode's slow root
// omega (zeta - sqrt(zeta^2 - 1)) >= omega / (2 zeta) = 1 / beta and the critical one decays as (1 + omega t)
// e^(-omega t) with omega = 2 / beta, so every transient is below e^(-t / beta) times its start (times 1 + 2 t / beta).
// After 40 beta it is below 1e-15 of the load's deflection; backward Euler only adds damping. omega_1 is the
// continuum closed form (beta_1 L)^2 sqrt(E I / (rho A L^4)): cantilever 1.87510^2, pinned-pinned pi^2, clamped-clamped
// 4.73004^2; the chain's own first mode is within 2.5 % of it at N >= 8 (lumped_beam.hpp), a settling margin.
// A carried carriage loads the member's inertia (brief M3-E7d): omega_1 is then the one-mode estimate at the most
// compliant point, sqrt(k / (m_c + f m_beam)) (test_carried_mass.cpp: k = 48, 3, 192 E I / L^3 at midspan, tip,
// midspan; f = 17/35, 33/140, 13/35), capped at the unloaded value. Settling needs only omega_1 >= omega / sqrt(2):
// a mode with omega < 2 / beta is underdamped and decays at zeta omega = omega^2 beta / 2 >= 1 / beta exactly then, so
// the bound e^(-t / beta) holds with a 29 % margin on the estimate (the carriage elsewhere only stiffens the mode).
enum class ParentKind : std::uint8_t { World, Fixed, Sliding };

struct CarriedRig {
    MultibodyModel model;
    BeamSpec spec;
    Eigen::Index parent = -1;
    Eigen::Index carriage = -1;
    double beta = 0.0;  // node damping / node stiffness, s
};

// k L^3 / (E I) at the most compliant point and the one-mode effective share f of the member's mass there
// (Rayleigh, the static deflection shape; derivation in test_carried_mass.cpp).
inline double point_stiffness_factor(BeamEnds ends) {
    if (ends == BeamEnds::Cantilever) {
        return 3.0;
    }
    if (ends == BeamEnds::SimplySupported) {
        return 48.0;
    }
    return 192.0;
}

inline double effective_mass_share(BeamEnds ends) {
    if (ends == BeamEnds::Cantilever) {
        return 33.0 / 140.0;
    }
    if (ends == BeamEnds::SimplySupported) {
        return 17.0 / 35.0;
    }
    return 13.0 / 35.0;
}

inline double first_mode_factor(BeamEnds ends) {
    if (ends == BeamEnds::Cantilever) {
        return 1.8751040687 * 1.8751040687;
    }
    if (ends == BeamEnds::SimplySupported) {
        return std::numbers::pi * std::numbers::pi;
    }
    return 4.7300407449 * 4.7300407449;
}

constexpr double rig_gravity = 9.81;
constexpr double parent_mass = 5.0;

// The parent: the world, a 5 kg body fixed to the world, or a 5 kg body on a world prismatic joint along y (across
// the member, in its bending plane).
inline CarriedRig carried_rig(BeamEnds ends, int segments, double carriage_mass, double offset, bool carry,
                              ParentKind kind = ParentKind::World, double damping_scale = 1.0) {
    CarriedRig rig;
    rig.model.gravity = Eigen::Vector3d(0.0, -rig_gravity, 0.0);
    if (kind != ParentKind::World) {
        Body parent;
        parent.parent = -1;
        parent.joint.type = JointType::Fixed;
        if (kind == ParentKind::Sliding) {
            parent.joint.type = JointType::Prismatic;
            parent.joint.axis = Eigen::Vector3d::UnitY();
        }
        parent.mass = parent_mass;
        parent.inertia = 0.1 * Eigen::Matrix3d::Identity();
        rig.model.bodies.push_back(parent);
        rig.parent = 0;
    }
    rig.spec = steel_bar(1.0, ends, segments);
    double omega = first_mode_factor(ends) * frequency_scale(rig.spec);
    if (carry) {
        const double length = rig.spec.length;
        const double stiffness = point_stiffness_factor(ends) * flexural_rigidity(rig.spec) / (length * length * length);
        const double member = rig.spec.density * rig.spec.area * length;
        omega = std::min(omega, std::sqrt(stiffness / (carriage_mass + (effective_mass_share(ends) * member))));
    }
    rig.beta = 2.0 / omega;
    rig.spec.node_damping = damping_scale * rig.beta * flexural_rigidity(rig.spec) / (rig.spec.length / segments);
    static_cast<void>(add_lumped_beam(rig.model, rig.spec, rig.parent));
    Body carriage;
    carriage.parent = rig.parent;
    carriage.joint.type = JointType::Prismatic;
    carriage.joint.axis = Eigen::Vector3d::UnitX();
    carriage.joint.position = Eigen::Vector3d(0.0, offset, 0.0);
    carriage.mass = carriage_mass;
    carriage.inertia = 1e-3 * Eigen::Matrix3d::Identity();
    rig.model.bodies.push_back(carriage);
    rig.carriage = static_cast<Eigen::Index>(rig.model.bodies.size()) - 1;
    if (carry) {
        rig.model.members.back().carried.push_back(rig.carriage);
    }
    return rig;
}

inline State start_state(const MultibodyProvider& provider, Eigen::Index carriage, double position) {
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    state.q[provider.position_offset(carriage)] = position;
    return state;
}

inline void run_steps(MultibodyProvider& provider, State& state, int steps, double h = 1e-3) {
    SemiImplicitEuler integrator(StepConfig{h});
    for (int n = 0; n < steps; ++n) {
        static_cast<void>(provider.step(integrator, state));
    }
}

inline int settling_steps(const CarriedRig& rig, double h = 1e-3) {
    return static_cast<int>(std::ceil(40.0 * rig.beta / h));
}

// Deflection of the member's point at x, world y (the bending plane), m.
inline double point_height(const MultibodyProvider& provider, const State& state, double x) {
    const LumpedBeam& beam = provider.model().members.front();
    return world_point(provider, state, beam_point(beam, x)).y();
}

// Unit-load chain value (lumped_beam.hpp, unit-load method) of the pinned-pinned chain: the deflection at b, measured
// by the lever rule's interpolation around b, under P = 1 distributed by the lever rule onto the load nodes around a;
// sum over the springs of m_a m_b l (exact for the statically determinate chain), divided by E I. Symmetric in a, b.
inline double pinned_chain_transfer(int segments, double a, double b, double length) {
    const double l = length / segments;
    std::vector<double> nodes{0.0};
    for (int j = 0; j < segments; ++j) {
        nodes.push_back((j + 0.5) * l);
    }
    nodes.push_back(length);
    const auto moment = [&](double s, double x) {
        std::size_t k = 0;
        while (k + 2 < nodes.size() && nodes[k + 1] <= s) {
            ++k;
        }
        const double w = (nodes[k + 1] - s) / (nodes[k + 1] - nodes[k]);
        double m = 0.0;
        const std::array<std::array<double, 2>, 2> loads{{{nodes[k], w}, {nodes[k + 1], 1.0 - w}}};
        for (const auto& load : loads) {
            if (x <= load[0]) {
                m += load[1] * (length - load[0]) * x / length;
            } else {
                m += load[1] * load[0] * (length - x) / length;
            }
        }
        return m;
    };
    double sum = 0.0;
    for (int j = 0; j < segments; ++j) {
        sum += moment(a, (j + 0.5) * l) * moment(b, (j + 0.5) * l) * l;
    }
    return sum;
}

// The collocated value: the chain's own deflection at a under P = 1 at a, divided by E I (ruling 57).
inline double pinned_chain_compliance(int segments, double a, double length) {
    return pinned_chain_transfer(segments, a, a, length);
}

// The largest deflection over the chain's load nodes under P = 1 at a, divided by E I: the member-wide quasi-static
// reference of a single carried force (member_loads.hpp, Definition).
inline double pinned_chain_member_max(int segments, double a, double length) {
    double largest = 0.0;
    for (int j = 0; j < segments; ++j) {
        largest = std::max(largest, std::abs(pinned_chain_transfer(segments, a, (j + 0.5) * length / segments, length)));
    }
    return largest;
}

// The largest collocated value over the chain's load nodes, divided by E I: C_max of the floor over the whole span.
inline double pinned_chain_largest(int segments, double length) {
    double largest = 0.0;
    for (int j = 0; j < segments; ++j) {
        largest = std::max(largest, pinned_chain_compliance(segments, (j + 0.5) * length / segments, length));
    }
    return largest;
}

}  // namespace mechsim::engine::test
