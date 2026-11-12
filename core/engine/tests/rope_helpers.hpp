#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstddef>

#include "mechsim/engine/multibody_model.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/rope.hpp"
#include "mechsim/engine/state.hpp"

namespace mechsim::engine::test {

inline constexpr double gravity = 9.81;

// A point bob on a Translation joint, its frame origin at position (world), center of mass at the origin.
inline Eigen::Index add_bob(MultibodyModel& model, const Eigen::Vector3d& position, double mass) {
    Body bob;
    bob.name = "bob";
    bob.parent = -1;
    bob.joint.type = JointType::Translation;
    bob.joint.position = position;
    bob.mass = mass;
    bob.inertia = 1e-3 * mass * Eigen::Matrix3d::Identity();
    model.bodies.push_back(bob);
    return static_cast<Eigen::Index>(model.bodies.size()) - 1;
}

// Rope pendulum (ADR 0016, ACC-037 form): pivot at the world origin, bob mass M on a rope of rest length L0 and mass
// ratio m_r / M, released at rest at theta0 in the x-z plane. The rope is placed straight at the loaded length
// L = L0 + g L0 (M + m_r / 2) / EA. Axial damping 2 sqrt(EA M L0) makes the bob-on-rope axial mode critical
// (series stiffness EA / L0, series damping damping / L0), so start-up axial transients die within a few ms.
struct RopePendulum {
    MultibodyModel model;
    Eigen::Index bob = -1;
    double bob_mass = 0.0;   // M, kg
    double rope_mass = 0.0;  // m_r, kg
    double length = 0.0;     // L loaded, m
    int segments = 0;
};

inline RopePendulum rope_pendulum(int segments, double bob_mass, double mass_ratio, double theta0,
                                  double attachment_damping) {
    const double rest = 1.0;
    const double axial = 1e6;
    RopePendulum out;
    out.model.gravity = Eigen::Vector3d(0.0, 0.0, -gravity);
    out.bob_mass = bob_mass;
    out.rope_mass = mass_ratio * bob_mass;
    out.segments = segments;
    out.length = rest + gravity * rest * (bob_mass + 0.5 * out.rope_mass) / axial;
    const Eigen::Vector3d at = out.length * Eigen::Vector3d(std::sin(theta0), 0.0, -std::cos(theta0));
    out.bob = add_bob(out.model, at, bob_mass);
    Rope rope;
    rope.from_body = -1;
    rope.to_body = out.bob;
    rope.segments = segments;
    rope.axial_rigidity = axial;
    rope.damping = 2.0 * std::sqrt(axial * bob_mass * rest);
    rope.mass_per_length = out.rope_mass / rest;
    rope.rest_length = rest;
    rope.attachment_damping = attachment_damping;
    static_cast<void>(add_rope(out.model, rope, Eigen::Vector3d::Zero(), at));
    return out;
}

// ADR 0016 closed form: straight-mode Rayleigh quotient of the chain as built (interior node masses m_r / N at
// i L / N, end half-masses at 0 and L): I_N = M L^2 + m_r L^2 (1/3 + 1 / (6 N^2)), S = M L + m_r L / 2,
// omega^2 = g S / I_N. Exact to second order in m_r / M (the Rayleigh quotient is stationary in the mode shape).
inline double pendulum_inertia(const RopePendulum& p) {
    const double l2 = p.length * p.length;
    const double n2 = static_cast<double>(p.segments) * p.segments;
    return p.bob_mass * l2 + p.rope_mass * l2 * (1.0 / 3.0 + 1.0 / (6.0 * n2));
}

inline double pendulum_moment(const RopePendulum& p) {
    return p.bob_mass * p.length + 0.5 * p.rope_mass * p.length;
}

inline double pendulum_omega(const RopePendulum& p) {
    return std::sqrt(gravity * pendulum_moment(p) / pendulum_inertia(p));
}

inline double bob_x(const MultibodyProvider& provider, const State& state, Eigen::Index bob) {
    return state.q[provider.position_offset(bob)] + provider.model().bodies[static_cast<std::size_t>(bob)].joint.position.x();
}

}  // namespace mechsim::engine::test
