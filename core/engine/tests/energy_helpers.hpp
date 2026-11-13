#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <vector>

#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/rope.hpp"
#include "mechsim/engine/state.hpp"
#include "mechsim/engine/transmission.hpp"

namespace mechsim::engine::test {

// Energy oracle helpers (brief M3-E6, shared since M3-E7): see test_energy.cpp for the derivation.
inline Eigen::Vector3d body_point(const BodyOutput& body, const Eigen::Vector3d& local) {
    return body.position + (body.orientation * local);
}

inline Eigen::Vector3d point_velocity(const BodyOutput& body, const Eigen::Vector3d& local) {
    const Eigen::Vector3d omega = body.spatial_velocity.head<3>();
    return body.origin_velocity + omega.cross(body.orientation * local);
}

// Total mechanical energy: kinetic and gravitational of every body (centre of mass), the transmissions' springs
// k d^2 / 2 (d the deflection beyond the backlash, in contact), the rope segments' tension-only springs
// (EA / l)(d - l)^2 / 2 when taut (l = rest / N), a one-DOF model joint spring K (q - rest)^2 / 2, and the carried
// masses' deflection motion (1/2) v^T B v (member_loads.hpp, Carried mass; zero without carried joints).
inline double mechanical_energy(const MultibodyProvider& provider, const State& state) {
    const MultibodyModel& model = provider.model();
    const std::vector<BodyOutput> bodies = provider.body_states(state.q, state.v);
    double energy = 0.5 * state.v.dot(provider.carried_mass_matrix(state.q) * state.v);
    for (std::size_t b = 0; b < bodies.size(); ++b) {
        const Body& body = model.bodies[b];
        const BodyOutput& out = bodies[b];
        const Eigen::Vector3d omega = out.spatial_velocity.head<3>();
        const Eigen::Vector3d com = body_point(out, body.com);
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
            points.push_back(body_point(bodies[static_cast<std::size_t>(rope.from_body)], rope.from_point));
        }
        for (const Eigen::Index node : rope.nodes) {
            points.push_back(bodies[static_cast<std::size_t>(node)].position);
        }
        points.push_back(body_point(bodies[static_cast<std::size_t>(rope.to_body)], rope.to_point));
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

}  // namespace mechsim::engine::test
