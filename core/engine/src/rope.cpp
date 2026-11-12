#include "mechsim/engine/rope.hpp"

#include <cmath>
#include <cstddef>
#include <string>

#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/multibody_model.hpp"

namespace mechsim::engine {

namespace {

void require(bool ok, Eigen::Index index, const std::string& what) {
    if (!ok) {
        throw InvalidModelError(0.0, "rope " + std::to_string(index) + ": " + what);
    }
}

bool positive(double value) { return std::isfinite(value) && value > 0.0; }

bool non_negative(double value) { return std::isfinite(value) && value >= 0.0; }

// A point mass at a body-frame point joins the body: mass, center of mass and inertia about the new center.
void add_point_mass(Body& body, const Eigen::Vector3d& point, double mass) {
    const double total = body.mass + mass;
    const Eigen::Vector3d com = (body.mass * body.com + mass * point) / total;
    const auto shift = [](double m, const Eigen::Vector3d& r) {
        return Eigen::Matrix3d(m * (r.squaredNorm() * Eigen::Matrix3d::Identity() - r * r.transpose()));
    };
    Eigen::Matrix3d inertia = body.inertia + shift(body.mass, body.com - com) + shift(mass, point - com);
    inertia = 0.5 * (inertia + inertia.transpose()).eval();
    body.mass = total;
    body.com = com;
    body.inertia = inertia;
}

}  // namespace

void validate_rope_parameters(const Rope& rope, Eigen::Index index) {
    require(rope.segments >= 1, index, "segment count must be at least 1");
    require(positive(rope.axial_rigidity), index, "axial rigidity EA must be finite and positive");
    require(non_negative(rope.damping), index, "damping must be finite and not negative");
    require(positive(rope.mass_per_length), index, "mass per length must be finite and positive");
    require(positive(rope.rest_length), index, "rest length must be finite and positive");
    require(!std::isnan(rope.breaking_tension) && rope.breaking_tension > 0.0, index,
            "breaking tension must be positive (infinity: never snaps)");
    require(non_negative(rope.attachment_damping), index, "attachment damping must be finite and not negative");
    require(non_negative(rope.attachment_friction), index, "attachment friction must be finite and not negative");
    require(rope.from_point.allFinite() && rope.to_point.allFinite(), index, "attach point is not finite");
}

Eigen::Index add_rope(MultibodyModel& model, Rope rope, const Eigen::Vector3d& from_world,
                      const Eigen::Vector3d& to_world) {
    const auto index = static_cast<Eigen::Index>(model.ropes.size());
    validate_rope_parameters(rope, index);
    const auto existing = static_cast<Eigen::Index>(model.bodies.size());
    require(rope.from_body >= -1 && rope.from_body < existing, index, "from_body must be the world or a body");
    require(rope.to_body >= -1 && rope.to_body < existing, index, "to_body must be the world or a body");
    require(rope.from_body < 0 || rope.from_body != rope.to_body, index, "joins a body to itself");
    require(from_world.allFinite() && to_world.allFinite(), index, "attach point is not finite");

    const double l = rope.rest_length / rope.segments;
    const double node_mass = rope.mass_per_length * l;
    rope.nodes.clear();
    for (int i = 1; i < rope.segments; ++i) {
        Body node;
        node.name = "rope " + std::to_string(index) + " node " + std::to_string(i);
        node.parent = -1;
        node.joint.type = JointType::Translation;
        node.joint.position = from_world + (static_cast<double>(i) / rope.segments) * (to_world - from_world);
        node.mass = node_mass;
        model.bodies.push_back(node);
        rope.nodes.push_back(static_cast<Eigen::Index>(model.bodies.size()) - 1);
    }
    if (rope.from_body >= 0) {
        add_point_mass(model.bodies[static_cast<std::size_t>(rope.from_body)], rope.from_point, 0.5 * node_mass);
    }
    if (rope.to_body >= 0) {
        add_point_mass(model.bodies[static_cast<std::size_t>(rope.to_body)], rope.to_point, 0.5 * node_mass);
    }
    model.ropes.push_back(rope);
    return index;
}

RopeSegment rope_segment(const Eigen::Vector3d& a, const Eigen::Vector3d& a_velocity, const Eigen::Vector3d& b,
                         const Eigen::Vector3d& b_velocity, double rest, double stiffness, double damping) {
    RopeSegment out;
    const Eigen::Vector3d d = b - a;
    out.length = d.norm();
    if (out.length > 0.0) {
        out.direction = d / out.length;
        out.rate = out.direction.dot(b_velocity - a_velocity);
    }
    if (out.length > rest) {
        const double tension = stiffness * (out.length - rest) + damping * out.rate;
        if (tension > 0.0) {
            out.tension = tension;
            out.taut = true;
        }
    }
    return out;
}

}  // namespace mechsim::engine
