#include "mechsim/engine/contact.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>

#include "mechsim/engine/engine_error.hpp"

namespace mechsim::engine {

namespace {

std::string geometry_label(Eigen::Index i) { return "contact geometry " + std::to_string(i) + ": "; }

void validate_parameters(const ContactParameters& p, const std::string& label) {
    if (!std::isfinite(p.stiffness) || !(p.stiffness > 0.0)) {
        throw InvalidModelError(0.0, label + "contact stiffness must be finite and > 0");
    }
    if (!std::isfinite(p.damping) || p.damping < 0.0) {
        throw InvalidModelError(0.0, label + "contact damping must be finite and >= 0");
    }
    if (!std::isfinite(p.friction) || p.friction < 0.0) {
        throw InvalidModelError(0.0, label + "friction coefficient must be finite and >= 0");
    }
}

}  // namespace

Shape to_shape(const Geometry& geometry, Eigen::Index index) {
    const std::string label = geometry_label(index);
    Shape shape;
    try {
        shape.kind = shape_kind(geometry.kind);
    } catch (const InvalidModelError&) {
        throw InvalidModelError(0.0, label + "kind '" + geometry.kind + "' is neither a primitive nor a convex hull");
    }
    shape.radius = geometry.radius;
    shape.half_length = 0.5 * geometry.length;
    shape.half_extents = 0.5 * geometry.size;
    shape.vertices = geometry.vertices;
    validate_shape(shape, label);
    return shape;
}

void validate_contact_model(const ContactModel& model, Eigen::Index bodies) {
    if (!std::isfinite(model.slip_velocity) || !(model.slip_velocity > 0.0)) {
        throw InvalidModelError(0.0, "contact slip velocity must be finite and > 0");
    }
    validate_parameters(model.default_parameters, "default contact parameters: ");
    for (const MaterialPair& pair : model.pairs) {
        validate_parameters(pair.parameters, "material pair " + pair.material_a + "/" + pair.material_b + ": ");
    }
    for (const auto& [a, b] : model.exclusions) {
        if (a < -1 || a >= bodies || b < -1 || b >= bodies) {
            throw InvalidModelError(0.0, "contact exclusion names a missing body");
        }
    }
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(model.geometries.size()); ++i) {
        const Geometry& g = model.geometries[static_cast<std::size_t>(i)];
        const Shape shape = to_shape(g, i);
        if (g.body < -1 || g.body >= bodies) {
            throw InvalidModelError(0.0, geometry_label(i) + "body is missing");
        }
        if (!g.position.allFinite() || !g.orientation.coeffs().allFinite() ||
            std::abs(g.orientation.squaredNorm() - 1.0) > 1e-12) {
            throw InvalidModelError(0.0, geometry_label(i) + "pose must be finite with a unit quaternion");
        }
        if (shape.kind == ShapeKind::Plane && g.body != -1) {
            throw InvalidModelError(0.0, geometry_label(i) + "a plane is allowed only on the world");
        }
    }
}

const ContactParameters& contact_parameters(const ContactModel& model, const std::string& material_a,
                                            const std::string& material_b) {
    for (const MaterialPair& pair : model.pairs) {
        const bool same = pair.material_a == material_a && pair.material_b == material_b;
        const bool swapped = pair.material_a == material_b && pair.material_b == material_a;
        if (same || swapped) {
            return pair.parameters;
        }
    }
    return model.default_parameters;
}

bool excluded(const ContactModel& model, Eigen::Index body_a, Eigen::Index body_b) {
    return std::ranges::any_of(model.exclusions, [body_a, body_b](const auto& pair) {
        return (pair.first == body_a && pair.second == body_b) || (pair.first == body_b && pair.second == body_a);
    });
}

ContactForce contact_force(const ContactParameters& parameters, double slip_velocity, const ContactPoint& contact,
                           const Eigen::Vector3d& relative_velocity) {
    ContactForce out;
    const Eigen::Vector3d& n = contact.normal;
    const double normal_rate = n.dot(relative_velocity);
    const double lambda = parameters.stiffness * contact.depth - parameters.damping * normal_rate;
    if (!(lambda > 0.0)) {
        return out;
    }
    out.active = true;
    out.normal_force = lambda;
    const Eigen::Matrix3d nn = n * n.transpose();
    out.stiffness = parameters.stiffness * nn;
    out.damping = parameters.damping * nn;
    out.force_on_b = lambda * n;
    const double limit = parameters.friction * lambda;
    if (!(limit > 0.0)) {
        return out;
    }
    const Eigen::Vector3d slip = relative_velocity - normal_rate * n;
    const double speed = slip.norm();
    const Eigen::Matrix3d tangent = Eigen::Matrix3d::Identity() - nn;
    if (speed < slip_velocity) {
        out.force_on_b -= (limit / slip_velocity) * slip;
        out.damping += (limit / slip_velocity) * tangent;
        return out;
    }
    const Eigen::Vector3d t = slip / speed;
    out.force_on_b -= limit * t;
    out.damping += (limit / speed) * (tangent - t * t.transpose());
    return out;
}

double angular_frequency(const WeldAttachment& attachment) { return 2.0 * std::numbers::pi * attachment.frequency; }

}  // namespace mechsim::engine
