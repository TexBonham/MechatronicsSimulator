#include "mechsim/engine/lumped_beam.hpp"

#include <cmath>
#include <cstddef>
#include <string>

#include "mechsim/engine/engine_error.hpp"

namespace mechsim::engine {

namespace {

void require_positive(double value, const std::string& what) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw InvalidModelError(0.0, "beam: " + what + " must be finite and positive");
    }
}

Body segment(const BeamSpec& spec, Eigen::Index parent, const Eigen::Vector3d& position,
             const Eigen::Quaterniond& orientation, double span) {
    Body body;
    body.parent = parent;
    body.joint.type = JointType::Revolute;
    body.joint.position = position;
    body.joint.orientation = orientation;
    body.joint.axis = Eigen::Vector3d::UnitZ();
    body.mass = spec.density * spec.area * span;
    body.com = Eigen::Vector3d(0.5 * span, 0.0, 0.0);
    const double transverse = body.mass * span * span / 12.0;
    const double axial = 2.0 * spec.density * spec.area_moment * span;
    body.inertia = Eigen::Vector3d(axial, transverse, transverse).asDiagonal();
    return body;
}

}  // namespace

LumpedBeam add_lumped_beam(MultibodyModel& model, const BeamSpec& spec, Eigen::Index parent,
                           const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    require_positive(spec.youngs_modulus, "Young's modulus");
    require_positive(spec.area_moment, "area moment");
    require_positive(spec.density, "density");
    require_positive(spec.area, "area");
    require_positive(spec.length, "length");
    require_positive(spec.roller_mass_fraction, "roller mass fraction");
    if (spec.segments < 1) {
        throw InvalidModelError(0.0, "beam: segment count must be at least 1");
    }
    if (spec.ends == BeamEnds::Clamped && spec.segments < BeamSpec::min_clamped_segments) {
        throw InvalidModelError(0.0, "beam: a clamped far end needs at least " +
                                         std::to_string(BeamSpec::min_clamped_segments) + " segments, got " +
                                         std::to_string(spec.segments) +
                                         " (midspan deflection error -4 / N^2: -6.25 % at 8, -1.56 % at 16)");
    }
    if (!std::isfinite(spec.node_damping) || spec.node_damping < 0.0) {
        throw InvalidModelError(0.0, "beam: node damping must be finite and not negative");
    }
    const auto existing = static_cast<Eigen::Index>(model.bodies.size());
    if (parent < -1 || parent >= existing) {
        throw InvalidModelError(0.0, "beam: parent must be the world (-1) or an existing body");
    }
    if (!position.allFinite()) {
        throw InvalidModelError(0.0, "beam: root position is not finite");
    }

    LumpedBeam beam;
    beam.parent = parent;
    beam.position = position;
    beam.orientation = orientation;
    beam.length = spec.length;
    const double l = spec.length / spec.segments;
    beam.node_stiffness = spec.youngs_modulus * spec.area_moment / l;
    const Eigen::Matrix3d rotation = orientation.toRotationMatrix();

    const bool pinned = spec.ends == BeamEnds::SimplySupported;
    const bool roller_end = pinned || spec.ends == BeamEnds::Clamped;
    beam.pinned_root = pinned;
    Eigen::Index previous = parent;
    double previous_span = 0.0;
    if (pinned) {
        Body pin = segment(spec, parent, position, orientation, 0.5 * l);
        pin.name = "beam pin segment";
        model.bodies.push_back(pin);
        previous = existing;
        previous_span = 0.5 * l;
        beam.bodies.push_back(previous);
        beam.starts.push_back(0.0);
    }
    for (int j = 0; j < spec.segments; ++j) {
        double span = l;
        if (j == spec.segments - 1) {
            span = 0.5 * l;
        }
        const double start = (j + 0.5) * l;
        Eigen::Vector3d joint_position = Eigen::Vector3d(previous_span, 0.0, 0.0);
        Eigen::Quaterniond joint_orientation = Eigen::Quaterniond::Identity();
        if (previous == parent) {
            joint_position = position + rotation * Eigen::Vector3d(start, 0.0, 0.0);
            joint_orientation = orientation;
        }
        Body body = segment(spec, previous, joint_position, joint_orientation, span);
        body.name = "beam segment " + std::to_string(j);
        body.joint.stiffness = Eigen::VectorXd::Constant(1, beam.node_stiffness);
        body.joint.damping = Eigen::VectorXd::Constant(1, spec.node_damping);
        model.bodies.push_back(body);
        previous = static_cast<Eigen::Index>(model.bodies.size()) - 1;
        previous_span = span;
        beam.bodies.push_back(previous);
        beam.starts.push_back(start);
    }
    if (roller_end) {
        Body roller;
        roller.name = "beam roller";
        roller.parent = parent;
        roller.joint.type = JointType::Prismatic;
        roller.joint.position = position + rotation * Eigen::Vector3d(spec.length, 0.0, 0.0);
        roller.joint.orientation = orientation;
        roller.joint.axis = Eigen::Vector3d::UnitX();
        roller.mass = spec.roller_mass_fraction * spec.density * spec.area * spec.length;
        roller.inertia = roller.mass * l * l * Eigen::Matrix3d::Identity();
        model.bodies.push_back(roller);
        beam.roller = static_cast<Eigen::Index>(model.bodies.size()) - 1;
        LoopConstraint end;
        end.type = ConstraintType::Point;
        end.body_a = previous;
        end.body_b = beam.roller;
        end.point_a = Eigen::Vector3d(previous_span, 0.0, 0.0);
        if (spec.ends == BeamEnds::Clamped) {
            end.type = ConstraintType::Hinge;
            end.axis_a = Eigen::Vector3d::UnitX();
            end.axis_b = Eigen::Vector3d::UnitX();
        }
        model.constraints.push_back(end);
        beam.roller_constraint = static_cast<Eigen::Index>(model.constraints.size()) - 1;
    }
    model.members.push_back(beam);
    return beam;
}

BeamPoint beam_point(const LumpedBeam& beam, double x) {
    if (!std::isfinite(x) || x < 0.0 || x > beam.length) {
        throw InvalidModelError(0.0, "beam: point lies outside the member");
    }
    BeamPoint out;
    if (beam.starts.empty() || x < beam.starts.front()) {
        out.body = beam.parent;
        out.point = beam.position + beam.orientation.toRotationMatrix() * Eigen::Vector3d(x, 0.0, 0.0);
        return out;
    }
    std::size_t k = 0;
    while (k + 1 < beam.starts.size() && beam.starts[k + 1] <= x) {
        ++k;
    }
    out.body = beam.bodies[k];
    out.point = Eigen::Vector3d(x - beam.starts[k], 0.0, 0.0);
    return out;
}

}  // namespace mechsim::engine
