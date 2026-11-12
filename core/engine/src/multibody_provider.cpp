#include "mechsim/engine/multibody_provider.hpp"

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "mechsim/engine/engine_error.hpp"

namespace mechsim::engine {

using spatial::Matrix6d;
using spatial::Matrix6Xd;
using spatial::Transform;
using spatial::Vector6d;

struct MultibodyProvider::Kinematics {
    std::vector<Transform> up;       // body_X_parent
    std::vector<Transform> world;    // body_X_world
    std::vector<Matrix6Xd> subspace;  // S, body coordinates
    std::vector<Vector6d> velocity;  // body coordinates
    std::vector<Vector6d> bias;      // c_J + v x (S qdot), body coordinates
};

struct MultibodyProvider::ConstraintRows {
    Eigen::MatrixXd jacobian;  // rows x nv, residual rate per generalized velocity
    ConstraintError error;
    Eigen::Vector3d point_a = Eigen::Vector3d::Zero();  // world, m
    Eigen::Vector3d point_b = Eigen::Vector3d::Zero();  // world, m
    std::vector<Eigen::Vector3d> torque_axes;           // world; hinge row k acts as torque along axis k on body_a
};

// ===== Helpers ===========================================================

namespace {

template <typename T>
const T& at(const std::vector<T>& items, Eigen::Index i) {
    return items[static_cast<std::size_t>(i)];
}

template <typename T>
T& at(std::vector<T>& items, Eigen::Index i) {
    return items[static_cast<std::size_t>(i)];
}

Eigen::Index count(std::size_t n) { return static_cast<Eigen::Index>(n); }

Eigen::Index joint_positions(JointType type) {
    switch (type) {
        case JointType::Revolute:
        case JointType::Prismatic:
            return 1;
        case JointType::Fixed:
            return 0;
        case JointType::Planar:
        case JointType::Translation:
            return 3;
        case JointType::Spherical:
            return 4;
        case JointType::Free:
            return 7;
    }
    return 0;
}

Eigen::Index joint_velocities(JointType type) {
    switch (type) {
        case JointType::Revolute:
        case JointType::Prismatic:
            return 1;
        case JointType::Fixed:
            return 0;
        case JointType::Planar:
        case JointType::Spherical:
        case JointType::Translation:
            return 3;
        case JointType::Free:
            return 6;
    }
    return 0;
}

struct PlaneBasis {
    Eigen::Vector3d u;
    Eigen::Vector3d w;
};

// Deterministic orthonormal u, w with u x w = n for a unit n.
PlaneBasis plane_basis(const Eigen::Vector3d& n) {
    Eigen::Index smallest = 0;
    n.cwiseAbs().minCoeff(&smallest);
    PlaneBasis basis;
    basis.u = n.cross(Eigen::Vector3d::Unit(smallest)).normalized();
    basis.w = n.cross(basis.u);
    return basis;
}

Eigen::Quaterniond quaternion_at(const Eigen::VectorXd& q, Eigen::Index offset) {
    return {q[offset], q[offset + 1], q[offset + 2], q[offset + 3]};
}

void store_quaternion(Eigen::VectorXd& q, Eigen::Index offset, const Eigen::Quaterniond& r) {
    q[offset] = r.w();
    q[offset + 1] = r.x();
    q[offset + 2] = r.y();
    q[offset + 3] = r.z();
}

// Right-multiplies by the rotation of the body-frame rotation vector, then renormalizes (ENG-010, ENG-011).
Eigen::Quaterniond rotate(const Eigen::Quaterniond& r, const Eigen::Vector3d& rotation_vector) {
    const double angle = rotation_vector.norm();
    Eigen::Quaterniond delta = Eigen::Quaterniond::Identity();
    if (angle > 0.0) {
        delta = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
    }
    return (r * delta).normalized();
}

bool is_unit(const Eigen::Vector3d& a, double tolerance) {
    return a.allFinite() && std::abs(a.squaredNorm() - 1.0) <= tolerance;
}

bool is_unit(const Eigen::Quaterniond& r, double tolerance) {
    return r.coeffs().allFinite() && std::abs(r.squaredNorm() - 1.0) <= tolerance;
}

std::string body_label(Eigen::Index i) { return "body " + std::to_string(i) + ": "; }

void validate_config(const MultibodyConfig& config) {
    if (config.iteration_cap < 1) {
        throw InvalidConfigError(0.0, "constraint iteration cap must be at least 1");
    }
    if (config.projection_iterations < 0) {
        throw InvalidConfigError(0.0, "projection iterations must be non-negative");
    }
    const std::vector<std::pair<double, std::string>> positive = {
        {config.velocity_tolerance, "constraint velocity tolerance"},
        {config.drift_threshold_m, "translational drift threshold"},
        {config.drift_threshold_rad, "rotational drift threshold"},
        {config.unit_tolerance, "unit tolerance"},
    };
    for (const auto& [value, name] : positive) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw InvalidConfigError(0.0, name + " must be finite and positive");
        }
    }
}

void validate_joint_vector(const Eigen::VectorXd& values, Eigen::Index dofs, bool non_negative,
                           const std::string& what) {
    if (values.size() == 0) {
        return;
    }
    if (values.size() != dofs) {
        throw InvalidModelError(0.0, what + " must be empty or have one entry per joint velocity");
    }
    if (!values.allFinite()) {
        throw InvalidModelError(0.0, what + " is not finite");
    }
    if (non_negative && values.minCoeff() < 0.0) {
        throw InvalidModelError(0.0, what + " must not be negative");
    }
}

void validate_body(const Body& body, Eigen::Index i, double tolerance) {
    const std::string label = body_label(i);
    if (body.parent < -1 || body.parent >= i) {
        throw InvalidModelError(0.0, label + "parent must be the world (-1) or an earlier body");
    }
    if (!std::isfinite(body.mass) || body.mass <= 0.0) {
        throw InvalidModelError(0.0, label + "mass must be finite and positive");
    }
    if (!body.com.allFinite()) {
        throw InvalidModelError(0.0, label + "center of mass is not finite");
    }
    if (!body.inertia.allFinite()) {
        throw InvalidModelError(0.0, label + "inertia is not finite");
    }
    if (body.inertia != body.inertia.transpose()) {
        throw InvalidModelError(0.0, label + "inertia is not symmetric");
    }
    if (!body.joint.position.allFinite()) {
        throw InvalidModelError(0.0, label + "joint position is not finite");
    }
    if (!is_unit(body.joint.orientation, tolerance)) {
        throw InvalidModelError(0.0, label + "joint orientation is not a unit quaternion");
    }
    const JointType type = body.joint.type;
    const bool uses_axis = type == JointType::Revolute || type == JointType::Prismatic || type == JointType::Planar;
    if (uses_axis && !is_unit(body.joint.axis, tolerance)) {
        throw InvalidModelError(0.0, label + "joint axis is not a unit vector");
    }
    const Eigen::Index dofs = joint_velocities(type);
    validate_joint_vector(body.joint.stiffness, dofs, true, label + "joint stiffness");
    validate_joint_vector(body.joint.damping, dofs, true, label + "joint damping");
    validate_joint_vector(body.joint.rest, dofs, false, label + "joint rest position");
    if (!uses_axis && (body.joint.stiffness.size() > 0 || body.joint.rest.size() > 0)) {
        throw InvalidModelError(0.0, label + "joint stiffness and rest position need a Revolute, Prismatic or "
                                             "Planar joint");
    }
}

void validate_constraint(const LoopConstraint& c, Eigen::Index i, Eigen::Index bodies, double tolerance) {
    const std::string label = "constraint " + std::to_string(i) + ": ";
    if (c.body_a < 0 || c.body_a >= bodies || c.body_b < 0 || c.body_b >= bodies) {
        throw InvalidModelError(0.0, label + "references a missing body");
    }
    if (c.body_a == c.body_b) {
        throw InvalidModelError(0.0, label + "joins a body to itself");
    }
    if (!c.point_a.allFinite() || !c.point_b.allFinite()) {
        throw InvalidModelError(0.0, label + "point is not finite");
    }
    if (c.type == ConstraintType::Hinge && (!is_unit(c.axis_a, tolerance) || !is_unit(c.axis_b, tolerance))) {
        throw InvalidModelError(0.0, label + "hinge axis is not a unit vector");
    }
}

void validate_transmission(const Transmission& t, Eigen::Index i, const std::vector<Body>& bodies) {
    validate_transmission_parameters(t, i);
    const std::string label = "transmission " + std::to_string(i) + ": ";
    const Eigen::Index n = count(bodies.size());
    if (t.input_body < 0 || t.input_body >= n || t.output_body < 0 || t.output_body >= n) {
        throw InvalidModelError(0.0, label + "references a missing body");
    }
    if (t.input_body == t.output_body) {
        throw InvalidModelError(0.0, label + "couples a body to itself");
    }
    if (at(bodies, t.input_body).joint.type != JointType::Revolute) {
        throw InvalidModelError(0.0, label + "input shaft needs a Revolute joint");
    }
    JointType output = JointType::Prismatic;
    if (t.type == TransmissionType::GearPair) {
        output = JointType::Revolute;
    }
    if (at(bodies, t.output_body).joint.type != output) {
        throw InvalidModelError(0.0, label + "output joint type does not match the transmission type");
    }
}

// Wrench given world-aligned at a world point, expressed as a force vector in body coordinates.
Vector6d to_body(const Transform& body_from_world, const Eigen::Vector3d& point, const Eigen::Vector3d& torque,
                 const Eigen::Vector3d& force) {
    Vector6d out;
    const Eigen::Vector3d arm = point - body_from_world.translation;
    out.head<3>() = body_from_world.rotation * (torque + arm.cross(force));
    out.tail<3>() = body_from_world.rotation * force;
    return out;
}

Eigen::Vector3d world_point(const Transform& body_from_world, const Eigen::Vector3d& local) {
    return body_from_world.translation + body_from_world.rotation.transpose() * local;
}

}  // namespace

// ===== Construction ======================================================

MultibodyProvider::MultibodyProvider(MultibodyModel model, MultibodyConfig config)
    : model_(std::move(model)), config_(config) {
    validate_config(config_);
    if (!model_.gravity.allFinite()) {
        throw InvalidModelError(0.0, "gravity is not finite");
    }
    const Eigen::Index n = count(model_.bodies.size());
    for (Eigen::Index i = 0; i < n; ++i) {
        const Body& body = at(model_.bodies, i);
        validate_body(body, i, config_.unit_tolerance);
        q_offset_.push_back(nq_);
        v_offset_.push_back(nv_);
        nq_ += joint_positions(body.joint.type);
        nv_ += joint_velocities(body.joint.type);
        Transform placement;
        placement.rotation = body.joint.orientation.toRotationMatrix().transpose();
        placement.translation = body.joint.position;
        placement_.push_back(placement);
        inertia_.push_back(spatial::inertia(body.mass, body.com, body.inertia));
    }
    stiffness_ = Eigen::VectorXd::Zero(nv_);
    damping_ = Eigen::VectorXd::Zero(nv_);
    rest_ = Eigen::VectorXd::Zero(nv_);
    for (Eigen::Index i = 0; i < n; ++i) {
        const Joint& joint = at(model_.bodies, i).joint;
        const Eigen::Index ov = at(v_offset_, i);
        if (joint.stiffness.size() > 0) {
            stiffness_.segment(ov, joint.stiffness.size()) = joint.stiffness;
        }
        if (joint.damping.size() > 0) {
            damping_.segment(ov, joint.damping.size()) = joint.damping;
        }
        if (joint.rest.size() > 0) {
            rest_.segment(ov, joint.rest.size()) = joint.rest;
        }
    }
    for (Eigen::Index i = 0; i < count(model_.constraints.size()); ++i) {
        validate_constraint(at(model_.constraints, i), i, n, config_.unit_tolerance);
    }
    for (Eigen::Index i = 0; i < count(model_.transmissions.size()); ++i) {
        const Transmission& t = at(model_.transmissions, i);
        validate_transmission(t, i, model_.bodies);
        slip_.push_back(t.slip);
        slip_offset_.push_back(0.0);
    }
    setup_members_and_ropes();
    clear_inputs();
}

Eigen::Index MultibodyProvider::position_size() const { return nq_; }

Eigen::Index MultibodyProvider::velocity_size() const { return nv_; }

const MultibodyModel& MultibodyProvider::model() const noexcept { return model_; }

const MultibodyConfig& MultibodyProvider::config() const noexcept { return config_; }

Eigen::Index MultibodyProvider::position_offset(Eigen::Index body) const { return at(q_offset_, body); }

Eigen::Index MultibodyProvider::velocity_offset(Eigen::Index body) const { return at(v_offset_, body); }

Eigen::VectorXd MultibodyProvider::neutral_positions() const {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(nq_);
    for (Eigen::Index i = 0; i < count(model_.bodies.size()); ++i) {
        const JointType type = at(model_.bodies, i).joint.type;
        if (type == JointType::Spherical) {
            q[at(q_offset_, i)] = 1.0;
        }
        if (type == JointType::Free) {
            q[at(q_offset_, i) + 3] = 1.0;
        }
    }
    return q;
}

// ===== Kinematics ========================================================

MultibodyProvider::Kinematics MultibodyProvider::kinematics(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const {
    Kinematics kin;
    for (Eigen::Index i = 0; i < count(model_.bodies.size()); ++i) {
        const Body& body = at(model_.bodies, i);
        const Joint& joint = body.joint;
        const Eigen::Index oq = at(q_offset_, i);
        const Eigen::Index ov = at(v_offset_, i);
        const Eigen::Index dofs = joint_velocities(joint.type);
        Transform xj;
        Matrix6Xd s = Matrix6Xd::Zero(6, dofs);
        Vector6d cj = Vector6d::Zero();
        switch (joint.type) {
            case JointType::Revolute: {
                xj.rotation = Eigen::AngleAxisd(q[oq], joint.axis).toRotationMatrix().transpose();
                s.col(0).head<3>() = joint.axis;
                break;
            }
            case JointType::Prismatic: {
                xj.translation = q[oq] * joint.axis;
                s.col(0).tail<3>() = joint.axis;
                break;
            }
            case JointType::Fixed: {
                break;
            }
            case JointType::Planar: {
                const PlaneBasis basis = plane_basis(joint.axis);
                xj.rotation = Eigen::AngleAxisd(q[oq + 2], joint.axis).toRotationMatrix().transpose();
                xj.translation = q[oq] * basis.u + q[oq + 1] * basis.w;
                s.col(0).tail<3>() = xj.rotation * basis.u;
                s.col(1).tail<3>() = xj.rotation * basis.w;
                s.col(2).head<3>() = joint.axis;
                const Eigen::Vector3d slide = xj.rotation * (v[ov] * basis.u + v[ov + 1] * basis.w);
                cj.tail<3>() = -(v[ov + 2] * joint.axis).cross(slide);
                break;
            }
            case JointType::Spherical: {
                xj.rotation = quaternion_at(q, oq).toRotationMatrix().transpose();
                s.topRows<3>().setIdentity();
                break;
            }
            case JointType::Free: {
                xj.rotation = quaternion_at(q, oq + 3).toRotationMatrix().transpose();
                xj.translation = q.segment<3>(oq);
                s.setIdentity();
                break;
            }
            case JointType::Translation: {
                xj.translation = q.segment<3>(oq);
                s.bottomRows<3>().setIdentity();
                break;
            }
        }
        const Transform up = xj.then_from(at(placement_, i));
        const Vector6d vj = s * v.segment(ov, dofs);
        Vector6d velocity = vj;
        Transform world = up;
        if (body.parent >= 0) {
            velocity += up.motion(at(kin.velocity, body.parent));
            world = up.then_from(at(kin.world, body.parent));
        }
        kin.up.push_back(up);
        kin.world.push_back(world);
        kin.subspace.push_back(s);
        kin.velocity.push_back(velocity);
        kin.bias.emplace_back(cj + spatial::cross_motion(velocity, vj));
    }
    return kin;
}

// ===== Dynamics ==========================================================

// Composite rigid body algorithm; the lower triangle is computed and mirrored so H is exactly symmetric.
Eigen::MatrixXd MultibodyProvider::mass_matrix(const Kinematics& kin) const {
    const Eigen::Index n = count(model_.bodies.size());
    std::vector<Matrix6d> composite = inertia_;
    for (Eigen::Index i = n - 1; i >= 0; --i) {
        const Eigen::Index parent = at(model_.bodies, i).parent;
        if (parent >= 0) {
            const Matrix6d x = at(kin.up, i).motion_matrix();
            at(composite, parent) += x.transpose() * at(composite, i) * x;
        }
    }
    Eigen::MatrixXd lower = Eigen::MatrixXd::Zero(nv_, nv_);
    for (Eigen::Index i = 0; i < n; ++i) {
        const Matrix6Xd& si = at(kin.subspace, i);
        const Eigen::Index dofs = si.cols();
        if (dofs == 0) {
            continue;
        }
        Matrix6Xd f = at(composite, i) * si;
        lower.block(at(v_offset_, i), at(v_offset_, i), dofs, dofs) = si.transpose() * f;
        Eigen::Index j = i;
        while (at(model_.bodies, j).parent >= 0) {
            f = at(kin.up, j).motion_matrix().transpose() * f;
            j = at(model_.bodies, j).parent;
            const Matrix6Xd& sj = at(kin.subspace, j);
            if (sj.cols() > 0) {
                lower.block(at(v_offset_, i), at(v_offset_, j), dofs, sj.cols()) = f.transpose() * sj;
            }
        }
    }
    Eigen::MatrixXd full = lower.selfadjointView<Eigen::Lower>();
    return full;
}

// Recursive Newton-Euler: generalized forces for qdd with the given external body forces (body coordinates).
Eigen::VectorXd MultibodyProvider::inverse_dynamics(const Kinematics& kin, const Eigen::VectorXd& qdd,
                                                    const Vector6d& base_acceleration,
                                                    const std::vector<Vector6d>& external, bool velocity_terms,
                                                    std::vector<Vector6d>& body_forces) const {
    const Eigen::Index n = count(model_.bodies.size());
    std::vector<Vector6d> acceleration(static_cast<std::size_t>(n), Vector6d::Zero());
    body_forces.assign(static_cast<std::size_t>(n), Vector6d::Zero());
    for (Eigen::Index i = 0; i < n; ++i) {
        const Eigen::Index parent = at(model_.bodies, i).parent;
        const Matrix6Xd& s = at(kin.subspace, i);
        Vector6d a = s * qdd.segment(at(v_offset_, i), s.cols());
        if (parent >= 0) {
            a += at(kin.up, i).motion(at(acceleration, parent));
        } else {
            a += at(kin.up, i).motion(base_acceleration);
        }
        if (velocity_terms) {
            a += at(kin.bias, i);
        }
        const Matrix6d& inertia = at(inertia_, i);
        Vector6d f = inertia * a - at(external, i);
        if (velocity_terms) {
            f += spatial::cross_force(at(kin.velocity, i), inertia * at(kin.velocity, i));
        }
        at(acceleration, i) = a;
        at(body_forces, i) = f;
    }
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(nv_);
    for (Eigen::Index i = n - 1; i >= 0; --i) {
        const Matrix6Xd& s = at(kin.subspace, i);
        tau.segment(at(v_offset_, i), s.cols()) = s.transpose() * at(body_forces, i);
        const Eigen::Index parent = at(model_.bodies, i).parent;
        if (parent >= 0) {
            at(body_forces, parent) += at(kin.up, i).inverse_force(at(body_forces, i));
        }
    }
    return tau;
}

std::vector<Vector6d> MultibodyProvider::applied_body_wrenches(const Kinematics& kin) const {
    std::vector<Vector6d> out;
    for (Eigen::Index i = 0; i < count(model_.bodies.size()); ++i) {
        const Transform& x = at(kin.world, i);
        const Vector6d& w = at(wrench_, i);
        out.push_back(to_body(x, world_point(x, at(model_.bodies, i).com), w.head<3>(), w.tail<3>()));
    }
    add_rope_wrenches(kin, out);
    return out;
}

void MultibodyProvider::evaluate(double /*t*/, const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                 DynamicsTerms& terms) const {
    const Kinematics kin = kinematics(q, v);
    terms.mass = mass_matrix(kin);
    Vector6d base = Vector6d::Zero();
    base.tail<3>() = -model_.gravity;
    const std::vector<Vector6d> none(model_.bodies.size(), Vector6d::Zero());
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(nv_);
    std::vector<Vector6d> forces;
    terms.bias = inverse_dynamics(kin, zero, base, none, true, forces);
    const Eigen::VectorXd external = inverse_dynamics(kin, zero, Vector6d::Zero(), applied_body_wrenches(kin), false,
                                                      forces);
    terms.force = tau_ - external + joint_spring_forces(q, v);
    terms.stiffness = stiffness_.asDiagonal();
    terms.damping = damping_.asDiagonal();
    add_transmission_terms(q, v, terms);
    add_rope_terms(kin, terms);
}

// ===== Transmissions =====================================================

// Transmissions (ENG-043 to ENG-048): f gets the shaft torque input, F on the output and -g F on the shaft; in
// contact K += k a b^T and D += c a b^T, a = [g, -1], b = [n_eff, -1] over (shaft, output); derivation in
// transmission.hpp.
TransmissionCoupling MultibodyProvider::coupling(Eigen::Index transmission, const Eigen::VectorXd& q,
                                                 const Eigen::VectorXd& v) const {
    const Transmission& t = at(model_.transmissions, transmission);
    const Eigen::Index in_q = at(q_offset_, t.input_body);
    const Eigen::Index in_v = at(v_offset_, t.input_body);
    const Eigen::Index out_q = at(q_offset_, t.output_body);
    const Eigen::Index out_v = at(v_offset_, t.output_body);
    double drive = tau_[in_v];
    for (Eigen::Index j = 0; j < count(model_.transmissions.size()); ++j) {
        if (at(model_.transmissions, j).input_body == t.input_body) {
            drive += at(shaft_torque_, j);
        }
    }
    return couple(t, at(slip_, transmission), at(slip_offset_, transmission), q[in_q], v[in_v], q[out_q], v[out_v],
                  drive);
}

void MultibodyProvider::add_transmission_terms(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                               DynamicsTerms& terms) const {
    for (Eigen::Index i = 0; i < count(model_.transmissions.size()); ++i) {
        const Transmission& t = at(model_.transmissions, i);
        const Eigen::Index in = at(v_offset_, t.input_body);
        const Eigen::Index out = at(v_offset_, t.output_body);
        const TransmissionCoupling c = coupling(i, q, v);
        terms.force[in] += at(shaft_torque_, i) + c.output.shaft_load_torque;
        terms.force[out] += c.output.force;
        if (!c.output.in_contact) {
            continue;
        }
        const Eigen::Vector2d a(c.gain, -1.0);
        const Eigen::Vector2d b(c.n_eff, -1.0);
        const Eigen::Matrix2d ab = a * b.transpose();
        const std::array<Eigen::Index, 2> dof = {in, out};
        for (std::size_t r = 0; r < 2; ++r) {
            for (std::size_t col = 0; col < 2; ++col) {
                const double entry = ab(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(col));
                terms.stiffness(dof.at(r), dof.at(col)) += t.stiffness * entry;
                terms.damping(dof.at(r), dof.at(col)) += t.damping * entry;
            }
        }
    }
}

// Joint springs and dampers (ENG-051): -K (q - rest) - D v; stiffness is zero wherever q and v are not aligned.
Eigen::VectorXd MultibodyProvider::joint_spring_forces(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const {
    Eigen::VectorXd out = -damping_.cwiseProduct(v);
    for (Eigen::Index i = 0; i < count(model_.bodies.size()); ++i) {
        const Eigen::Index dofs = at(model_.bodies, i).joint.stiffness.size();
        if (dofs == 0) {
            continue;
        }
        const Eigen::Index oq = at(q_offset_, i);
        const Eigen::Index ov = at(v_offset_, i);
        out.segment(ov, dofs) -=
            stiffness_.segment(ov, dofs).cwiseProduct(q.segment(oq, dofs) - rest_.segment(ov, dofs));
    }
    return out;
}

Eigen::VectorXd MultibodyProvider::integrate_positions(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                       double h) const {
    Eigen::VectorXd out = q;
    for (Eigen::Index i = 0; i < count(model_.bodies.size()); ++i) {
        const Eigen::Index oq = at(q_offset_, i);
        const Eigen::Index ov = at(v_offset_, i);
        switch (at(model_.bodies, i).joint.type) {
            case JointType::Revolute:
            case JointType::Prismatic: {
                out[oq] += h * v[ov];
                break;
            }
            case JointType::Fixed: {
                break;
            }
            case JointType::Planar:
            case JointType::Translation: {
                out.segment<3>(oq) += h * v.segment<3>(ov);
                break;
            }
            case JointType::Spherical: {
                store_quaternion(out, oq, rotate(quaternion_at(q, oq), h * v.segment<3>(ov)));
                break;
            }
            case JointType::Free: {
                const Eigen::Quaterniond r = quaternion_at(q, oq + 3);
                out.segment<3>(oq) += h * (r.toRotationMatrix() * v.segment<3>(ov + 3));
                store_quaternion(out, oq + 3, rotate(r, h * v.segment<3>(ov)));
                break;
            }
        }
    }
    return out;
}

// ===== Constraints =======================================================

Matrix6Xd MultibodyProvider::world_jacobian(const Kinematics& kin, Eigen::Index body) const {
    Matrix6Xd jacobian = Matrix6Xd::Zero(6, nv_);
    Eigen::Index j = body;
    while (j >= 0) {
        const Matrix6Xd& s = at(kin.subspace, j);
        for (Eigen::Index k = 0; k < s.cols(); ++k) {
            jacobian.col(at(v_offset_, j) + k) = at(kin.world, j).inverse_motion(s.col(k));
        }
        j = at(model_.bodies, j).parent;
    }
    return jacobian;
}

MultibodyProvider::ConstraintRows MultibodyProvider::constraint_rows(const Kinematics& kin,
                                                                     Eigen::Index constraint) const {
    const LoopConstraint& c = at(model_.constraints, constraint);
    const Transform& xa = at(kin.world, c.body_a);
    const Transform& xb = at(kin.world, c.body_b);
    ConstraintRows rows;
    rows.point_a = world_point(xa, c.point_a);
    rows.point_b = world_point(xb, c.point_b);
    const Matrix6Xd ja = world_jacobian(kin, c.body_a);
    const Matrix6Xd jb = world_jacobian(kin, c.body_b);
    const Eigen::MatrixXd point_rate = (ja.bottomRows<3>() - spatial::skew(rows.point_a) * ja.topRows<3>()) -
                                       (jb.bottomRows<3>() - spatial::skew(rows.point_b) * jb.topRows<3>());
    const Eigen::Vector3d gap = rows.point_a - rows.point_b;
    rows.error.translational = gap.norm();
    if (c.type == ConstraintType::Point) {
        rows.jacobian = point_rate;
        rows.error.residual = gap;
        return rows;
    }
    const Eigen::Vector3d axis_a = xa.rotation.transpose() * c.axis_a;
    const Eigen::Vector3d axis_b = xb.rotation.transpose() * c.axis_b;
    const PlaneBasis basis = plane_basis(c.axis_a);
    const Eigen::Vector3d u1 = xa.rotation.transpose() * basis.u;
    const Eigen::Vector3d u2 = xa.rotation.transpose() * basis.w;
    rows.torque_axes = {u1.cross(axis_b), u2.cross(axis_b)};
    const Eigen::MatrixXd relative_rate = ja.topRows<3>() - jb.topRows<3>();
    rows.jacobian.resize(5, nv_);
    rows.jacobian.topRows<3>() = point_rate;
    rows.jacobian.row(3) = rows.torque_axes[0].transpose() * relative_rate;
    rows.jacobian.row(4) = rows.torque_axes[1].transpose() * relative_rate;
    rows.error.residual.resize(5);
    rows.error.residual << gap, u1.dot(axis_b), u2.dot(axis_b);
    rows.error.rotational = std::atan2(axis_a.cross(axis_b).norm(), axis_a.dot(axis_b));
    return rows;
}

std::vector<MultibodyProvider::ConstraintRows> MultibodyProvider::all_rows(const Kinematics& kin) const {
    std::vector<ConstraintRows> rows;
    for (Eigen::Index c = 0; c < count(model_.constraints.size()); ++c) {
        ConstraintRows r = constraint_rows(kin, c);
        if (at(constraint_enabled_, c) == 0) {
            r.jacobian.setZero();
            r.error.residual.setZero();
            r.error.translational = 0.0;
            r.error.rotational = 0.0;
        }
        rows.push_back(r);
    }
    return rows;
}

std::vector<ConstraintError> MultibodyProvider::constraint_errors(const Eigen::VectorXd& q) const {
    const Kinematics kin = kinematics(q, Eigen::VectorXd::Zero(nv_));
    std::vector<ConstraintError> errors;
    for (const ConstraintRows& rows : all_rows(kin)) {
        errors.push_back(rows.error);
    }
    return errors;
}

// Block Gauss-Seidel over constraint impulses: each sweep solves every constraint's block of
// J A^-1 J^T lambda = -J v exactly (pseudo-inverse, so rows with no reachable motion stay at zero).
int MultibodyProvider::solve_constraints(double t, double /*h*/, const Eigen::VectorXd& q,
                                         const Eigen::MatrixXd& system, Eigen::VectorXd& v) const {
    const Eigen::Index constraints = count(model_.constraints.size());
    impulse_.clear();
    if (constraints == 0) {
        return 0;
    }
    const std::vector<ConstraintRows> rows = all_rows(kinematics(q, v));
    const Eigen::PartialPivLU<Eigen::MatrixXd> lu(system);
    std::vector<Eigen::MatrixXd> response;
    std::vector<Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>> block;
    for (const ConstraintRows& r : rows) {
        const Eigen::MatrixXd m = lu.solve(r.jacobian.transpose());
        response.push_back(m);
        block.emplace_back(r.jacobian * m);
        impulse_.emplace_back(Eigen::VectorXd::Zero(r.jacobian.rows()));
    }
    for (int iteration = 1; iteration <= config_.iteration_cap; ++iteration) {
        for (Eigen::Index c = 0; c < constraints; ++c) {
            const Eigen::VectorXd delta = -at(block, c).solve(at(rows, c).jacobian * v);
            at(impulse_, c) += delta;
            v += at(response, c) * delta;
        }
        double residual = 0.0;
        for (const ConstraintRows& r : rows) {
            residual = std::max(residual, (r.jacobian * v).cwiseAbs().maxCoeff());
        }
        if (residual <= config_.velocity_tolerance) {
            return iteration;
        }
    }
    throw SolverNonConvergenceError(
        t, "constraint solver did not converge within " + std::to_string(config_.iteration_cap) + " iterations");
}

// Gauss-Newton on the stacked position residual (minimum-norm step), then the ENG-024 drift check.
double MultibodyProvider::project_positions(double t, Eigen::VectorXd& q, Eigen::VectorXd& /*v*/) const {
    const Eigen::Index constraints = count(model_.constraints.size());
    error_.clear();
    if (constraints == 0) {
        return 0.0;
    }
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(nv_);
    for (int iteration = 0; iteration < config_.projection_iterations; ++iteration) {
        const std::vector<ConstraintRows> rows = all_rows(kinematics(q, zero));
        Eigen::Index total = 0;
        for (const ConstraintRows& r : rows) {
            total += r.jacobian.rows();
        }
        Eigen::MatrixXd jacobian(total, nv_);
        Eigen::VectorXd residual(total);
        Eigen::Index row = 0;
        for (const ConstraintRows& r : rows) {
            jacobian.middleRows(row, r.jacobian.rows()) = r.jacobian;
            residual.segment(row, r.jacobian.rows()) = r.error.residual;
            row += r.jacobian.rows();
        }
        const Eigen::VectorXd correction = -Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(jacobian).solve(
            residual);
        q = integrate_positions(q, correction, 1.0);
    }
    error_ = constraint_errors(q);
    double drift = 0.0;
    for (Eigen::Index c = 0; c < constraints; ++c) {
        const ConstraintError& e = at(error_, c);
        drift = std::max({drift, e.translational, e.rotational});
        if (e.translational > config_.drift_threshold_m || e.rotational > config_.drift_threshold_rad) {
            throw DriftError(t, "constraint " + std::to_string(c) + " drift exceeds the threshold", c);
        }
    }
    return drift;
}

// ===== Members, ropes and model changes ==================================

// A rope attach point or node: world position and velocity, the holding body's angular velocity (world), and on
// request the 3 x nv point and angular Jacobians (world_jacobian rows, as constraint_rows uses them).
struct MultibodyProvider::RopePoint {
    Eigen::Index body = -1;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd angular_jacobian;
};

namespace {

// Ruling 14: the top attachment's spherical-joint torque coefficient and the relative swing rate omega_rel.
struct Attachment {
    Eigen::Vector3d rate = Eigen::Vector3d::Zero();  // omega_rel, world, rad/s
    double coefficient = 0.0;                        // N m s/rad
    bool active = false;
};

Attachment attachment(const Rope& rope, const Eigen::Vector3d& n, double d, const Eigen::Vector3d& relative_velocity,
                      const Eigen::Vector3d& from_angular_velocity) {
    Attachment out;
    if (d <= 0.0 || (rope.attachment_damping == 0.0 && rope.attachment_friction == 0.0)) {
        return out;
    }
    const Eigen::Matrix3d across = Eigen::Matrix3d::Identity() - n * n.transpose();
    out.rate = across * (n.cross(relative_velocity) / d - from_angular_velocity);
    out.coefficient = rope.attachment_damping;
    const double speed = out.rate.norm();
    if (speed > 0.0) {
        out.coefficient += rope.attachment_friction / speed;
    }
    out.active = out.coefficient > 0.0;
    return out;
}

}  // namespace

MultibodyProvider::RopePoint MultibodyProvider::rope_point(const Kinematics& kin, Eigen::Index body,
                                                           const Eigen::Vector3d& local, bool jacobian) const {
    RopePoint point;
    point.body = body;
    point.position = local;
    if (jacobian) {
        point.jacobian = Eigen::MatrixXd::Zero(3, nv_);
        point.angular_jacobian = Eigen::MatrixXd::Zero(3, nv_);
    }
    if (body < 0) {
        return point;
    }
    const Transform& x = at(kin.world, body);
    const Vector6d& velocity = at(kin.velocity, body);
    const Eigen::Matrix3d to_world = x.rotation.transpose();
    point.position = world_point(x, local);
    point.angular_velocity = to_world * velocity.head<3>();
    point.velocity = to_world * (velocity.tail<3>() + velocity.head<3>().cross(local));
    if (jacobian) {
        const Matrix6Xd j = world_jacobian(kin, body);
        point.jacobian = j.bottomRows<3>() - spatial::skew(point.position) * j.topRows<3>();
        point.angular_jacobian = j.topRows<3>();
    }
    return point;
}

std::vector<MultibodyProvider::RopePoint> MultibodyProvider::rope_points(const Kinematics& kin, Eigen::Index rope,
                                                                         bool jacobian) const {
    const Rope& r = at(model_.ropes, rope);
    std::vector<RopePoint> points;
    points.push_back(rope_point(kin, r.from_body, r.from_point, jacobian));
    for (const Eigen::Index node : r.nodes) {
        points.push_back(rope_point(kin, node, Eigen::Vector3d::Zero(), jacobian));
    }
    points.push_back(rope_point(kin, r.to_body, r.to_point, jacobian));
    return points;
}

std::vector<RopeSegment> MultibodyProvider::rope_segments(const std::vector<RopePoint>& points,
                                                          Eigen::Index rope) const {
    const Rope& r = at(model_.ropes, rope);
    const double l = at(rope_rest_, rope) / r.segments;
    std::vector<RopeSegment> segments;
    for (int s = 0; s < r.segments; ++s) {
        const RopePoint& a = at(points, s);
        const RopePoint& b = at(points, s + 1);
        if (s == r.segments - 1 && at(rope_attached_, rope) == 0) {
            segments.emplace_back();
            continue;
        }
        segments.push_back(rope_segment(a.position, a.velocity, b.position, b.velocity, l, r.axial_rigidity / l,
                                        r.damping / l));
    }
    return segments;
}

void MultibodyProvider::add_rope_wrenches(const Kinematics& kin, std::vector<Vector6d>& wrenches) const {
    const auto apply = [&](const RopePoint& p, const Eigen::Vector3d& torque, const Eigen::Vector3d& force) {
        if (p.body >= 0) {
            at(wrenches, p.body) += to_body(at(kin.world, p.body), p.position, torque, force);
        }
    };
    for (Eigen::Index r = 0; r < count(model_.ropes.size()); ++r) {
        const std::vector<RopePoint> points = rope_points(kin, r, false);
        const std::vector<RopeSegment> segments = rope_segments(points, r);
        for (Eigen::Index s = 0; s < count(segments.size()); ++s) {
            const RopeSegment& seg = at(segments, s);
            if (!seg.taut) {
                continue;
            }
            const Eigen::Vector3d force = seg.tension * seg.direction;
            apply(at(points, s), Eigen::Vector3d::Zero(), force);
            apply(at(points, s + 1), Eigen::Vector3d::Zero(), -force);
        }
        if (at(model_.ropes, r).segments == 1 && at(rope_attached_, r) == 0) {
            continue;
        }
        const RopePoint& a = at(points, 0);
        const RopePoint& b = at(points, 1);
        const RopeSegment& top = at(segments, 0);
        const Attachment joint =
            attachment(at(model_.ropes, r), top.direction, top.length, b.velocity - a.velocity, a.angular_velocity);
        if (!joint.active) {
            continue;
        }
        const Eigen::Vector3d torque = -joint.coefficient * joint.rate;
        const Eigen::Vector3d end_force = torque.cross(top.direction) / top.length;
        apply(b, Eigen::Vector3d::Zero(), end_force);
        apply(a, -torque, -end_force);
    }
}

void MultibodyProvider::add_rope_terms(const Kinematics& kin, DynamicsTerms& terms) const {
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    for (Eigen::Index r = 0; r < count(model_.ropes.size()); ++r) {
        const Rope& rope = at(model_.ropes, r);
        const std::vector<RopePoint> points = rope_points(kin, r, true);
        const std::vector<RopeSegment> segments = rope_segments(points, r);
        const double l = at(rope_rest_, r) / rope.segments;
        for (Eigen::Index s = 0; s < count(segments.size()); ++s) {
            const RopeSegment& seg = at(segments, s);
            if (!seg.taut) {
                continue;
            }
            const Eigen::Vector3d& n = seg.direction;
            const Eigen::Matrix3d axial = n * n.transpose();
            const Eigen::Matrix3d stiffness =
                (rope.axial_rigidity / l) * axial + (seg.tension / seg.length) * (identity - axial);
            const Eigen::MatrixXd relative = at(points, s + 1).jacobian - at(points, s).jacobian;
            terms.stiffness += relative.transpose() * stiffness * relative;
            terms.damping += (rope.damping / l) * relative.transpose() * axial * relative;
        }
        if (rope.segments == 1 && at(rope_attached_, r) == 0) {
            continue;
        }
        const RopePoint& a = at(points, 0);
        const RopePoint& b = at(points, 1);
        const RopeSegment& top = at(segments, 0);
        const Attachment joint = attachment(rope, top.direction, top.length, b.velocity - a.velocity,
                                            a.angular_velocity);
        if (!joint.active) {
            continue;
        }
        const Eigen::Matrix3d across = identity - top.direction * top.direction.transpose();
        const Eigen::MatrixXd rate = spatial::skew(top.direction) * (b.jacobian - a.jacobian) / top.length -
                                     across * a.angular_jacobian;
        terms.damping += joint.coefficient * rate.transpose() * rate;
    }
}

void MultibodyProvider::setup_members_and_ropes() {
    const Eigen::Index bodies = count(model_.bodies.size());
    const Eigen::Index constraints = count(model_.constraints.size());
    constraint_enabled_.assign(model_.constraints.size(), 1);
    for (Eigen::Index m = 0; m < count(model_.members.size()); ++m) {
        const LumpedBeam& beam = at(model_.members, m);
        const std::string label = "member " + std::to_string(m) + ": ";
        if (beam.bodies.empty()) {
            throw InvalidModelError(0.0, label + "has no bodies");
        }
        for (const Eigen::Index b : beam.bodies) {
            if (b < 0 || b >= bodies) {
                throw InvalidModelError(0.0, label + "references a missing body");
            }
        }
        if (beam.parent < -1 || beam.parent >= bodies || beam.roller_constraint < -1 ||
            beam.roller_constraint >= constraints) {
            throw InvalidModelError(0.0, label + "references a missing parent or constraint");
        }
    }
    for (Eigen::Index r = 0; r < count(model_.ropes.size()); ++r) {
        const Rope& rope = at(model_.ropes, r);
        validate_rope_parameters(rope, r);
        const std::string label = "rope " + std::to_string(r) + ": ";
        if (rope.from_body < -1 || rope.from_body >= bodies || rope.to_body < -1 || rope.to_body >= bodies) {
            throw InvalidModelError(0.0, label + "references a missing body");
        }
        if (count(rope.nodes.size()) != rope.segments - 1) {
            throw InvalidModelError(0.0, label + "needs segments - 1 nodes (build it with add_rope)");
        }
        for (const Eigen::Index node : rope.nodes) {
            if (node < 0 || node >= bodies || at(model_.bodies, node).joint.type != JointType::Translation) {
                throw InvalidModelError(0.0, label + "node is not a Translation body");
            }
        }
        rope_rest_.push_back(rope.rest_length);
        rope_attached_.push_back(1);
    }
}

void MultibodyProvider::record_members_and_ropes(const State& before, const State& after, double h) {
    outputs_.events = pending_events_;
    pending_events_.clear();
    outputs_.members.clear();
    if (!model_.members.empty()) {
        const std::vector<BodyOutput> poses = body_states(before.q, before.v);
        for (const LumpedBeam& beam : model_.members) {
            outputs_.members.push_back(member_end_loads(beam, poses, outputs_.joints, outputs_.constraints));
        }
    }
    outputs_.ropes = rope_states(after.q, after.v);
    for (Eigen::Index r = 0; r < count(model_.ropes.size()); ++r) {
        const Rope& rope = at(model_.ropes, r);
        RopeOutput& out = at(outputs_.ropes, r);
        if (rope.to_body >= 0) {
            const double mass = at(model_.bodies, rope.to_body).mass;
            out.snatch_omega_h = std::sqrt(rope.axial_rigidity / (at(rope_rest_, r) * mass)) * h;
            out.snatch_underestimated = out.snatch_omega_h > Rope::snatch_resolution_limit;
        }
        if (at(rope_attached_, r) == 0 || !(out.max_tension > rope.breaking_tension)) {
            continue;
        }
        Eigen::Index worst = 0;
        for (Eigen::Index s = 0; s < count(out.tension.size()); ++s) {
            if (at(out.tension, s) > at(out.tension, worst)) {
                worst = s;
            }
        }
        at(rope_attached_, r) = 0;
        out.attached = false;
        ModelChangeEvent event;
        event.kind = ModelChange::RopeSnapped;
        event.target = r;
        event.index = worst;
        event.value = out.max_tension;
        event.limit = rope.breaking_tension;
        event.time = after.t;
        outputs_.events.push_back(event);
    }
}

std::vector<RopeOutput> MultibodyProvider::rope_states(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const {
    if (q.size() != nq_ || v.size() != nv_) {
        throw InvalidModelError(0.0, "rope state needs q and v of the model's sizes");
    }
    const Kinematics kin = kinematics(q, v);
    std::vector<RopeOutput> out;
    for (Eigen::Index r = 0; r < count(model_.ropes.size()); ++r) {
        RopeOutput rope;
        for (const RopeSegment& seg : rope_segments(rope_points(kin, r, false), r)) {
            rope.tension.push_back(seg.tension);
            rope.max_tension = std::max(rope.max_tension, seg.tension);
        }
        rope.load_end_tension = rope.tension.back();
        rope.attached = at(rope_attached_, r) != 0;
        rope.rest_length = at(rope_rest_, r);
        out.push_back(rope);
    }
    return out;
}

void MultibodyProvider::set_spring_offset(Eigen::Index member, Eigen::Index node, double offset) {
    if (member < 0 || member >= count(model_.members.size())) {
        throw InvalidModelError(0.0, "spring offset on a missing member");
    }
    const std::vector<Eigen::Index> nodes = member_nodes(at(model_.members, member));
    if (node < 0 || node >= count(nodes.size())) {
        throw InvalidModelError(0.0, "spring offset on a missing node");
    }
    if (!std::isfinite(offset)) {
        throw InvalidModelError(0.0, "spring offset is not finite");
    }
    const Eigen::Index body = at(nodes, node);
    const Joint& joint = at(model_.bodies, body).joint;
    double built = 0.0;
    if (joint.rest.size() > 0) {
        built = joint.rest[0];
    }
    rest_[at(v_offset_, body)] = built + offset;
    pending_events_.push_back({ModelChange::SpringOffset, member, node, offset, 0.0, outputs_.time});
}

void MultibodyProvider::set_spring_stiffness(Eigen::Index member, Eigen::Index node, double stiffness) {
    if (member < 0 || member >= count(model_.members.size())) {
        throw InvalidModelError(0.0, "spring stiffness on a missing member");
    }
    const std::vector<Eigen::Index> nodes = member_nodes(at(model_.members, member));
    if (node < 0 || node >= count(nodes.size())) {
        throw InvalidModelError(0.0, "spring stiffness on a missing node");
    }
    if (!std::isfinite(stiffness) || stiffness < 0.0) {
        throw InvalidModelError(0.0, "spring stiffness must be finite and not negative");
    }
    stiffness_[at(v_offset_, at(nodes, node))] = stiffness;
    pending_events_.push_back({ModelChange::SpringStiffness, member, node, stiffness, 0.0, outputs_.time});
}

void MultibodyProvider::check_constraint_change(Eigen::Index constraint) const {
    if (constraint < 0 || constraint >= count(model_.constraints.size())) {
        throw InvalidModelError(0.0, "change on a missing constraint");
    }
    if (at(constraint_enabled_, constraint) == 0) {
        throw InvalidModelError(0.0, "constraint " + std::to_string(constraint) + " is already disabled");
    }
}

void MultibodyProvider::disable_constraint(Eigen::Index constraint) {
    check_constraint_change(constraint);
    at(constraint_enabled_, constraint) = 0;
    pending_events_.push_back({ModelChange::ConstraintDisabled, constraint, -1, 0.0, 0.0, outputs_.time});
}

void MultibodyProvider::release_load(Eigen::Index constraint) {
    check_constraint_change(constraint);
    at(constraint_enabled_, constraint) = 0;
    pending_events_.push_back({ModelChange::LoadReleased, constraint, -1, 0.0, 0.0, outputs_.time});
}

void MultibodyProvider::release_rope(Eigen::Index rope) {
    if (rope < 0 || rope >= count(model_.ropes.size())) {
        throw InvalidModelError(0.0, "release of a missing rope");
    }
    if (at(rope_attached_, rope) == 0) {
        throw InvalidModelError(0.0, "rope " + std::to_string(rope) + " is already released");
    }
    at(rope_attached_, rope) = 0;
    pending_events_.push_back({ModelChange::RopeReleased, rope, -1, 0.0, 0.0, outputs_.time});
}

bool MultibodyProvider::constraint_enabled(Eigen::Index constraint) const {
    if (constraint < 0 || constraint >= count(model_.constraints.size())) {
        throw InvalidModelError(0.0, "query of a missing constraint");
    }
    return at(constraint_enabled_, constraint) != 0;
}

bool MultibodyProvider::rope_attached(Eigen::Index rope) const {
    if (rope < 0 || rope >= count(model_.ropes.size())) {
        throw InvalidModelError(0.0, "query of a missing rope");
    }
    return at(rope_attached_, rope) != 0;
}

void MultibodyProvider::set_rope_rest_length(Eigen::Index rope, double length) {
    if (rope < 0 || rope >= count(model_.ropes.size())) {
        throw InvalidModelError(0.0, "rest length on a missing rope");
    }
    Rope changed = at(model_.ropes, rope);
    changed.rest_length = length;
    validate_rope_parameters(changed, rope);
    at(rope_rest_, rope) = length;
}

// ===== Inputs and outputs ================================================

void MultibodyProvider::set_generalized_forces(const Eigen::VectorXd& tau) {
    if (tau.size() != nv_) {
        throw InvalidModelError(0.0, "generalized forces must have one entry per velocity coordinate");
    }
    if (!tau.allFinite()) {
        throw InvalidModelError(0.0, "generalized forces are not finite");
    }
    tau_ = tau;
}

void MultibodyProvider::apply_body_wrench(Eigen::Index body, const Eigen::Vector3d& torque,
                                          const Eigen::Vector3d& force) {
    if (body < 0 || body >= count(model_.bodies.size())) {
        throw InvalidModelError(0.0, "wrench applied to a missing body");
    }
    if (!torque.allFinite() || !force.allFinite()) {
        throw InvalidModelError(0.0, "applied wrench is not finite");
    }
    at(wrench_, body).head<3>() += torque;
    at(wrench_, body).tail<3>() += force;
}

void MultibodyProvider::clear_inputs() {
    tau_ = Eigen::VectorXd::Zero(nv_);
    wrench_.assign(model_.bodies.size(), Vector6d::Zero());
    shaft_torque_.assign(model_.transmissions.size(), 0.0);
}

void MultibodyProvider::set_shaft_torque(Eigen::Index transmission, double torque) {
    if (transmission < 0 || transmission >= count(model_.transmissions.size())) {
        throw InvalidModelError(0.0, "shaft torque applied to a missing transmission");
    }
    if (!std::isfinite(torque)) {
        throw InvalidModelError(0.0, "shaft torque is not finite");
    }
    at(shaft_torque_, transmission) = torque;
}

void MultibodyProvider::set_slip(Eigen::Index transmission, double slip) {
    if (transmission < 0 || transmission >= count(model_.transmissions.size())) {
        throw InvalidModelError(0.0, "slip set on a missing transmission");
    }
    Transmission changed = at(model_.transmissions, transmission);
    changed.slip = slip;
    validate_transmission_parameters(changed, transmission);
    at(slip_, transmission) = slip;
}

double MultibodyProvider::slip(Eigen::Index transmission) const {
    if (transmission < 0 || transmission >= count(model_.transmissions.size())) {
        throw InvalidModelError(0.0, "slip read from a missing transmission");
    }
    return at(slip_, transmission);
}

std::vector<TransmissionOutput> MultibodyProvider::transmission_states(const Eigen::VectorXd& q,
                                                                       const Eigen::VectorXd& v) const {
    if (q.size() != nq_ || v.size() != nv_) {
        throw InvalidModelError(0.0, "transmission state needs q and v of the model's sizes");
    }
    std::vector<TransmissionOutput> out;
    for (Eigen::Index i = 0; i < count(model_.transmissions.size()); ++i) {
        TransmissionOutput published = coupling(i, q, v).output;
        published.shaft_load_torque = -published.shaft_load_torque;
        out.push_back(published);
    }
    return out;
}

std::vector<BodyOutput> MultibodyProvider::body_states(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const {
    const Kinematics kin = kinematics(q, v);
    std::vector<BodyOutput> out;
    for (Eigen::Index i = 0; i < count(model_.bodies.size()); ++i) {
        const Transform& x = at(kin.world, i);
        BodyOutput body;
        body.position = x.translation;
        body.orientation = Eigen::Quaterniond(Eigen::Matrix3d(x.rotation.transpose())).normalized();
        body.spatial_velocity = x.inverse_motion(at(kin.velocity, i));
        body.origin_velocity = x.rotation.transpose() * at(kin.velocity, i).tail<3>();
        out.push_back(body);
    }
    return out;
}

StepHealth MultibodyProvider::step(Integrator& integrator, State& state) {
    const State before = state;
    StepHealth health;
    try {
        health = integrator.step(*this, state);
    } catch (...) {
        clear_inputs();
        throw;
    }
    for (Eigen::Index i = 0; i < count(model_.transmissions.size()); ++i) {
        const Transmission& t = at(model_.transmissions, i);
        const Eigen::Index in_q = at(q_offset_, t.input_body);
        at(slip_offset_, i) += at(slip_, i) * output_per_input(t) * (state.q[in_q] - before.q[in_q]);
    }
    record_outputs(before, state, integrator.config().h);
    outputs_.transmissions = transmission_states(state.q, state.v);
    record_members_and_ropes(before, state, integrator.config().h);
    clear_inputs();
    return health;
}

void MultibodyProvider::record_outputs(const State& before, const State& after, double h) {
    const Kinematics kin = kinematics(before.q, before.v);
    std::vector<Vector6d> external = applied_body_wrenches(kin);
    outputs_.constraints.clear();
    const std::vector<ConstraintRows> rows = all_rows(kin);
    for (Eigen::Index c = 0; c < count(rows.size()); ++c) {
        const LoopConstraint& lc = at(model_.constraints, c);
        const ConstraintRows& r = at(rows, c);
        ConstraintOutput out;
        out.impulse = Eigen::VectorXd::Zero(r.jacobian.rows());
        if (c < count(impulse_.size())) {
            out.impulse = at(impulse_, c);
        }
        out.force = out.impulse / h;
        if (c < count(error_.size())) {
            out.translational_error = at(error_, c).translational;
            out.rotational_error = at(error_, c).rotational;
        }
        const Eigen::Vector3d force = out.force.head<3>();
        Eigen::Vector3d torque = Eigen::Vector3d::Zero();
        for (std::size_t k = 0; k < r.torque_axes.size(); ++k) {
            torque += out.force[static_cast<Eigen::Index>(3 + k)] * r.torque_axes[k];
        }
        at(external, lc.body_a) += to_body(at(kin.world, lc.body_a), r.point_a, torque, force);
        at(external, lc.body_b) += to_body(at(kin.world, lc.body_b), r.point_b, -torque, -force);
        outputs_.constraints.push_back(out);
    }
    Vector6d base = Vector6d::Zero();
    base.tail<3>() = -model_.gravity;
    const Eigen::VectorXd qdd = (after.v - before.v) / h;
    std::vector<Vector6d> forces;
    static_cast<void>(inverse_dynamics(kin, qdd, base, external, true, forces));
    outputs_.joints.clear();
    for (Eigen::Index i = 0; i < count(model_.bodies.size()); ++i) {
        const Transform& x = at(kin.world, i);
        JointOutput joint;
        joint.generalized_force = at(kin.subspace, i).transpose() * at(forces, i);
        joint.reaction_wrench.head<3>() = x.rotation.transpose() * at(forces, i).head<3>();
        joint.reaction_wrench.tail<3>() = x.rotation.transpose() * at(forces, i).tail<3>();
        outputs_.joints.push_back(joint);
    }
    outputs_.bodies = body_states(after.q, after.v);
    outputs_.contacts.clear();
    outputs_.time = after.t;
}

const MultibodyOutputs& MultibodyProvider::outputs() const noexcept { return outputs_; }

}  // namespace mechsim::engine
