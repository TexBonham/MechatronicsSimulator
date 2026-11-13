#include "mechsim/engine/member_loads.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <cstddef>

#include "mechsim/engine/multibody_provider.hpp"

namespace mechsim::engine {

std::vector<Eigen::Index> member_nodes(const LumpedBeam& beam) {
    std::vector<Eigen::Index> nodes;
    std::size_t first = 0;
    if (beam.pinned_root) {
        first = 1;
    }
    for (std::size_t k = first; k < beam.bodies.size(); ++k) {
        nodes.push_back(beam.bodies[k]);
    }
    return nodes;
}

MemberOutput member_end_loads(const LumpedBeam& beam, const std::vector<BodyOutput>& poses,
                              const std::vector<JointOutput>& joints,
                              const std::vector<ConstraintOutput>& constraints) {
    MemberOutput out;
    if (beam.bodies.empty()) {
        return out;
    }
    Eigen::Matrix3d parent_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d parent_position = Eigen::Vector3d::Zero();
    if (beam.parent >= 0) {
        const BodyOutput& parent = poses.at(static_cast<std::size_t>(beam.parent));
        parent_rotation = parent.orientation.toRotationMatrix();
        parent_position = parent.position;
    }
    const Eigen::Matrix3d root_rotation = parent_rotation * beam.orientation.toRotationMatrix();
    const Eigen::Vector3d root = parent_position + parent_rotation * beam.position;

    const auto first = static_cast<std::size_t>(beam.bodies.front());
    const spatial::Vector6d& reaction = joints.at(first).reaction_wrench;
    const Eigen::Vector3d force = reaction.tail<3>();
    const Eigen::Vector3d moment = reaction.head<3>() + (poses.at(first).position - root).cross(force);
    out.start_force = root_rotation.transpose() * force;
    out.start_moment = root_rotation.transpose() * moment;

    if (beam.roller_constraint >= 0) {
        const ConstraintOutput& end = constraints.at(static_cast<std::size_t>(beam.roller_constraint));
        out.end_force = root_rotation.transpose() * end.force.head<3>();
        out.end_moment = root_rotation.transpose() * end.torque;
    }
    return out;
}

std::vector<LoadNode> load_nodes(const LumpedBeam& beam) {
    std::vector<LoadNode> nodes;
    if (beam.bodies.empty()) {
        return nodes;
    }
    LoadNode root;
    root.body = beam.parent;
    root.point = beam.position;
    if (beam.pinned_root) {
        root.body = beam.bodies.front();
        root.point = Eigen::Vector3d::Zero();
    }
    nodes.push_back(root);
    for (std::size_t k = 0; k < beam.bodies.size(); ++k) {
        if (beam.starts[k] > 0.0) {
            nodes.push_back(LoadNode{beam.starts[k], beam.bodies[k], Eigen::Vector3d::Zero()});
        }
    }
    LoadNode tip;
    tip.x = beam.length;
    tip.body = beam.bodies.back();
    tip.point = Eigen::Vector3d(beam.length - beam.starts.back(), 0.0, 0.0);
    nodes.push_back(tip);
    return nodes;
}

CarriedPair carried_pair(const LumpedBeam& beam, Eigen::Index carried, const std::vector<Eigen::Matrix3d>& rotation,
                         const std::vector<Eigen::Vector3d>& origin, const Eigen::Matrix<double, 6, 1>& load) {
    const auto world_point = [&](Eigen::Index body, const Eigen::Vector3d& local) -> Eigen::Vector3d {
        if (body < 0) {
            return local;
        }
        const auto b = static_cast<std::size_t>(body);
        return origin.at(b) + (rotation.at(b) * local);
    };
    Eigen::Matrix3d parent_rotation = Eigen::Matrix3d::Identity();
    if (beam.parent >= 0) {
        parent_rotation = rotation.at(static_cast<std::size_t>(beam.parent));
    }
    const Eigen::Matrix3d root_rotation = parent_rotation * beam.orientation.toRotationMatrix();
    const Eigen::Vector3d root = world_point(beam.parent, beam.position);
    const Eigen::Vector3d axis_x = root_rotation.col(0);

    CarriedPair pair;
    pair.axis_y = root_rotation.col(1);
    pair.axis_z = root_rotation.col(2);
    const Eigen::Vector3d r_c = origin.at(static_cast<std::size_t>(carried));
    const double s_raw = (r_c - root).dot(axis_x);
    double s = s_raw;
    if (s < 0.0) {
        pair.excursion = -s;
        s = 0.0;
    }
    if (s > beam.length) {
        pair.excursion = s - beam.length;
        s = beam.length;
    }
    const Eigen::Vector3d foot = root + (s_raw * axis_x);
    const Eigen::Vector3d force = load.tail<3>();
    const Eigen::Vector3d moment = load.head<3>() + (r_c - foot).cross(force);

    const std::vector<LoadNode> nodes = load_nodes(beam);
    std::size_t k = 0;
    while (k + 2 < nodes.size() && nodes[k + 1].x <= s) {
        ++k;
    }
    const LoadNode& lower = nodes[k];
    const LoadNode& upper = nodes[k + 1];
    const double weight = (upper.x - s) / (upper.x - lower.x);

    pair.body_lower = lower.body;
    pair.body_upper = upper.body;
    pair.point_lower = world_point(lower.body, lower.point);
    pair.point_upper = world_point(upper.body, upper.point);

    CarriedLoadOutput& out = pair.output;
    out.body = carried;
    out.position = s;
    out.force = force.dot(pair.axis_y);
    out.moment = moment.dot(pair.axis_z);
    out.node = static_cast<Eigen::Index>(k);
    out.weight = weight;
    const Eigen::Vector3d interpolated = (weight * pair.point_lower) + ((1.0 - weight) * pair.point_upper);
    out.deflection = (interpolated - (root + (s * axis_x))).dot(pair.axis_y);
    return pair;
}

double lag_growth_factor(double r, double x, double zeta) {
    const double lead = 1.0 + (2.0 * zeta * x) + (x * x);
    Eigen::Matrix3d companion = Eigen::Matrix3d::Zero();
    companion(0, 0) = (2.0 + r + (2.0 * zeta * x)) / lead;
    companion(0, 1) = -(1.0 + (2.0 * r)) / lead;
    companion(0, 2) = r / lead;
    companion(1, 0) = 1.0;
    companion(2, 1) = 1.0;
    const Eigen::EigenSolver<Eigen::Matrix3d> solver(companion, false);
    return solver.eigenvalues().cwiseAbs().maxCoeff();
}

double lag_spectral_radius(const Eigen::MatrixXd& lag, const Eigen::VectorXd& omega_squared,
                           const Eigen::MatrixXd& damping, double h) {
    const Eigen::Index n = omega_squared.size();
    if (n == 0) {
        return 0.0;
    }
    const Eigen::MatrixXd stiffness = omega_squared.asDiagonal();
    const Eigen::MatrixXd system = Eigen::MatrixXd::Identity(n, n) + (h * damping) + ((h * h) * stiffness);
    const Eigen::PartialPivLU<Eigen::MatrixXd> lu(system);
    // a_n = A^-1 (B a_{n-1} - K u_n - (D + h K) v_n) on the state (u_n, v_n, a_{n-1}).
    Eigen::MatrixXd from(n, 3 * n);
    from.leftCols(n) = -stiffness;
    from.middleCols(n, n) = -(damping + (h * stiffness));
    from.rightCols(n) = lag;
    const Eigen::MatrixXd acceleration = lu.solve(from);
    Eigen::MatrixXd step = Eigen::MatrixXd::Zero(3 * n, 3 * n);
    step.topLeftCorner(n, n).setIdentity();
    step.block(0, n, n, n) = h * Eigen::MatrixXd::Identity(n, n);
    step.topRows(n) += (h * h) * acceleration;
    step.block(n, n, n, n).setIdentity();
    step.middleRows(n, n) += h * acceleration;
    step.bottomRows(n) = acceleration;
    const Eigen::EigenSolver<Eigen::MatrixXd> solver(step, false);
    return solver.eigenvalues().cwiseAbs().maxCoeff();
}

double lag_stability_boundary(double x) { return 0.5 * (1.0 + (x * x)); }

}  // namespace mechsim::engine
