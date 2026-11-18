#include "mechsim/engine/member_loads.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>

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

namespace {

Eigen::Vector3d placed_point(Eigen::Index body, const Eigen::Vector3d& local, const std::vector<Eigen::Matrix3d>& rotation,
                             const std::vector<Eigen::Vector3d>& origin) {
    if (body < 0) {
        return local;
    }
    const auto b = static_cast<std::size_t>(body);
    return origin.at(b) + (rotation.at(b) * local);
}

// The member frame and the lever rule at s (clamped to the span by the caller).
void place_pair(const LumpedBeam& beam, double s, const std::vector<Eigen::Matrix3d>& rotation,
                     const std::vector<Eigen::Vector3d>& origin, CarriedPair& pair) {
    Eigen::Matrix3d parent_rotation = Eigen::Matrix3d::Identity();
    if (beam.parent >= 0) {
        parent_rotation = rotation.at(static_cast<std::size_t>(beam.parent));
    }
    const Eigen::Matrix3d root_rotation = parent_rotation * beam.orientation.toRotationMatrix();
    pair.axis_y = root_rotation.col(1);
    pair.axis_z = root_rotation.col(2);
    const std::vector<LoadNode> nodes = load_nodes(beam);
    std::size_t k = 0;
    while (k + 2 < nodes.size() && nodes[k + 1].x <= s) {
        ++k;
    }
    const LoadNode& lower = nodes[k];
    const LoadNode& upper = nodes[k + 1];
    pair.body_lower = lower.body;
    pair.body_upper = upper.body;
    pair.point_lower = placed_point(lower.body, lower.point, rotation, origin);
    pair.point_upper = placed_point(upper.body, upper.point, rotation, origin);
    pair.output.position = s;
    pair.output.node = static_cast<Eigen::Index>(k);
    pair.output.weight = (upper.x - s) / (upper.x - lower.x);
}

}  // namespace

CarriedPair carried_pair_at(const LumpedBeam& beam, double s, const std::vector<Eigen::Matrix3d>& rotation,
                            const std::vector<Eigen::Vector3d>& origin) {
    CarriedPair pair;
    place_pair(beam, std::clamp(s, 0.0, beam.length), rotation, origin, pair);
    return pair;
}

CarriedPair carried_pair(const LumpedBeam& beam, Eigen::Index carried, const std::vector<Eigen::Matrix3d>& rotation,
                         const std::vector<Eigen::Vector3d>& origin, const Eigen::Matrix<double, 6, 1>& load) {
    Eigen::Matrix3d parent_rotation = Eigen::Matrix3d::Identity();
    if (beam.parent >= 0) {
        parent_rotation = rotation.at(static_cast<std::size_t>(beam.parent));
    }
    const Eigen::Vector3d root = placed_point(beam.parent, beam.position, rotation, origin);
    const Eigen::Vector3d axis_x = (parent_rotation * beam.orientation.toRotationMatrix()).col(0);
    const Eigen::Vector3d& r_c = origin.at(static_cast<std::size_t>(carried));
    const double s_raw = (r_c - root).dot(axis_x);
    double s = s_raw;
    double excursion = 0.0;
    if (s < 0.0) {
        excursion = -s;
        s = 0.0;
    }
    if (s > beam.length) {
        excursion = s - beam.length;
        s = beam.length;
    }
    CarriedPair pair;
    place_pair(beam, s, rotation, origin, pair);
    pair.excursion = excursion;
    const Eigen::Vector3d foot = root + (s_raw * axis_x);
    const Eigen::Vector3d force = load.tail<3>();
    const Eigen::Vector3d moment = load.head<3>() + (r_c - foot).cross(force);

    CarriedLoadOutput& out = pair.output;
    out.body = carried;
    out.force = force.dot(pair.axis_y);
    out.moment = moment.dot(pair.axis_z);
    const double weight = out.weight;
    const Eigen::Vector3d interpolated = (weight * pair.point_lower) + ((1.0 - weight) * pair.point_upper);
    out.deflection = (interpolated - (root + (s * axis_x))).dot(pair.axis_y);
    return pair;
}

std::vector<double> node_deflections(const LumpedBeam& beam, const std::vector<Eigen::Matrix3d>& rotation,
                                     const std::vector<Eigen::Vector3d>& origin) {
    Eigen::Matrix3d parent_rotation = Eigen::Matrix3d::Identity();
    if (beam.parent >= 0) {
        parent_rotation = rotation.at(static_cast<std::size_t>(beam.parent));
    }
    const Eigen::Matrix3d root_rotation = parent_rotation * beam.orientation.toRotationMatrix();
    const Eigen::Vector3d root = placed_point(beam.parent, beam.position, rotation, origin);
    std::vector<double> out;
    for (const LoadNode& node : load_nodes(beam)) {
        const Eigen::Vector3d point = placed_point(node.body, node.point, rotation, origin);
        out.push_back((point - (root + (node.x * root_rotation.col(0)))).dot(root_rotation.col(1)));
    }
    return out;
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

double carried_window_periods(double zeta) {
    if (!(zeta > 0.0)) {
        return carried_window_periods_max;
    }
    const double periods = std::log(1.0 / carried_validity_margin) / (2.0 * std::numbers::pi * zeta);
    if (!(periods < carried_window_periods_max)) {
        return carried_window_periods_max;
    }
    return periods;
}

Eigen::Index carried_window_bucket(double x) {
    const double steps = std::floor(std::numbers::pi / (2.0 * x));
    if (!(steps > 1.0)) {
        return 1;
    }
    constexpr double largest = 1e15;
    if (steps > largest) {
        return static_cast<Eigen::Index>(largest);
    }
    return static_cast<Eigen::Index>(steps);
}

}  // namespace mechsim::engine
