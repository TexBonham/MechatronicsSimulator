#include "mechsim/engine/member_loads.hpp"

#include <cstddef>

#include "mechsim/engine/multibody_provider.hpp"

namespace mechsim::engine {

std::vector<Eigen::Index> member_nodes(const LumpedBeam& beam) {
    std::vector<Eigen::Index> nodes;
    std::size_t first = 0;
    if (beam.roller >= 0) {
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
    }
    return out;
}

}  // namespace mechsim::engine
