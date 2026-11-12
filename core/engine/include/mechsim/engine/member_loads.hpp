#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace mechsim::engine {

struct BodyOutput;
struct JointOutput;
struct ConstraintOutput;

// A lumped member as built by add_lumped_beam (ENG-022); model.members holds one per beam (ENG-030).
struct LumpedBeam {
    Eigen::Index parent = -1;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();               // root frame origin in the parent frame, m
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // root frame in the parent frame
    double length = 0.0;                                              // m
    double node_stiffness = 0.0;                                      // E I / l, N m/rad
    std::vector<Eigen::Index> bodies;                                 // root to tip
    std::vector<double> starts;                                       // x of each body's joint along the beam, m
    Eigen::Index roller = -1;                                         // SimplySupported: the prismatic roller body
    Eigen::Index roller_constraint = -1;  // SimplySupported: point constraint of the far end onto the roller;
                                          // Clamped: hinge constraint (point plus rotation lock about z and y)
    bool pinned_root = false;             // SimplySupported: bodies[0] is the spring-free pin segment
};

// Spring nodes of a member, root to tip: the bodies whose joint carries E I / l (a SimplySupported pin carries none).
// Node j is the spring index of set_spring_offset and set_spring_stiffness (ENG-031, ENG-032).
[[nodiscard]] std::vector<Eigen::Index> member_nodes(const LumpedBeam& beam);

// End-loads of one member (ENG-030, PHY-007): the wrench its support exerts on the member at each end, moments about
// the end point (x = 0 and x = L), both in the member's root frame axes. The start wrench is the root joint's
// reaction (segment springs and joint constraint together) moved from the first joint to x = 0 across the clamped
// stub; the end wrench is the roller constraint's force and torque on the last segment (a pinned far end's point
// constraint carries no moment, a clamped far end's hinge rows carry the end moment), zero for a free end or a
// disabled roller. Cantilever under a tip load P (world -y, tip at x_t): start force +P y,
// start moment P x_t z. Statics, not a fit.
struct MemberOutput {
    Eigen::Vector3d start_moment = Eigen::Vector3d::Zero();  // N m
    Eigen::Vector3d start_force = Eigen::Vector3d::Zero();   // N
    Eigen::Vector3d end_moment = Eigen::Vector3d::Zero();    // N m
    Eigen::Vector3d end_force = Eigen::Vector3d::Zero();     // N
};

// poses at the configuration the reactions were computed at (the step's start state, as record_outputs uses).
[[nodiscard]] MemberOutput member_end_loads(const LumpedBeam& beam, const std::vector<BodyOutput>& poses,
                                            const std::vector<JointOutput>& joints,
                                            const std::vector<ConstraintOutput>& constraints);

}  // namespace mechsim::engine
