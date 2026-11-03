#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <utility>

#include "mechsim/engine/multibody_model.hpp"

namespace mechsim::engine::test {

constexpr double standard_gravity = 9.81;  // m/s^2
constexpr double rod_radius = 0.01;        // m, for the rod's inertia about its own axis

// Uniform rod along local +x on a revolute joint about z, its joint frame at `position` in the parent frame.
inline Body rod(std::string name, Eigen::Index parent, const Eigen::Vector3d& position, double length, double mass) {
    Body body;
    body.name = std::move(name);
    body.parent = parent;
    body.joint.type = JointType::Revolute;
    body.joint.position = position;
    body.joint.axis = Eigen::Vector3d::UnitZ();
    body.mass = mass;
    body.com = Eigen::Vector3d(0.5 * length, 0.0, 0.0);
    const double transverse = mass * length * length / 12.0;
    body.inertia = Eigen::Vector3d(0.5 * mass * rod_radius * rod_radius, transverse, transverse).asDiagonal();
    return body;
}

struct FourBar {
    double crank = 0.0;    // a, m
    double coupler = 0.0;  // b, m
    double rocker = 0.0;   // c, m
    double ground = 0.0;   // d, m
    double density = 1.0;  // kg per m of link
};

// Crank (body 0) pivots at the world origin, coupler (body 1) at the crank tip, rocker (body 2) at (d, 0, 0); a
// point constraint closes the coupler tip onto the rocker tip. All joints rotate about z.
inline MultibodyModel four_bar_model(const FourBar& bar, const Eigen::Vector3d& gravity) {
    MultibodyModel model;
    model.gravity = gravity;
    model.bodies.push_back(rod("crank", -1, Eigen::Vector3d::Zero(), bar.crank, bar.density * bar.crank));
    model.bodies.push_back(
        rod("coupler", 0, Eigen::Vector3d(bar.crank, 0.0, 0.0), bar.coupler, bar.density * bar.coupler));
    model.bodies.push_back(
        rod("rocker", -1, Eigen::Vector3d(bar.ground, 0.0, 0.0), bar.rocker, bar.density * bar.rocker));
    LoopConstraint closure;
    closure.type = ConstraintType::Point;
    closure.body_a = 1;
    closure.body_b = 2;
    closure.point_a = Eigen::Vector3d(bar.coupler, 0.0, 0.0);
    closure.point_b = Eigen::Vector3d(bar.rocker, 0.0, 0.0);
    model.constraints.push_back(closure);
    return model;
}

// Closed configuration [crank angle, coupler angle relative to crank, rocker angle] for a crank angle, with the
// coupler tip on the left of the crank-tip-to-rocker-pivot line.
inline Eigen::Vector3d four_bar_closure(const FourBar& bar, double crank_angle) {
    const Eigen::Vector2d tip(bar.crank * std::cos(crank_angle), bar.crank * std::sin(crank_angle));
    const Eigen::Vector2d pivot(bar.ground, 0.0);
    const double distance = (pivot - tip).norm();
    const Eigen::Vector2d along_unit = (pivot - tip) / distance;
    const double along =
        (bar.coupler * bar.coupler - bar.rocker * bar.rocker + distance * distance) / (2.0 * distance);
    const double across = std::sqrt(bar.coupler * bar.coupler - along * along);
    const Eigen::Vector2d left(-along_unit.y(), along_unit.x());
    const Eigen::Vector2d joint = tip + along * along_unit + across * left;
    const double coupler_angle = std::atan2(joint.y() - tip.y(), joint.x() - tip.x());
    const double rocker_angle = std::atan2(joint.y() - pivot.y(), joint.x() - pivot.x());
    return {crank_angle, coupler_angle - crank_angle, rocker_angle};
}

// Grashof crank-rocker (s + l = 0.5 m < p + q = 0.7 m, crank shortest): the crank turns fully.
inline FourBar crank_rocker() { return FourBar{0.1, 0.4, 0.3, 0.4, 1.0}; }

}  // namespace mechsim::engine::test
