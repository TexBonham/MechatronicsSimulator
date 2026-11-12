#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>

#include "mechsim/engine/member_loads.hpp"
#include "mechsim/engine/rope.hpp"
#include "mechsim/engine/transmission.hpp"

namespace mechsim::engine {

// Joint coordinates (q; v), ENG-006 to ENG-011:
//   Revolute  1; 1  angle about axis, rad
//   Prismatic 1; 1  displacement along axis, m
//   Fixed     0; 0
//   Planar    3; 3  [x, y, theta]: translation x u + y w in the plane normal to axis, then rotation theta about it
//   Spherical 4; 3  q unit quaternion [w, x, y, z]; v angular velocity in child coordinates, rad/s
//   Free      7; 6  q [p (m, parent joint frame), quaternion]; v [omega; velocity of child origin], child coordinates
//   Translation 3; 3  q p (m, parent joint frame); v its rate; no rotation (rope nodes, ENG-047)
enum class JointType : std::uint8_t { Revolute, Prismatic, Fixed, Planar, Spherical, Free, Translation };

// The joint frame is placed in the parent body frame; at zero joint coordinates the child body frame equals it.
// Joint springs and dampers (ENG-051), per velocity DOF, each empty (none) or sized to the joint's velocity count:
//   f = -stiffness (q - rest) - damping v; K and D of DynamicsTerms carry them, so the integrator treats them
//   implicitly (ADR 0013). Stiffness and rest need q and v aligned (Revolute, Prismatic, Planar); Spherical and
//   Free joints take damping only.
struct Joint {
    JointType type = JointType::Revolute;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();                // joint frame origin in parent frame, m
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // joint frame relative to parent, unit
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();                   // unit, joint frame; plane normal for Planar
    Eigen::VectorXd stiffness;  // finite, >= 0; N/m or N m/rad
    Eigen::VectorXd damping;    // finite, >= 0; N s/m or N m s/rad
    Eigen::VectorXd rest;       // finite; rest coordinates, m or rad; empty is zero
    // Travel limits (ENG-013), Revolute and Prismatic only, each empty (none) or one entry: one-sided rows in the
    // velocity pass, active once the coordinate has reached the bound, zero restitution; the position projection
    // returns a coordinate that passed the bound to it. -inf / +inf leaves that side open; lower == upper locks.
    Eigen::VectorXd lower;      // not NaN, not +inf; m or rad
    Eigen::VectorXd upper;      // not NaN, not -inf, >= lower; m or rad
    // Coulomb friction magnitude per velocity DOF (ENG-014, ruling 38), Revolute, Prismatic, Planar; empty is none:
    // an implicit damping friction / max(|v_start|, friction_slip_threshold) frozen at the step's start rate, so a
    // load L below it creeps at friction_slip_threshold L / friction.
    Eigen::VectorXd friction;   // finite, >= 0; N or N m
};

struct Body {
    std::string name;
    Eigen::Index parent = -1;                                      // -1 is the fixed world; else an earlier body
    Joint joint;                                                   // connects this body to its parent
    double mass = 1.0;                                             // kg, finite and > 0
    Eigen::Vector3d com = Eigen::Vector3d::Zero();                 // center of mass in body frame, m
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();         // about the center of mass, body frame, kg m^2
};

// Bilateral loop-closure constraint between two distinct bodies (ENG-012).
//   Point (3 equations): point_a on body_a coincides with point_b on body_b.
//   Hinge (5 equations): Point plus axis_a on body_a parallel to axis_b on body_b.
enum class ConstraintType : std::uint8_t { Point, Hinge };

struct LoopConstraint {
    ConstraintType type = ConstraintType::Point;
    Eigen::Index body_a = 0;
    Eigen::Index body_b = 0;
    Eigen::Vector3d point_a = Eigen::Vector3d::Zero();  // body_a frame, m
    Eigen::Vector3d point_b = Eigen::Vector3d::Zero();  // body_b frame, m
    Eigen::Vector3d axis_a = Eigen::Vector3d::UnitZ();  // unit, body_a frame (Hinge)
    Eigen::Vector3d axis_b = Eigen::Vector3d::UnitZ();  // unit, body_b frame (Hinge)
};

// Bodies in topological order: each parent precedes its child (ENG-005).
struct MultibodyModel {
    std::vector<Body> bodies;
    std::vector<LoopConstraint> constraints;
    std::vector<Transmission> transmissions;  // ENG-043 to ENG-046, ENG-048
    std::vector<LumpedBeam> members;          // ENG-030; add_lumped_beam registers each beam
    std::vector<Rope> ropes;                  // ENG-047; add_rope registers each rope
    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);  // world frame, m/s^2
};

// Run-configuration values for the constraint solver; the defaults are the spec values (docs/spec/eng.md).
struct MultibodyConfig {
    static constexpr int default_iteration_cap = 50;                // ENG-025
    static constexpr double default_velocity_tolerance = 1e-9;     // m/s or rad/s, constraint velocity residual
    static constexpr int default_projection_iterations = 3;         // Gauss-Newton position projection
    static constexpr double default_drift_threshold_m = 1e-6;      // ENG-024
    static constexpr double default_drift_threshold_rad = 1e-6;    // ENG-024
    static constexpr double default_unit_tolerance = 1e-12;        // |norm^2 - 1| of axes and quaternions
    static constexpr double default_friction_slip_threshold = 1e-4;  // rad/s; Coulomb friction regularisation

    int iteration_cap = default_iteration_cap;                       // >= 1
    double velocity_tolerance = default_velocity_tolerance;          // finite, > 0
    int projection_iterations = default_projection_iterations;       // >= 0
    double drift_threshold_m = default_drift_threshold_m;            // finite, > 0
    double drift_threshold_rad = default_drift_threshold_rad;        // finite, > 0
    double unit_tolerance = default_unit_tolerance;                  // finite, > 0
    // Below this relative rate Coulomb friction (the rope attachment's and the joints') acts as the viscous damping
    // friction / threshold, so D stays bounded at zero rate; at and above it the torque is the full Coulomb value.
    double friction_slip_threshold = default_friction_slip_threshold;  // finite, > 0
};

}  // namespace mechsim::engine
