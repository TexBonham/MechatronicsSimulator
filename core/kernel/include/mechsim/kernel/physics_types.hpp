#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

// Neutral value types crossing out of a PhysicsAdapter (PHY-012). SI throughout; world frame unless stated.
// Element indices are positions in the neutral model's element lists, in file order.
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // w first, unit

struct Wrench {
    Vec3 moment{};  // N m
    Vec3 force{};   // N
};

struct BodyState {
    Vec3 position{};                 // body origin, m
    Quat orientation{1.0, 0.0, 0.0, 0.0};  // body to world
    Vec3 angular_velocity{};         // rad/s
    Vec3 linear_velocity{};          // body origin, m/s
};

struct JointForce {
    std::vector<double> generalized_force;  // per joint DOF, N or N m
    Wrench reaction{};                      // parent on child, about the child origin, world-aligned
};

struct ConstraintForce {
    std::vector<double> force;  // per constraint row, N or N m
    double translational_error = 0.0;  // m
    double rotational_error = 0.0;     // rad
};

// Body index standing for the world, or a body outside the neutral body list, in ContactForce::body_a and body_b.
inline constexpr std::size_t kWorldBody = static_cast<std::size_t>(-1);

struct ContactForce {
    std::size_t body_a = 0;
    std::size_t body_b = 0;
    Vec3 point{};  // m
    Vec3 force{};  // on body_a, N
};

// End-loads of one lumped member (ENG-030, ADR 0010): three forces and three moments at each end.
struct MemberEndLoads {
    Wrench start{};
    Wrench end{};
};

// Rope tension after the step (PHY-004, REC-026, ruling 40).
struct RopeTension {
    double tension = 0.0;      // N, >= 0, the load-end segment
    double max_tension = 0.0;  // N, >= tension, the largest segment
    bool slack = false;        // the load-end segment carries no tension
};

// A mid-run model change or rope snap that took effect in the tick (PHY-008, ruling 40), neutral indices.
enum class ModelChangeKind : std::uint8_t {
    SpringOffset,        // source Spring, index spring, value offset (m or rad)
    SpringStiffness,     // source Spring, index spring, value stiffness (N/m or N m/rad)
    ConstraintDisabled,  // source Constraint, index constraint
    LoadReleased,        // source Constraint, Rope or Payload: index the constraint, rope or payload body; value the
                         // carrier body a payload left
    RopeSnapped,         // source Rope, index rope, value the tension that broke it (N)
    PayloadAttached,     // source Payload, index payload body, value carrier body (-1 the world)
    PayloadMass          // source Payload, index payload body, value mass (kg)
};

enum class ModelChangeSource : std::uint8_t { Spring, Constraint, Rope, Payload };

struct ModelChangeEvent {
    ModelChangeKind kind = ModelChangeKind::SpringOffset;
    ModelChangeSource source = ModelChangeSource::Spring;
    std::int64_t index = 0;
    double value = 0.0;
};

// Direction of power through a transmission at the step (ENG-044): Static at zero shaft speed, Forward when the shaft
// drives the load, Backward when the load drives the shaft, Locked for a self-locking screw holding.
enum class PowerFlow : std::uint8_t { Static, Forward, Backward, Locked };

// Transmission state after the step, one per neutral transmission in file order (PHY-006, ENG-048, PHY-033).
struct TransmissionState {
    double slip = 0.0;  // shaft travel lost to slip, shaft units (rad or m)
    bool slipping = false;
    double shaft_load_torque = 0.0;  // torque the shaft must supply along its + axis, N m (PHY-033)
    double force = 0.0;              // on the driven joint: N (belt, screw) or N m (gear)
    double stretch = 0.0;            // m or rad at the output; 0 for a rigid coupling
    PowerFlow power_flow = PowerFlow::Static;
};

// A payload body and its attachment after the step (PHY-006, ENG-035): carrier is kWorldBody once released; force is
// what the attachment applies to the payload, world frame, N (zero when released).
struct PayloadState {
    std::size_t body = 0;
    std::size_t carrier = kWorldBody;
    bool attached = false;
    Vec3 force{};
};

// Field for field with mechsim::engine::StepHealth (ENG-026); time and step are carried as the completed step count
// and the run's physics rate (time = steps / rate, step = 1 / rate) so the adapter needs no kernel base tick.
struct PhysicsHealth {
    double max_constraint_drift = 0.0;  // m or rad
    bool converged = true;
    int iterations = 0;
    std::int64_t steps = 0;  // completed steps
    Rate physics_rate;
};

// Torque or force a joint drive applied in the step, one per PhysicsInputs::joint_drives entry in its order (PHY-034):
// on the drive's joint along its + axis, N m or N (a reflected shaft: the torque on the shaft).
struct JointDriveTorque {
    std::size_t joint = 0;
    double torque = 0.0;
};

// Published every tick (PHY-003, PHY-007). The sensor models sample this; the controller never sees it (KER-020).
struct PhysicsOutputs {
    std::int64_t steps = 0;  // completed steps; time = steps / physics rate
    std::vector<BodyState> bodies;
    std::vector<JointForce> joints;
    std::vector<ConstraintForce> constraints;
    std::vector<ContactForce> contacts;
    std::vector<MemberEndLoads> members;
    std::vector<RopeTension> ropes;
    std::vector<TransmissionState> transmissions;
    // Per neutral joint: the coordinate of a revolute or prismatic joint relative to its neutral placement (rad or m;
    // a reflected shaft: q_driven / c), 0 for other joint types (PHY-034).
    std::vector<double> joint_positions;
    std::vector<JointDriveTorque> joint_drive_torques;
    std::vector<PayloadState> payloads;  // one per payload the adapter maps, in its order
    // The tick's model changes, each delivered in exactly one tick's outputs (ruling 40); recorded as events of kind
    // model_change (ruling 26).
    std::vector<ModelChangeEvent> model_changes;
};

struct BodyWrench {
    std::size_t body = 0;
    Wrench wrench{};  // world-aligned, at the body's centre of mass
};

// A spring-damper on one revolute or prismatic joint toward a rest coordinate that may move every step, applied by
// physics implicitly (KER-027, PHY-034): force -stiffness (q - rest) - damping qd. A disabled drive applies nothing.
struct JointDrive {
    std::size_t joint = 0;   // neutral joint index (the body's joint, file order)
    double rest = 0.0;       // rad or m, the joint coordinate of joint_positions
    double stiffness = 0.0;  // N m/rad or N/m, >= 0
    double damping = 0.0;    // N m s/rad or N s/m, >= 0
    bool enabled = false;
};

// Applied for the next step only, then cleared by the adapter (ENG-028, KER-010).
struct PhysicsInputs {
    std::vector<double> generalized_forces;  // one per velocity DOF, N or N m
    std::vector<BodyWrench> body_wrenches;
    std::vector<JointDrive> joint_drives;  // at most one per joint; empty: none (KER-027)
};

}  // namespace mechsim::kernel
