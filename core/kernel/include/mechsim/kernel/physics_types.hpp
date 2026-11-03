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

// Rope tension after the step (PHY-004, REC-026).
struct RopeTension {
    double tension = 0.0;  // N, >= 0
    bool slack = false;
};

// Transmission slip state after the step (PHY-006): accumulated slip between input and output side.
struct TransmissionState {
    double slip = 0.0;  // m or rad, output side
    bool slipping = false;
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
};

struct BodyWrench {
    std::size_t body = 0;
    Wrench wrench{};  // world-aligned, at the body's centre of mass
};

// Applied for the next step only, then cleared by the adapter (ENG-028, KER-010).
struct PhysicsInputs {
    std::vector<double> generalized_forces;  // one per velocity DOF, N or N m
    std::vector<BodyWrench> body_wrenches;
};

}  // namespace mechsim::kernel
