#pragma once

#include <cstdint>
#include <map>
#include <string>

#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

enum class ControllerMode : std::uint8_t {
    native,
    emulated,
};

// ADR 0013 solver values; defaults are the spec values (docs/spec/eng.md ENG-024, ENG-025), mirrored from
// mechsim::engine::MultibodyConfig so the kernel does not depend on the engine.
struct SolverConfig {
    static constexpr int default_iteration_cap = 50;
    static constexpr double default_velocity_tolerance = 1e-9;
    static constexpr int default_projection_iterations = 3;

    int iteration_cap = default_iteration_cap;                  // projected Gauss-Seidel sweeps, >= 1
    double velocity_tolerance = default_velocity_tolerance;     // m/s or rad/s, constraint velocity residual
    int projection_iterations = default_projection_iterations;  // post-step position projection, >= 0
};

// Drift past either threshold ends the run (ENG-024, PHY-009).
struct DriftThresholds {
    static constexpr double default_translational_m = 1e-6;
    static constexpr double default_rotational_rad = 1e-6;

    double translational_m = default_translational_m;
    double rotational_rad = default_rotational_rad;
};

// Run configuration (KER-004, KER-005, KER-012). Every rate is data here, never a code constant.
struct RunConfig {
    static constexpr std::int64_t default_physics_rate_hz = 1'000;
    static constexpr std::int64_t default_control_rate_hz = 1'000;

    Rate physics_rate{default_physics_rate_hz};            // physics step = 1 / physics_rate (KER-004)
    Rate control_rate{default_control_rate_hz};            // KER-005; may differ from the physics rate (KER-006)
    std::map<std::string, Rate, std::less<>> party_rates;  // overrides Party::nominal_rate() by party name
    // Written by the kernel at run start: lcm of physics_rate and every party rate; the recorder stores it as an
    // archive attribute. Any value set before run() is overwritten.
    Rate base_rate{default_physics_rate_hz};
    std::uint64_t seed = 0;                                      // KER-012
    ControllerMode controller_mode = ControllerMode::native;
    SolverConfig solver;
    DriftThresholds drift;
};

}  // namespace mechsim::kernel
