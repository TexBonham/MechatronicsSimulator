#pragma once

namespace mechsim::engine {

// The only step length literal in the engine (ENG-002); every other step comes from a run configuration.
struct StepConfig {
    static constexpr double default_step_s = 1e-3;

    double h = default_step_s;  // step length, s; finite and > 0
};

}  // namespace mechsim::engine
