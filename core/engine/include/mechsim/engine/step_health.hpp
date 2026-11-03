#pragma once

namespace mechsim::engine {

struct StepHealth {
    double max_constraint_drift = 0.0;  // m or rad; max constraint error after projection, 0 without constraints
    bool converged = true;              // false never returns; non-convergence throws instead
    int iterations = 0;                 // constraint solver sweeps this step, 0 without constraints
    double time = 0.0;                  // simulated time at the end of the step, s
    double step = 0.0;                  // step length used, s
};

}  // namespace mechsim::engine
