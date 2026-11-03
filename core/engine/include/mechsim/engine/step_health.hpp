#pragma once

namespace mechsim::engine {

struct StepHealth {
    double max_constraint_drift = 0.0;  // m or rad; 0 until the post-step projection exists
    bool converged = true;              // false never returns; non-convergence throws instead
    int iterations = 0;                 // constraint solver iterations; 0 until the solver exists
    double time = 0.0;                  // simulated time at the end of the step, s
    double step = 0.0;                  // step length used, s
};

}  // namespace mechsim::engine
