#pragma once

#include <Eigen/Core>

namespace mechsim::engine {

// Units per coordinate follow the coordinate: m and m/s for prismatic, rad and rad/s for revolute.
struct State {
    Eigen::VectorXd q;  // generalized positions, size nq
    Eigen::VectorXd v;  // generalized velocities, size nv
    double t = 0.0;     // simulated time, s
};

}  // namespace mechsim::engine
