#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <cmath>
#include <string>
#include <utility>

#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"

namespace mechsim::engine {

namespace {

void require_square(const Eigen::MatrixXd& m, Eigen::Index n, double t, const std::string& name) {
    if (m.rows() != n || m.cols() != n) {
        throw InvalidModelError(t, name + " is not nv x nv");
    }
}

void require_size(const Eigen::VectorXd& x, Eigen::Index n, double t, const std::string& name) {
    if (x.size() != n) {
        throw InvalidModelError(t, name + " has the wrong size");
    }
}

void require_finite(const Eigen::VectorXd& x, double t, const std::string& name) {
    if (!x.allFinite()) {
        throw NonFiniteStateError(t, name + " is not finite");
    }
}

}  // namespace

SemiImplicitEuler::SemiImplicitEuler(StepConfig config) : config_(config) {
    if (!std::isfinite(config_.h) || config_.h <= 0.0) {
        throw InvalidConfigError(0.0, "step length h must be finite and positive");
    }
}

const StepConfig& SemiImplicitEuler::config() const noexcept { return config_; }

StepHealth SemiImplicitEuler::step(const DynamicsProvider& provider, State& state) {
    const double t = state.t;
    const double h = config_.h;
    const Eigen::Index nq = provider.position_size();
    const Eigen::Index nv = provider.velocity_size();
    require_size(state.q, nq, t, "q");
    require_size(state.v, nv, t, "v");

    provider.evaluate(t, state.q, state.v, terms_);
    require_square(terms_.mass, nv, t, "mass matrix");
    require_square(terms_.stiffness, nv, t, "stiffness Jacobian");
    require_square(terms_.damping, nv, t, "damping Jacobian");
    require_size(terms_.bias, nv, t, "bias force");
    require_size(terms_.force, nv, t, "applied force");

    Eigen::VectorXd v_new = state.v;
    if (nv > 0) {
        if (terms_.mass != terms_.mass.transpose()) {
            throw InvalidModelError(t, "mass matrix is not symmetric");
        }
        const Eigen::LLT<Eigen::MatrixXd> mass_check(terms_.mass);
        if (mass_check.info() != Eigen::Success) {
            throw InvalidModelError(t, "mass matrix is not positive definite");
        }
        const Eigen::MatrixXd system = terms_.mass + h * terms_.damping + (h * h) * terms_.stiffness;
        const Eigen::VectorXd rhs = h * (terms_.force - terms_.bias) - (h * h) * (terms_.stiffness * state.v);
        const Eigen::VectorXd dv = Eigen::PartialPivLU<Eigen::MatrixXd>(system).solve(rhs);
        require_finite(dv, t, "velocity increment dv");
        v_new += dv;
    }

    Eigen::VectorXd q_new = provider.integrate_positions(state.q, v_new, h);
    require_size(q_new, nq, t, "integrated q");
    const double drift = provider.project(q_new, v_new);
    if (!std::isfinite(drift)) {
        throw NonFiniteStateError(t, "constraint drift is not finite");
    }
    require_finite(q_new, t, "q");
    require_finite(v_new, t, "v");

    state.q = std::move(q_new);
    state.v = std::move(v_new);
    state.t = t + h;

    StepHealth health;
    health.max_constraint_drift = drift;
    health.time = state.t;
    health.step = h;
    return health;
}

}  // namespace mechsim::engine
