#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <cmath>
#include <cstddef>
#include <vector>

#include "mechsim/engine/dynamics_provider.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/state.hpp"

namespace mechsim::engine::test {

// Steel bar, 10 mm square: E I = 166.7 N m^2, rho A = 0.785 kg/m.
inline BeamSpec steel_bar(double length, BeamEnds ends, int segments) {
    BeamSpec spec;
    spec.youngs_modulus = 2.0e11;
    spec.area_moment = 0.01 * 0.01 * 0.01 * 0.01 / 12.0;
    spec.density = 7850.0;
    spec.area = 0.01 * 0.01;
    spec.length = length;
    spec.segments = segments;
    spec.ends = ends;
    return spec;
}

inline double flexural_rigidity(const BeamSpec& spec) {
    return spec.youngs_modulus * spec.area_moment;
}

// sqrt(E I / (rho A L^4)), rad/s; the first-mode closed forms are (beta_1 L)^2 times it.
inline double frequency_scale(const BeamSpec& spec) {
    const double l2 = spec.length * spec.length;
    return std::sqrt(flexural_rigidity(spec) / (spec.density * spec.area * l2 * l2));
}

inline Eigen::Vector3d world_point(const MultibodyProvider& provider, const State& state, const BeamPoint& point) {
    if (point.body < 0) {
        return point.point;
    }
    const BodyOutput body = provider.body_states(state.q, state.v)[static_cast<std::size_t>(point.body)];
    return body.position + body.orientation * point.point;
}

// World-aligned force at a material point, applied for the next step as a wrench at the body's center of mass.
inline void apply_point_force(MultibodyProvider& provider, const State& state, const BeamPoint& point,
                              const Eigen::Vector3d& force) {
    const BodyOutput body = provider.body_states(state.q, state.v)[static_cast<std::size_t>(point.body)];
    const Body& model_body = provider.model().bodies[static_cast<std::size_t>(point.body)];
    const Eigen::Vector3d com = body.position + body.orientation * model_body.com;
    const Eigen::Vector3d at = body.position + body.orientation * point.point;
    provider.apply_body_wrench(point.body, (at - com).cross(force), force);
}

// Lowest eigenfrequency, rad/s, of the linearized model at its neutral positions: K phi = omega^2 H phi on the
// null space of the loop-constraint Jacobian (central differences of the residuals, which are smooth in q).
inline double lumped_first_frequency(const MultibodyProvider& provider) {
    const Eigen::VectorXd q0 = provider.neutral_positions();
    const Eigen::Index nv = provider.velocity_size();
    DynamicsTerms terms;
    provider.evaluate(0.0, q0, Eigen::VectorXd::Zero(nv), terms);
    Eigen::MatrixXd basis = Eigen::MatrixXd::Identity(nv, nv);
    if (!provider.model().constraints.empty()) {
        const double eps = 1e-7;
        std::vector<Eigen::VectorXd> columns;
        Eigen::Index rows = 0;
        for (Eigen::Index j = 0; j < nv; ++j) {
            Eigen::VectorXd plus = q0;
            Eigen::VectorXd minus = q0;
            plus[j] += eps;
            minus[j] -= eps;
            const auto up = provider.constraint_errors(plus);
            const auto down = provider.constraint_errors(minus);
            Eigen::VectorXd column;
            for (std::size_t c = 0; c < up.size(); ++c) {
                const Eigen::VectorXd slope = (up[c].residual - down[c].residual) / (2.0 * eps);
                Eigen::VectorXd grown(column.size() + slope.size());
                grown << column, slope;
                column = grown;
            }
            rows = column.size();
            columns.push_back(column);
        }
        Eigen::MatrixXd jacobian(rows, nv);
        for (Eigen::Index j = 0; j < nv; ++j) {
            jacobian.col(j) = columns[static_cast<std::size_t>(j)];
        }
        Eigen::FullPivLU<Eigen::MatrixXd> lu(jacobian);
        lu.setThreshold(1e-6);
        basis = lu.kernel();
    }
    const Eigen::MatrixXd stiffness = basis.transpose() * terms.stiffness * basis;
    const Eigen::MatrixXd mass = basis.transpose() * terms.mass * basis;
    const Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> solver(stiffness, mass);
    return std::sqrt(solver.eigenvalues()[0]);
}

// Times of upward zero crossings of the world y of a point, linear interpolation between steps.
inline std::vector<double> upward_crossings(MultibodyProvider& provider, Integrator& integrator, State& state,
                                            const BeamPoint& point, int steps) {
    std::vector<double> out;
    double before = world_point(provider, state, point).y();
    for (int n = 0; n < steps; ++n) {
        const double t0 = state.t;
        provider.step(integrator, state);
        const double after = world_point(provider, state, point).y();
        if (before < 0.0 && after >= 0.0) {
            out.push_back(t0 + integrator.config().h * before / (before - after));
        }
        before = after;
    }
    return out;
}

}  // namespace mechsim::engine::test
