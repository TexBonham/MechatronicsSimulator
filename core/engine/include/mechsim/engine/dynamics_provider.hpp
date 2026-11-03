#pragma once

#include <Eigen/Core>

namespace mechsim::engine {

// Equation of motion: mass * dv/dt + bias = force, all in generalized coordinates.
struct DynamicsTerms {
    Eigen::MatrixXd mass;       // H(q), nv x nv, symmetric positive definite; kg or kg m^2
    Eigen::VectorXd bias;       // c(q, v): Coriolis, gravity, passive; N or N m
    Eigen::VectorXd force;      // f: applied, spring and damper forces; N or N m
    Eigen::MatrixXd stiffness;  // K = -df/dq, nv x nv; N/m or N m/rad
    Eigen::MatrixXd damping;    // D = -df/dv, nv x nv; N s/m or N m s/rad
};

class DynamicsProvider {
public:
    virtual ~DynamicsProvider() = default;

    [[nodiscard]] virtual Eigen::Index position_size() const = 0;
    [[nodiscard]] virtual Eigen::Index velocity_size() const = 0;

    virtual void evaluate(double t, const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                          DynamicsTerms& terms) const = 0;

    // Returns q advanced by velocity v over h; non-Euclidean coordinates (quaternions) map here.
    [[nodiscard]] virtual Eigen::VectorXd integrate_positions(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                              double h) const = 0;

    // Post-step projection onto the constraint manifold (ADR 0013); returns max drift corrected, m or rad.
    virtual double project(Eigen::VectorXd& /*q*/, Eigen::VectorXd& /*v*/) const { return 0.0; }

protected:
    DynamicsProvider() = default;
    DynamicsProvider(const DynamicsProvider&) = default;
    DynamicsProvider(DynamicsProvider&&) = default;
    DynamicsProvider& operator=(const DynamicsProvider&) = default;
    DynamicsProvider& operator=(DynamicsProvider&&) = default;
};

}  // namespace mechsim::engine
