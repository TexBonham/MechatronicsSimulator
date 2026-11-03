#pragma once

#include <Eigen/Core>
#include <limits>
#include <utility>

#include "mechsim/engine/dynamics_provider.hpp"

namespace mechsim::engine::test {

// Linear system: H dv/dt = f_applied - K q - D v - gravity, with constant H, K, D.
class LinearProvider : public DynamicsProvider {
public:
    LinearProvider(Eigen::MatrixXd mass, Eigen::MatrixXd stiffness, Eigen::MatrixXd damping, Eigen::VectorXd applied,
                   Eigen::VectorXd gravity_bias)
        : mass_(std::move(mass)),
          stiffness_(std::move(stiffness)),
          damping_(std::move(damping)),
          applied_(std::move(applied)),
          gravity_bias_(std::move(gravity_bias)) {}

    [[nodiscard]] Eigen::Index position_size() const override { return mass_.rows(); }
    [[nodiscard]] Eigen::Index velocity_size() const override { return mass_.rows(); }

    void evaluate(double t, const Eigen::VectorXd& q, const Eigen::VectorXd& v, DynamicsTerms& terms) const override {
        terms.mass = mass_;
        terms.stiffness = stiffness_;
        terms.damping = damping_;
        terms.bias = gravity_bias_;
        terms.force = applied_ - stiffness_ * q - damping_ * v;
        if (t >= nan_from_time_) {
            terms.force.setConstant(std::numeric_limits<double>::quiet_NaN());
        }
    }

    [[nodiscard]] Eigen::VectorXd integrate_positions(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                      double h) const override {
        return q + h * v;
    }

    void set_nan_from_time(double t) { nan_from_time_ = t; }

private:
    Eigen::MatrixXd mass_;
    Eigen::MatrixXd stiffness_;
    Eigen::MatrixXd damping_;
    Eigen::VectorXd applied_;
    Eigen::VectorXd gravity_bias_;
    double nan_from_time_ = std::numeric_limits<double>::infinity();
};

inline LinearProvider one_dof(double m, double k, double d, double applied, double gravity_bias) {
    return {Eigen::MatrixXd::Constant(1, 1, m), Eigen::MatrixXd::Constant(1, 1, k), Eigen::MatrixXd::Constant(1, 1, d),
            Eigen::VectorXd::Constant(1, applied), Eigen::VectorXd::Constant(1, gravity_bias)};
}

inline LinearProvider two_dof() {
    Eigen::MatrixXd mass(2, 2);
    mass << 3.0, 1.0, 1.0, 2.0;
    Eigen::MatrixXd stiffness(2, 2);
    stiffness << 300.0, -100.0, -100.0, 200.0;
    Eigen::MatrixXd damping(2, 2);
    damping << 20.0, -5.0, -5.0, 15.0;
    Eigen::VectorXd applied(2);
    applied << 4.0, -2.0;
    return {mass, stiffness, damping, applied, Eigen::VectorXd::Zero(2)};
}

}  // namespace mechsim::engine::test
