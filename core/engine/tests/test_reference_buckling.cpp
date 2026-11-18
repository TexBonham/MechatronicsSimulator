#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <numbers>
#include <vector>

#include "beam_helpers.hpp"
#include "mechsim/engine/dynamics_provider.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/multibody_provider.hpp"

using mechsim::engine::BeamEnds;
using mechsim::engine::BeamPoint;
using mechsim::engine::BeamSpec;
using mechsim::engine::DynamicsTerms;
using mechsim::engine::LumpedBeam;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::State;
using mechsim::engine::test::apply_point_force;
using mechsim::engine::test::flexural_rigidity;
using mechsim::engine::test::steel_bar;

// Euler buckling of the lumped pinned-pinned column (ENG-042).
//
// Discrete critical load, derived. Nodes j = 0..N-1 at x_j = (j + 1/2) l carry springs k = E I / l; links are the
// half pin segment [0, l/2], N - 1 full segments and the half end segment. With transverse node displacements y_j and
// ghost values y_-1 = -y_0, y_N = -y_{N-1} (the pinned ends), the link slopes are uniform differences and
//   spring energy  (k / 2) sum phi_j^2,  phi = C y,  C = A / l,
//   load work      (P / 2) sum s_i alpha_i^2 = (P / 2) y^T G y,  G = -A / l,
// with A the second-difference matrix whose corner entries are -3 (the ghosts). Both forms share A's eigenvectors
// y_j = sin(m pi x_j / L), eigenvalue mu_m = -4 sin^2(m pi / (2 N)), so k mu^2 / l^2 = P (-mu) / l gives
//   P_N = (2 N / L)^2 E I sin^2(pi / (2 N)) = P_Euler [sin(pi / 2N) / (pi / 2N)]^2,
// N = 8: P_N = 0.987213 P_Euler (-1.28 %), inside the brief's 2 % (spec ENG-042 tolerance not set, proposed 5 %).
//
// Engine check. With the end load P along world -x on the last segment's end point (the roller carries no force in
// the straight state, so no constraint curvature term enters), the tangent stiffness is K_t = -dQ/dq, Q the
// generalized force of evaluate() at v = 0 (springs plus the load; gravity off), by central differences
// (eps = 1e-6: truncation eps^2 ~ 1e-12, rounding 1e-16 |Q| / eps ~ 1e-9 of the entries). Reduced to the null space
// Z of the roller constraint's Jacobian, Z^T K_t Z = Z^T (K_0 - P G_e) Z is linear in P, so its smallest
// generalized eigenvalue is the engine's critical load, which must equal P_N to 1e-6; the smallest eigenvalue of
// Z^T K_t Z is positive at 0.98 P_N (stable) and negative at 1.02 P_N (diverges), no run to divergence.

namespace {

constexpr double difference_step = 1e-6;

Eigen::MatrixXd constraint_null_space(const MultibodyProvider& provider) {
    const Eigen::VectorXd q0 = provider.neutral_positions();
    const Eigen::Index nv = provider.velocity_size();
    Eigen::MatrixXd jacobian(3, nv);
    for (Eigen::Index j = 0; j < nv; ++j) {
        Eigen::VectorXd plus = q0;
        Eigen::VectorXd minus = q0;
        plus[j] += difference_step;
        minus[j] -= difference_step;
        jacobian.col(j) =
            (provider.constraint_errors(plus)[0].residual - provider.constraint_errors(minus)[0].residual) /
            (2.0 * difference_step);
    }
    Eigen::FullPivLU<Eigen::MatrixXd> lu(jacobian);
    lu.setThreshold(1e-6);
    return lu.kernel();
}

Eigen::VectorXd generalized_force(MultibodyProvider& provider, const BeamPoint& end, const Eigen::VectorXd& q,
                                  double load) {
    const Eigen::VectorXd v = Eigen::VectorXd::Zero(provider.velocity_size());
    const State state{q, v, 0.0};
    provider.clear_inputs();
    apply_point_force(provider, state, end, Eigen::Vector3d(-load, 0.0, 0.0));
    DynamicsTerms terms;
    provider.evaluate(0.0, q, v, terms);
    provider.clear_inputs();
    return terms.force - terms.bias;
}

Eigen::MatrixXd tangent_stiffness(MultibodyProvider& provider, const BeamPoint& end, double load) {
    const Eigen::VectorXd q0 = provider.neutral_positions();
    const Eigen::Index nv = provider.velocity_size();
    Eigen::MatrixXd out(nv, nv);
    for (Eigen::Index j = 0; j < nv; ++j) {
        Eigen::VectorXd plus = q0;
        Eigen::VectorXd minus = q0;
        plus[j] += difference_step;
        minus[j] -= difference_step;
        out.col(j) = -(generalized_force(provider, end, plus, load) - generalized_force(provider, end, minus, load)) /
                     (2.0 * difference_step);
    }
    return out;
}

double smallest_eigenvalue(const Eigen::MatrixXd& z, const Eigen::MatrixXd& stiffness) {
    const Eigen::MatrixXd reduced = z.transpose() * stiffness * z;
    const Eigen::MatrixXd symmetric = 0.5 * (reduced + reduced.transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
    return solver.eigenvalues()[0];
}

}  // namespace

TEST_CASE("lumped pinned-pinned column buckles at the discrete Euler load", "[ENG-042][reference]") {
    const int segments = 8;
    const BeamSpec spec = steel_bar(1.0, BeamEnds::SimplySupported, segments);
    MultibodyModel model;
    model.gravity = Eigen::Vector3d::Zero();
    const LumpedBeam beam = add_lumped_beam(model, spec);
    MultibodyProvider provider(model);
    const BeamPoint end = beam_point(beam, spec.length);

    const double euler = std::numbers::pi * std::numbers::pi * flexural_rigidity(spec) / (spec.length * spec.length);
    const double half_angle = std::numbers::pi / (2.0 * segments);
    const double discrete = std::pow(2.0 * segments / spec.length, 2) * flexural_rigidity(spec) *
                            std::pow(std::sin(half_angle), 2);
    std::printf("ENG-042: Euler %.10g N, lumped N = 8 closed form %.10g N (%.4f %%)\n", euler, discrete,
                100.0 * (discrete / euler - 1.0));
    REQUIRE(std::abs(discrete / euler - 1.0) <= 0.02);

    const Eigen::MatrixXd z = constraint_null_space(provider);
    REQUIRE(z.cols() == provider.velocity_size() - 2);
    const Eigen::MatrixXd unloaded = tangent_stiffness(provider, end, 0.0);
    const Eigen::MatrixXd loaded = tangent_stiffness(provider, end, discrete);
    const Eigen::MatrixXd geometric = (unloaded - loaded) / discrete;
    const Eigen::MatrixXd k0 = z.transpose() * unloaded * z;
    const Eigen::MatrixXd g0 = z.transpose() * geometric * z;
    const Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> solver(0.5 * (k0 + k0.transpose()),
                                                                           0.5 * (g0 + g0.transpose()));
    const double engine_critical = solver.eigenvalues()[0];
    std::printf("ENG-042: engine tangent-stiffness critical load %.10g N\n", engine_critical);
    REQUIRE(std::abs(engine_critical / discrete - 1.0) <= 1e-6);

    const double below = smallest_eigenvalue(z, tangent_stiffness(provider, end, 0.98 * discrete));
    const double above = smallest_eigenvalue(z, tangent_stiffness(provider, end, 1.02 * discrete));
    std::printf("ENG-042: smallest tangent eigenvalue %.6g at 0.98 P_N, %.6g at 1.02 P_N\n", below, above);
    REQUIRE(below > 0.0);
    REQUIRE(above < 0.0);
}
