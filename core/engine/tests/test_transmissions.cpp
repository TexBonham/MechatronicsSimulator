#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <array>
#include <cmath>
#include <complex>
#include <cstdio>
#include <limits>
#include <numbers>
#include <vector>

#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/transmission.hpp"

using mechsim::engine::Body;
using mechsim::engine::DynamicsTerms;
using mechsim::engine::InvalidModelError;
using mechsim::engine::JointType;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::PowerFlow;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::Transmission;
using mechsim::engine::TransmissionOutput;
using mechsim::engine::TransmissionType;

// Every model here is a shaft (body 0, Revolute about z, inertia J, q[0] = theta) and an output (body 1, q[1] = x),
// both on the world, so H = diag(J, m) and the transmission is the only coupling (transmission.hpp holds the model).
//
// Settling bound used throughout. With the gain g frozen, the scheme is backward Euler on a linear system:
// S v' = M v - h K x + h f0, x' = x + h v', S = M + h D + h^2 K, so the deviation from the fixed point evolves by
// A = [[I - h^2 S^-1 K, h S^-1 M], [-h S^-1 K, S^-1 M]]. settling_steps builds A from the parameters (not from
// the engine), takes its spectral radius rho over the modes that decay, and returns N with rho^N <= 1e-20. Modes
// with eigenvalue 1 are the rigid modes a test shows unexcited by a conservation argument. The 1e-20 against the
// asserted 1e-9 leaves 1e11 for the eigenvector condition number and the initial amplitude.
//
// Tolerances (docs/spec/eng.md, Open parameters, transmission unit-test tolerance):
//   1e-9 relative   baseline: settled static and steady-state values, energy ratios, discrete closed forms
//   1e-12 relative  consistency: published values against the hand formulas; slip offset of one step
//   1e-11 relative  slip offset summed over the settling run (rounding of the sum)
//   1e-6 relative   backlash probe force past the edge (rounding of E against delta = 1e-6 b/2)
//   transient 1e-20 by the spectral radius, at most 20 000 settling steps

namespace {

constexpr double gravity = 9.81;
constexpr double transient = 1e-20;
constexpr double rel = 1e-9;

Body shaft(double inertia) {
    Body body;
    body.parent = -1;
    body.joint.type = JointType::Revolute;
    body.joint.axis = Eigen::Vector3d::UnitZ();
    body.mass = 1.0;
    body.inertia = Eigen::Vector3d(inertia, inertia, inertia).asDiagonal();
    return body;
}

Body carriage(double mass, const Eigen::Vector3d& axis) {
    Body body;
    body.parent = -1;
    body.joint.type = JointType::Prismatic;
    body.joint.axis = axis;
    body.mass = mass;
    return body;
}

MultibodyModel pair_model(const Body& output, const Transmission& t, const Eigen::Vector3d& g) {
    MultibodyModel model;
    model.gravity = g;
    model.bodies.push_back(shaft(1e-4));
    model.bodies.push_back(output);
    model.transmissions.push_back(t);
    return model;
}

Eigen::Matrix2d coupling_matrix(double gain, double n_eff) {
    const Eigen::Vector2d a(gain, -1.0);
    const Eigen::Vector2d b(n_eff, -1.0);
    return a * b.transpose();
}

long settling_steps(const Eigen::Matrix2d& m, const Eigen::Matrix2d& k, const Eigen::Matrix2d& d, double h) {
    const Eigen::Matrix2d s = m + h * d + h * h * k;
    const Eigen::Matrix2d si = s.inverse();
    Eigen::Matrix4d a;
    a.topLeftCorner<2, 2>() = Eigen::Matrix2d::Identity() - h * h * si * k;
    a.topRightCorner<2, 2>() = h * si * m;
    a.bottomLeftCorner<2, 2>() = -h * si * k;
    a.bottomRightCorner<2, 2>() = si * m;
    const Eigen::EigenSolver<Eigen::Matrix4d> solver(a);
    double rho = 0.0;
    for (Eigen::Index i = 0; i < 4; ++i) {
        const std::complex<double> lambda = solver.eigenvalues()[i];
        if (std::abs(lambda - 1.0) < 1e-6) {
            continue;
        }
        rho = std::max(rho, std::abs(lambda));
    }
    REQUIRE(rho < 1.0);
    const long n = static_cast<long>(std::ceil(std::log(transient) / std::log(rho)));
    REQUIRE(n < 20000);
    return n;
}

State rest_state(const MultibodyProvider& provider) {
    State state;
    state.q = provider.neutral_positions();
    state.v = Eigen::VectorXd::Zero(provider.velocity_size());
    return state;
}

bool close(double measured, double target, double tolerance) {
    std::printf("  target %.15g measured %.15g\n", target, measured);
    return std::abs(measured - target) <= tolerance * std::abs(target);
}

}  // namespace

TEST_CASE("belt-driven carriage holds a hanging mass at force = torque / r", "[ENG-043][ENG-048]") {
    // Vertical carriage m on a belt of radius r, shaft torque tau = m g r. Momentum-like p = J omega + r m xdot
    // changes per step by h (tau - r m g) = 0 (the coupling enters with [1, r] . a = g - r = 0 at eta = 1), so the
    // rigid mode stays at rest and the coupling mode decays: F -> m g = tau / r, stretch -> m g / k, shaft reaction
    // -> -tau. Coupling damping c is critical for the reduced mass mu = 1 / (1/m + r^2 / J).
    const double r = 0.02;
    const double m = 2.0;
    const double j = 1e-4;
    const double k = 1e5;
    const double mu = 1.0 / (1.0 / m + r * r / j);
    Transmission t;
    t.ratio = r;
    t.stiffness = k;
    t.damping = 2.0 * std::sqrt(k * mu);
    MultibodyModel model = pair_model(carriage(m, Eigen::Vector3d::UnitZ()), t, Eigen::Vector3d(0, 0, -gravity));
    model.bodies[0] = shaft(j);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    State state = rest_state(provider);
    const double tau = m * gravity * r;
    const Eigen::Matrix2d mass = Eigen::Vector2d(j, m).asDiagonal();
    const Eigen::Matrix2d ab = coupling_matrix(r, r);
    const long n = settling_steps(mass, k * ab, t.damping * ab, integrator.config().h);
    for (long i = 0; i < n; ++i) {
        provider.set_shaft_torque(0, tau);
        provider.step(integrator, state);
    }
    const TransmissionOutput& out = provider.outputs().transmissions.at(0);
    REQUIRE(close(out.force, tau / r, rel));
    REQUIRE(close(out.shaft_load_torque, tau, rel));
    REQUIRE(close(out.stretch, m * gravity / k, rel));
    REQUIRE(out.in_contact);
    REQUIRE_FALSE(out.slip_active);
}

TEST_CASE("published shaft load torque and joint forces equal the hand-computed reaction", "[ENG-048][ENG-043]") {
    // Hand formulas (transmission.hpp): E = n theta - x - s, d(E) dead band, F = k d + c (n_eff omega - xdot),
    // P = -n F omega, g = n / eta_f (P < 0), n eta_b (P > 0), n (P = 0); load torque -g F. f gets drive - g F on the
    // shaft and F on the output. Only rounding separates the two evaluations: 1e-12 relative.
    Transmission t;
    t.ratio = 0.015;
    t.efficiency = 0.8;
    t.back_efficiency = 0.5;
    t.stiffness = 2e4;
    t.damping = 30.0;
    t.backlash = 1e-4;
    t.slip = 0.1;
    MultibodyProvider provider(pair_model(carriage(0.5, Eigen::Vector3d::UnitX()), t, Eigen::Vector3d::Zero()));
    SemiImplicitEuler integrator{StepConfig{}};
    State state = rest_state(provider);
    const double n = t.ratio;
    std::vector<PowerFlow> seen;
    for (int i = 0; i < 200; ++i) {
        const double drive = 0.02 * std::cos(0.05 * i);
        provider.set_shaft_torque(0, drive);
        provider.step(integrator, state);
        const TransmissionOutput& out = provider.outputs().transmissions.at(0);
        const double s = out.slip_offset;
        const double e = n * state.q[0] - state.q[1] - s;
        double d = 0.0;
        if (e > 0.5 * t.backlash) {
            d = e - 0.5 * t.backlash;
        }
        if (e < -0.5 * t.backlash) {
            d = e + 0.5 * t.backlash;
        }
        double f = 0.0;
        if (d != 0.0) {
            f = t.stiffness * d + t.damping * ((1.0 - t.slip) * n * state.v[0] - state.v[1]);
        }
        const double p = -n * f * state.v[0];
        double g = n;
        if (p < 0.0) {
            g = n / t.efficiency;
        }
        if (p > 0.0) {
            g = n * t.back_efficiency;
        }
        const double scale = std::max(1.0, std::abs(f));
        REQUIRE(std::abs(out.force - f) <= 1e-12 * scale);
        REQUIRE(std::abs(out.shaft_load_torque - g * f) <= 1e-12 * scale);
        seen.push_back(out.power_flow);

        provider.set_shaft_torque(0, drive);
        DynamicsTerms terms;
        provider.evaluate(state.t, state.q, state.v, terms);
        REQUIRE(std::abs(terms.force[1] - f) <= 1e-12 * scale);
        REQUIRE(std::abs(terms.force[0] - (drive - g * f)) <= 1e-12 * scale);
        provider.clear_inputs();
    }
    REQUIRE(std::find(seen.begin(), seen.end(), PowerFlow::Forward) != seen.end());
    REQUIRE(std::find(seen.begin(), seen.end(), PowerFlow::Backward) != seen.end());
}

namespace {

struct EnergyCase {
    TransmissionType type = TransmissionType::BeltPulley;
    double ratio = 0.0;
    double eta_forward = 1.0;
    double eta_backward = 1.0;
    double damping = 0.0;
    double c_shaft = 0.0;
};

// Forward: motor torque tau on the shaft, viscous load c_L on the carriage. At the fixed point xdot = n omega,
// F = c_L xdot, tau = (n / eta_f) F, so omega = eta_f tau / (c_L n^2) and load power c_L xdot^2 = eta_f tau omega.
// Backward: force F_ext on the carriage, viscous c_s on the shaft: c_s omega = n eta_b F_ext, and shaft power
// c_s omega^2 = eta_b F_ext xdot. The energy over a window after settling has the same ratio.
void check_energy(const EnergyCase& e) {
    const double j = 1e-5;
    const double m = 1.0;
    const double k = 1e6;
    const double c_load = 50.0;
    const double c_shaft = e.c_shaft;
    const double tau = 0.01;
    const double push = 5.0;
    const int window = 1000;
    Transmission t;
    t.type = e.type;
    t.ratio = e.ratio;
    t.efficiency = e.eta_forward;
    t.back_efficiency = e.eta_backward;
    t.stiffness = k;
    t.damping = e.damping;
    const double n = mechsim::engine::output_per_input(t);
    const Eigen::Matrix2d mass = Eigen::Vector2d(j, m).asDiagonal();
    SemiImplicitEuler integrator{StepConfig{}};
    const double h = integrator.config().h;
    {
        Body out = carriage(m, Eigen::Vector3d::UnitX());
        out.joint.damping = Eigen::VectorXd::Constant(1, c_load);
        MultibodyModel model = pair_model(out, t, Eigen::Vector3d::Zero());
        model.bodies[0] = shaft(j);
        MultibodyProvider provider(model);
        const Eigen::Matrix2d ab = coupling_matrix(n / e.eta_forward, n);
        const Eigen::Matrix2d d = e.damping * ab + Eigen::Vector2d(0.0, c_load).asDiagonal().toDenseMatrix();
        const long settle = settling_steps(mass, k * ab, d, h);
        State state = rest_state(provider);
        double in = 0.0;
        double load = 0.0;
        for (long i = 0; i < settle + window; ++i) {
            provider.set_shaft_torque(0, tau);
            provider.step(integrator, state);
            if (i >= settle) {
                REQUIRE(provider.outputs().transmissions.at(0).power_flow == PowerFlow::Forward);
                in += tau * state.v[0] * h;
                load += c_load * state.v[1] * state.v[1] * h;
            }
        }
        REQUIRE(close(state.v[0], e.eta_forward * tau / (c_load * n * n), rel));
        REQUIRE(close(load / in, e.eta_forward, rel));
    }
    {
        Body in_shaft = shaft(j);
        in_shaft.joint.damping = Eigen::VectorXd::Constant(1, c_shaft);
        MultibodyModel model = pair_model(carriage(m, Eigen::Vector3d::UnitX()), t, Eigen::Vector3d::Zero());
        model.bodies[0] = in_shaft;
        MultibodyProvider provider(model);
        const Eigen::Matrix2d ab = coupling_matrix(n * e.eta_backward, n);
        const Eigen::Matrix2d d = e.damping * ab + Eigen::Vector2d(c_shaft, 0.0).asDiagonal().toDenseMatrix();
        const long settle = settling_steps(mass, k * ab, d, h);
        State state = rest_state(provider);
        double in = 0.0;
        double load = 0.0;
        for (long i = 0; i < settle + window; ++i) {
            provider.set_generalized_forces(Eigen::Vector2d(0.0, push));
            provider.step(integrator, state);
            if (i >= settle) {
                REQUIRE(provider.outputs().transmissions.at(0).power_flow == PowerFlow::Backward);
                in += push * state.v[1] * h;
                load += c_shaft * state.v[0] * state.v[0] * h;
            }
        }
        REQUIRE(close(state.v[0], n * e.eta_backward * push / c_shaft, rel));
        REQUIRE(close(load / in, e.eta_backward, rel));
    }
}

}  // namespace

TEST_CASE("belt efficiency scales the delivered energy in both power-flow directions", "[ENG-043]") {
    check_energy({TransmissionType::BeltPulley, 0.01, 0.9, 0.9, 600.0, 1e-3});
}

TEST_CASE("lead screw forward and backward efficiencies scale the delivered energy", "[ENG-044]") {
    check_energy({TransmissionType::LeadScrew, 0.01, 0.8, 0.6, 2000.0, 1e-4});
}

TEST_CASE("gear pair maps speed by the ratio and reverses direction", "[ENG-045]") {
    // Input J_in at omega0, output J_out at rest, n = -1/N. J_in omega + n J_out omega_out is conserved (the
    // coupling enters as [1, n] . a = 0 at eta = 1), so after the coupling mode decays omega_out = n omega_f with
    // omega_f = J_in omega0 / (J_in + n^2 J_out).
    const double j_in = 1e-3;
    const double j_out = 2e-3;
    const double ratio = 3.0;
    const double omega0 = 10.0;
    Transmission t;
    t.type = TransmissionType::GearPair;
    t.ratio = ratio;
    t.reverses = true;
    t.stiffness = 50.0;
    t.damping = 0.6;
    MultibodyModel model = pair_model(shaft(j_out), t, Eigen::Vector3d::Zero());
    model.bodies[0] = shaft(j_in);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    const double n = -1.0 / ratio;
    const Eigen::Matrix2d ab = coupling_matrix(n, n);
    const long settle =
        settling_steps(Eigen::Vector2d(j_in, j_out).asDiagonal(), t.stiffness * ab, t.damping * ab, integrator.config().h);
    State state = rest_state(provider);
    state.v[0] = omega0;
    for (long i = 0; i < settle; ++i) {
        provider.step(integrator, state);
    }
    const double omega_f = j_in * omega0 / (j_in + n * n * j_out);
    REQUIRE(close(state.v[0], omega_f, rel));
    REQUIRE(close(state.v[1], -omega_f / ratio, rel));
}

TEST_CASE("gear pair multiplies the shaft torque by the ratio at the output", "[ENG-045][ENG-048]") {
    // Output on a joint spring K_o. Equilibrium: shaft tau - n F = 0, so F = tau / n = -N tau, theta_out = F / K_o,
    // load torque -tau. No rigid mode is free (K_o restrains it), so every mode decays.
    const double ratio = 3.0;
    const double k_out = 5.0;
    const double d_out = 0.5;
    const double tau = 0.01;
    Body out = shaft(2e-3);
    out.joint.stiffness = Eigen::VectorXd::Constant(1, k_out);
    out.joint.damping = Eigen::VectorXd::Constant(1, d_out);
    Transmission t;
    t.type = TransmissionType::GearPair;
    t.ratio = ratio;
    t.reverses = true;
    t.stiffness = 50.0;
    t.damping = 0.6;
    MultibodyModel model = pair_model(out, t, Eigen::Vector3d::Zero());
    model.bodies[0] = shaft(1e-3);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    const double n = -1.0 / ratio;
    const Eigen::Matrix2d ab = coupling_matrix(n, n);
    const long settle = settling_steps(Eigen::Vector2d(1e-3, 2e-3).asDiagonal(),
                                       t.stiffness * ab + Eigen::Vector2d(0.0, k_out).asDiagonal().toDenseMatrix(),
                                       t.damping * ab + Eigen::Vector2d(0.0, d_out).asDiagonal().toDenseMatrix(),
                                       integrator.config().h);
    State state = rest_state(provider);
    for (long i = 0; i < settle; ++i) {
        provider.set_shaft_torque(0, tau);
        provider.step(integrator, state);
    }
    const TransmissionOutput& o = provider.outputs().transmissions.at(0);
    REQUIRE(close(o.force, -ratio * tau, rel));
    REQUIRE(close(o.shaft_load_torque, tau, rel));
    REQUIRE(close(state.q[1], -ratio * tau / k_out, rel));
}

TEST_CASE("slip fraction is reproduced on a constant-speed drive", "[ENG-043][ENG-046]") {
    // Shaft at omega0 drives a free carriage through a belt slipping sigma. J omega + n m xdot is conserved (the
    // reaction keeps the full radius), and the coupling settles where dE/dt = n_eff omega - xdot = 0, so
    // xdot = (1 - sigma) n omega_f, omega_f = J omega0 / (J + n^2 m (1 - sigma)). The slip offset is
    // sigma n (theta_end - theta_0), summed per step: 1e-11 relative covers the rounding of the sum.
    const double r = 0.01;
    const double j = 1e-4;
    const double m = 0.5;
    const double sigma = 0.25;
    const double omega0 = 20.0;
    Transmission t;
    t.ratio = r;
    t.stiffness = 1e4;
    t.damping = 120.0;
    t.slip = sigma;
    MultibodyModel model = pair_model(carriage(m, Eigen::Vector3d::UnitX()), t, Eigen::Vector3d::Zero());
    model.bodies[0] = shaft(j);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    const Eigen::Matrix2d ab = coupling_matrix(r, (1.0 - sigma) * r);
    const long settle =
        settling_steps(Eigen::Vector2d(j, m).asDiagonal(), t.stiffness * ab, t.damping * ab, integrator.config().h);
    State state = rest_state(provider);
    state.v[0] = omega0;
    for (long i = 0; i < settle; ++i) {
        provider.step(integrator, state);
    }
    const double omega_f = j * omega0 / (j + r * r * m * (1.0 - sigma));
    REQUIRE(close(state.v[0], omega_f, rel));
    REQUIRE(close(state.v[1], (1.0 - sigma) * r * omega_f, rel));
    const TransmissionOutput& o = provider.outputs().transmissions.at(0);
    REQUIRE(o.slip_active);
    REQUIRE(close(o.slip_offset, sigma * r * state.q[0], 1e-11));
}

TEST_CASE("slip is zero by default and a mid-run change takes effect at the next step", "[ENG-046]") {
    Transmission t;
    t.ratio = 0.01;
    t.stiffness = 1e4;
    t.damping = 50.0;
    const MultibodyModel model = pair_model(carriage(0.5, Eigen::Vector3d::UnitX()), t, Eigen::Vector3d::Zero());
    MultibodyProvider faulted(model);
    MultibodyProvider clean(model);
    REQUIRE(clean.slip(0) == 0.0);
    SemiImplicitEuler integrator{StepConfig{}};
    State a = rest_state(faulted);
    State b = rest_state(clean);
    double offset_before = 0.0;
    for (int i = 0; i < 120; ++i) {
        if (i == 50) {
            faulted.set_slip(0, 0.5);
        }
        if (i == 80) {
            faulted.set_slip(0, 0.0);
        }
        const State before = a;
        faulted.set_shaft_torque(0, 0.01);
        clean.set_shaft_torque(0, 0.01);
        faulted.step(integrator, a);
        clean.step(integrator, b);
        const TransmissionOutput& o = faulted.outputs().transmissions.at(0);
        REQUIRE_FALSE(clean.outputs().transmissions.at(0).slip_active);
        if (i < 50) {
            REQUIRE(a.q == b.q);
            REQUIRE(a.v == b.v);
            REQUIRE(o.slip_offset == 0.0);
        }
        if (i == 50) {
            REQUIRE(o.slip_active);
            REQUIRE(a.v != b.v);
            REQUIRE(close(o.slip_offset, 0.5 * t.ratio * (a.q[0] - before.q[0]), 1e-12));
        }
        if (i >= 80) {
            REQUIRE_FALSE(o.slip_active);
            REQUIRE(o.slip_offset == offset_before);
        }
        offset_before = o.slip_offset;
    }
}

TEST_CASE("backlash dead band width is reproduced exactly", "[ENG-043][ENG-045]") {
    // Static: x = 0, s = 0, so E = n theta. F = 0 for |E| <= b/2 and k (|E| - b/2) beyond, on both sides: the band
    // is [-b/2, b/2], width b. Probes at b/2 (1 -+ 1e-6). Past the edge F = k delta, delta = 1e-6 b/2; E carries
    // a few eps b/2 of rounding, a few eps / 1e-6 (about 1e-9) relative to delta, inside the 1e-6 asserted.
    for (const TransmissionType type : {TransmissionType::BeltPulley, TransmissionType::GearPair}) {
        Transmission t;
        t.type = type;
        t.ratio = 0.02;
        t.stiffness = 1e5;
        t.backlash = 2e-3;
        Body out = carriage(1.0, Eigen::Vector3d::UnitX());
        if (type == TransmissionType::GearPair) {
            out = shaft(1e-3);
        }
        const MultibodyProvider provider(pair_model(out, t, Eigen::Vector3d::Zero()));
        const double n = mechsim::engine::output_per_input(t);
        const double half = 0.5 * t.backlash;
        const Eigen::Vector2d v = Eigen::Vector2d::Zero();
        for (const double side : {1.0, -1.0}) {
            const TransmissionOutput inside =
                provider.transmission_states(Eigen::Vector2d(side * half * (1.0 - 1e-6) / n, 0.0), v).at(0);
            REQUIRE_FALSE(inside.in_contact);
            REQUIRE(inside.force == 0.0);
            REQUIRE(inside.backlash_position == inside.stretch);
            const double delta = half * 1e-6;
            const TransmissionOutput outside =
                provider.transmission_states(Eigen::Vector2d(side * (half + delta) / n, 0.0), v).at(0);
            REQUIRE(outside.in_contact);
            REQUIRE(outside.backlash_position == side * half);
            REQUIRE(close(outside.force, side * t.stiffness * delta, 1e-6));
        }
    }
}

TEST_CASE("carriage stays put until the shaft has taken up half the backlash", "[ENG-043]") {
    // From the band center with no force, omega is exactly constant and x exactly 0 until a step starts in contact
    // (contact is decided at the step start). The first moving step therefore starts with n theta in
    // (b/2, b/2 + n omega h]: the dead band is reproduced to one step's travel.
    Transmission t;
    t.ratio = 0.02;
    t.stiffness = 1e5;
    t.damping = 10.0;
    t.backlash = 2e-3;
    const MultibodyModel model = pair_model(carriage(1.0, Eigen::Vector3d::UnitX()), t, Eigen::Vector3d::Zero());
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    const double h = integrator.config().h;
    const double omega = 1.0;
    State state = rest_state(provider);
    state.v[0] = omega;
    double onset = -1.0;
    for (int i = 0; i < 200 && onset < 0.0; ++i) {
        const double theta = state.q[0];
        provider.step(integrator, state);
        if (state.q[1] != 0.0) {
            onset = t.ratio * theta;
        } else {
            REQUIRE(state.v[0] == omega);
        }
    }
    REQUIRE(onset > 0.5 * t.backlash);
    REQUIRE(onset <= 0.5 * t.backlash + t.ratio * omega * h * (1.0 + 1e-12));
}

TEST_CASE("self-locking lead screw holds a hanging load with zero drive torque", "[ENG-044]") {
    // Locked: the shaft gain is 0, so the shaft row of f, K and D is zero and theta stays exactly 0. The carriage
    // is a spring-damper on a fixed point: x -> -m g / k. A drive opposing the load unlocks the static gain n.
    // Exactly 0 needs the LU not to pivot: |S11| = J = 1e-2 exceeds |S21| = n (h^2 k + h c) = 3.5e-3, so the zero
    // first row and right-hand side solve to dv_0 = 0 with no rounding.
    const double m = 5.0;
    const double k = 1e6;
    Transmission t;
    t.type = TransmissionType::LeadScrew;
    t.ratio = 4e-3;
    t.self_locking = true;
    t.efficiency = 0.4;
    t.back_efficiency = 0.3;
    t.stiffness = k;
    t.damping = 2.0 * std::sqrt(k * m);
    MultibodyModel model = pair_model(carriage(m, Eigen::Vector3d::UnitZ()), t, Eigen::Vector3d(0, 0, -gravity));
    const double j = 1e-2;
    model.bodies[0] = shaft(j);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    const double n = mechsim::engine::output_per_input(t);
    const Eigen::Matrix2d ab = coupling_matrix(0.0, n);
    const double h = integrator.config().h;
    REQUIRE(j > n * (h * h * k + h * t.damping));
    const long settle = settling_steps(Eigen::Vector2d(j, m).asDiagonal(), k * ab, t.damping * ab, h);
    State state = rest_state(provider);
    for (long i = 0; i < settle; ++i) {
        provider.step(integrator, state);
        REQUIRE(state.q[0] == 0.0);
        REQUIRE(state.v[0] == 0.0);
    }
    const TransmissionOutput& o = provider.outputs().transmissions.at(0);
    REQUIRE(o.power_flow == PowerFlow::Locked);
    REQUIRE(o.shaft_load_torque == 0.0);
    REQUIRE(close(state.q[1], -m * gravity / k, rel));
    provider.set_shaft_torque(0, 1.0);
    const TransmissionOutput lift = provider.transmission_states(state.q, state.v).at(0);
    REQUIRE(lift.power_flow == PowerFlow::Static);
    REQUIRE(lift.shaft_load_torque == n * lift.force);
}

TEST_CASE("a lead screw without self-locking is back-driven by the load", "[ENG-044]") {
    // eta = 1 both ways: y = J theta + n m x obeys y'' = -n m g exactly, the coupling cancelling as [1, n] . a = 0.
    // Semi-implicit Euler from rest: ydot_k = -k h n m g, y_N = -h^2 n m g N (N + 1) / 2, exact to rounding.
    const double m = 5.0;
    const double j = 1e-5;
    Transmission t;
    t.type = TransmissionType::LeadScrew;
    t.ratio = 4e-3;
    t.stiffness = 1e6;
    t.damping = 2.0 * std::sqrt(1e6 * m);
    MultibodyModel model = pair_model(carriage(m, Eigen::Vector3d::UnitZ()), t, Eigen::Vector3d(0, 0, -gravity));
    model.bodies[0] = shaft(j);
    MultibodyProvider provider(model);
    SemiImplicitEuler integrator{StepConfig{}};
    const double h = integrator.config().h;
    const double n = mechsim::engine::output_per_input(t);
    State state = rest_state(provider);
    const long steps = 500;
    for (long i = 0; i < steps; ++i) {
        provider.step(integrator, state);
    }
    const double y = j * state.q[0] + n * m * state.q[1];
    const double target = -h * h * n * m * gravity * static_cast<double>(steps * (steps + 1)) / 2.0;
    REQUIRE(close(y, target, rel));
    REQUIRE(state.q[0] < 0.0);
    REQUIRE(provider.outputs().transmissions.at(0).power_flow == PowerFlow::Backward);
}

TEST_CASE("transmission runs are bitwise identical when repeated", "[ENG-043][ENG-046][ENG-048]") {
    Transmission t;
    t.ratio = 0.015;
    t.efficiency = 0.85;
    t.back_efficiency = 0.7;
    t.stiffness = 2e4;
    t.damping = 20.0;
    t.backlash = 1e-4;
    const MultibodyModel model =
        pair_model(carriage(0.5, Eigen::Vector3d::UnitZ()), t, Eigen::Vector3d(0, 0, -gravity));
    std::vector<std::vector<double>> runs;
    for (int run = 0; run < 2; ++run) {
        MultibodyProvider provider(model);
        SemiImplicitEuler integrator{StepConfig{}};
        State state = rest_state(provider);
        std::vector<double> trace;
        for (int i = 0; i < 500; ++i) {
            if (i == 200) {
                provider.set_slip(0, 0.3);
            }
            provider.set_shaft_torque(0, 0.08 + 0.05 * std::sin(0.02 * i));
            provider.step(integrator, state);
            const TransmissionOutput& o = provider.outputs().transmissions.at(0);
            trace.insert(trace.end(), {state.q[0], state.q[1], state.v[0], state.v[1], o.force, o.shaft_load_torque,
                                       o.slip_offset});
        }
        runs.push_back(trace);
    }
    REQUIRE(runs[0] == runs[1]);
}

TEST_CASE("invalid transmission parameters are rejected with the cause", "[ENG-043][ENG-044][ENG-045][ENG-046]") {
    const auto rejects = [](const Transmission& t, const Body& out) {
        REQUIRE_THROWS_AS(MultibodyProvider(pair_model(out, t, Eigen::Vector3d::Zero())), InvalidModelError);
    };
    const Body slide = carriage(1.0, Eigen::Vector3d::UnitX());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const double ratio : {-0.01, 0.0, nan, inf}) {
        Transmission t;
        t.ratio = ratio;
        rejects(t, slide);
    }
    for (const double eta : {0.0, -0.1, 1.5, nan}) {
        Transmission t;
        t.efficiency = eta;
        rejects(t, slide);
        Transmission b;
        b.back_efficiency = eta;
        rejects(b, slide);
    }
    for (const double backlash : {-1e-6, nan, inf}) {
        Transmission t;
        t.backlash = backlash;
        rejects(t, slide);
    }
    for (const double stiffness : {0.0, -1.0, inf, nan}) {
        Transmission t;
        t.stiffness = stiffness;
        rejects(t, slide);
    }
    for (const double damping : {-1.0, nan}) {
        Transmission t;
        t.damping = damping;
        rejects(t, slide);
    }
    for (const double slip : {-0.1, 1.1, nan}) {
        Transmission t;
        t.slip = slip;
        rejects(t, slide);
    }
    Transmission locking_belt;
    locking_belt.self_locking = true;
    rejects(locking_belt, slide);
    Transmission gear_on_slide;
    gear_on_slide.type = TransmissionType::GearPair;
    rejects(gear_on_slide, slide);
    const Transmission belt_on_rotor;
    rejects(belt_on_rotor, shaft(1e-3));
    Transmission missing;
    missing.output_body = 2;
    rejects(missing, slide);
    Transmission self;
    self.output_body = 0;
    rejects(self, slide);
    Transmission reversed;
    reversed.input_body = 1;
    reversed.output_body = 0;
    rejects(reversed, slide);

    Transmission ok;
    ok.efficiency = 1.0;
    ok.slip = 1.0;
    MultibodyProvider provider(pair_model(slide, ok, Eigen::Vector3d::Zero()));
    REQUIRE_THROWS_AS(provider.set_slip(0, -0.01), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_slip(0, 1.01), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_slip(0, nan), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_slip(1, 0.1), InvalidModelError);
    REQUIRE_THROWS_AS(provider.slip(-1), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_shaft_torque(1, 0.1), InvalidModelError);
    REQUIRE_THROWS_AS(provider.set_shaft_torque(0, inf), InvalidModelError);
    REQUIRE(provider.slip(0) == 1.0);
}

TEST_CASE("reversing a gear pair flips the output's F, q and qd and keeps the shaft load torque", "[ENG-045][ENG-048]") {
    // The model is symmetric under q_out -> -q_out when n -> -n: the output's force, position and rate change sign
    // exactly (IEEE negation is exact), while the shaft sees the same coupling and supplies the same load torque
    // along its + axis. Held bitwise.
    auto run = [](bool reverses) {
        Body out = shaft(2e-3);
        out.joint.stiffness = Eigen::VectorXd::Constant(1, 5.0);
        Transmission t;
        t.type = TransmissionType::GearPair;
        t.ratio = 3.0;
        t.reverses = reverses;
        t.stiffness = 50.0;
        t.damping = 0.6;
        MultibodyModel model = pair_model(out, t, Eigen::Vector3d::Zero());
        model.bodies[0] = shaft(1e-3);
        MultibodyProvider provider(model);
        SemiImplicitEuler integrator{StepConfig{}};
        State state = rest_state(provider);
        for (int i = 0; i < 50; ++i) {
            provider.set_shaft_torque(0, 0.01);
            provider.step(integrator, state);
        }
        const TransmissionOutput o = provider.outputs().transmissions.at(0);
        return std::array<double, 6>{o.force, state.q[1], state.v[1], o.shaft_load_torque, state.q[0], state.v[0]};
    };
    const std::array<double, 6> forward = run(false);
    const std::array<double, 6> reversed = run(true);
    REQUIRE(forward[0] != 0.0);
    REQUIRE(forward[1] != 0.0);
    REQUIRE(forward[2] != 0.0);
    CHECK(reversed[0] == -forward[0]);
    CHECK(reversed[1] == -forward[1]);
    CHECK(reversed[2] == -forward[2]);
    CHECK(reversed[3] == forward[3]);
    CHECK(reversed[4] == forward[4]);
    CHECK(reversed[5] == forward[5]);
}
