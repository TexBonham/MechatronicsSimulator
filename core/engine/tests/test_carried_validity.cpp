#include <Eigen/Core>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

#include "beam_helpers.hpp"
#include "carried_helpers.hpp"
#include "energy_helpers.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/member_loads.hpp"
#include "mechsim/engine/multibody_provider.hpp"

using mechsim::engine::add_lumped_beam;
using mechsim::engine::BeamEnds;
using mechsim::engine::Body;
using mechsim::engine::carried_ratio_limit;
using mechsim::engine::carried_validity_margin;
using mechsim::engine::carried_window_bucket;
using mechsim::engine::carried_window_periods;
using mechsim::engine::carried_window_periods_max;
using mechsim::engine::JointType;
using mechsim::engine::MemberOutput;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::carried_rig;
using mechsim::engine::test::CarriedRig;
using mechsim::engine::test::flexural_rigidity;
using mechsim::engine::test::lumped_first_frequency;
using mechsim::engine::test::mechanical_energy;
using mechsim::engine::test::pinned_chain_compliance;
using mechsim::engine::test::settling_steps;
using mechsim::engine::test::start_state;

// The pair's work and the validity flag (ENG-052, brief M3-E7c; derivation in member_loads.hpp, Work and validity).
//
// Rigs: the pinned-pinned steel bar of carried_helpers (1 m, 10 mm square, 8 segments) on the world, gravity off, a
// 1 g carriage at midspan on the member axis (no moment arm), loaded by a force along y applied to the carriage each
// step; its joint passes the force to the member one step late. The carriage's mass loads the member's inertia
// (brief M3-E7d); at 1 g, m_c / (17/35 m_beam) = 0.26 %, so the member's modes and shares below are the bare chain's
// to 0.13 % in omega, inside every bound's slack; omega_1 is the loaded chain's own (the drive frequencies use it).
// The member's first mode at midspan: the continuum share of the collocated static compliance is 96 / pi^4 = 0.9855 (pinned-pinned modes sin(n pi x / L), odd n only at
// midspan, shares 1 / n^4 over sum 1 / n^4); the chain's differs by its frequency error (2.5 % at 8 segments, the
// share goes as omega^-2) and its static error (2 %, ENG-022): s1 >= 0.9 is taken as the bound. The next mode with a
// share at midspan is n = 3, omega_3 / omega_1 near 9. omega_1 is the chain's own with the carriage's mass on it
// (lumped_first_frequency of the rig at the carriage's position).

namespace {

constexpr double first_share_lower = 0.9;

CarriedRig loaded_rig(double damping_scale) {
    CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1e-3, 0.0, true,
                                 mechsim::engine::test::ParentKind::World, damping_scale);
    rig.model.gravity = Eigen::Vector3d::Zero();
    return rig;
}

double chain_first_frequency(const CarriedRig& rig) {
    const MultibodyProvider provider(rig.model);
    return lumped_first_frequency(provider, start_state(provider, rig.carriage, 0.5).q);
}

struct Run {
    MemberOutput last;
    double flagged_at = -1.0;  // unresolved_time when first flagged, s
    double peak_force = 0.0;
};

// Steps the rig with force(t) on the carriage (applied before each step at the step's start time).
template <typename Force>
Run run_forced(MultibodyProvider& provider, State& state, Eigen::Index carriage, int steps, double h, Force force) {
    SemiImplicitEuler integrator(StepConfig{h});
    Run run;
    for (int n = 0; n < steps; ++n) {
        provider.apply_body_wrench(carriage, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, force(state.t), 0.0));
        static_cast<void>(provider.step(integrator, state));
        run.last = provider.outputs().members.front();
        if (run.last.unresolved && run.flagged_at < 0.0) {
            run.flagged_at = run.last.unresolved_time;
        }
    }
    return run;
}

}  // namespace

TEST_CASE("carried validity: a static load does work P u, twice the stored energy, at a ratio of one",
          "[ENG-052]") {
    // Critically damped first mode (carried_helpers settling): every mode's step response rises monotonically, so
    // the deflection never passes the quasi-static value: peak ratio <= 1. Settled after settling_steps to 1e-15.
    // W = F (d_end - d_0) to second order in the steps (the work rows are the gradient of the published deflection),
    // d_end = the chain's lever-rule value c F (pinned_chain_compliance / E I, ruling 57) and u_qs = c F linear: both
    // within (u / L)^2 of the geometric deflection, u = 1.3e-4 m here, so 1e-6 relative. The member stores
    // E = F u / 2 (mechanical_energy: the node springs, gravity off, at rest).
    const CarriedRig rig = loaded_rig(1.0);
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, 0.5);
    constexpr double load = 1.0;
    const double h = 1e-3;
    const Run run = run_forced(provider, state, rig.carriage, settling_steps(rig, h), h, [](double) { return -load; });
    const double chain = pinned_chain_compliance(8, 0.5, 1.0) / flexural_rigidity(rig.spec) * load;
    const double energy = mechanical_energy(provider, state);
    std::printf("static: chain c F %.9e m, deflection %.9e m, u_qs %.9e m; W %.9e J against F u %.9e J; stored %.9e J "
                "against W / 2; ratio %.9f, peak %.9f\n",
                chain, run.last.carried_deflection, run.last.quasi_static_deflection, run.last.carried_work,
                load * run.last.carried_deflection, energy, run.last.dynamic_ratio, run.last.peak_ratio);
    CHECK(std::abs(run.last.carried_deflection - chain) <= 1e-6 * chain);
    CHECK(std::abs(run.last.quasi_static_deflection - chain) <= 1e-6 * chain);
    CHECK(std::abs(run.last.carried_work - (load * chain)) <= 1e-6 * load * chain);
    CHECK(std::abs(energy - (0.5 * run.last.carried_work)) <= 1e-6 * run.last.carried_work);
    CHECK(std::abs(run.last.dynamic_ratio - 1.0) <= 1e-6);
    CHECK(run.last.peak_ratio <= 1.0 + 1e-6);
    CHECK(run.last.peak_carried_force == load);
    CHECK(run.last.peak_quasi_static >= run.last.quasi_static_deflection);
    CHECK_FALSE(run.last.unresolved);
    CHECK(run.last.unresolved_time == 0.0);
}

TEST_CASE("carried validity: without a load the work and the ratio stay zero", "[ENG-052]") {
    // No force and no gravity: the pair is zero, u_qs is zero and the ratio is defined as zero, never flagged.
    const CarriedRig rig = loaded_rig(0.0);
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, 0.5);
    const Run run = run_forced(provider, state, rig.carriage, 100, 1e-3, [](double) { return 0.0; });
    CHECK(run.last.carried_work == 0.0);
    CHECK(run.last.peak_quasi_static == 0.0);
    CHECK(run.last.peak_ratio == 0.0);
    CHECK_FALSE(run.last.unresolved);
}

TEST_CASE("carried validity: a suddenly applied load on an undamped member approaches twice and is not flagged",
          "[ENG-052]") {
    // Backward Euler on a step load (header): each mode's error e_n = -u_s rho^n cos(n theta), rho = (1 + x^2)^-1/2,
    // so u_n < 2 u_s, and every mode's share is positive at the loaded joint: ratio < 2 below the limit 2.04. The
    // first mode peaks at n theta = pi, theta = atan x: 1 + rho^(pi / atan x); the others add >= 0, so the peak is
    // at least s1 (1 + (1 + x^2)^(-pi / (2 atan x))). Upper bound with the geometric term (u / L)^2 < 1e-7.
    const CarriedRig rig = loaded_rig(0.0);
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, 0.5);
    const double h = 1e-4;
    const double omega = chain_first_frequency(rig);
    const double x = omega * h;
    const int steps = static_cast<int>(std::ceil(3.0 * 2.0 * std::numbers::pi / x));
    const Run run = run_forced(provider, state, rig.carriage, steps, h, [](double) { return -1.0; });
    const double lower = first_share_lower * (1.0 + std::pow(1.0 + (x * x), -std::numbers::pi / (2.0 * std::atan(x))));
    std::printf("sudden: x %.6f, peak ratio %.9f in [%.6f, 2], limit %.3f\n", x, run.last.peak_ratio, lower,
                carried_ratio_limit);
    CHECK(run.last.peak_ratio >= lower);
    CHECK(run.last.peak_ratio <= 2.0 * (1.0 + 1e-6));
    CHECK_FALSE(run.last.unresolved);
}

TEST_CASE("carried validity: a force at the member's first frequency is flagged when the resonant growth passes the "
          "limit",
          "[ENG-052]") {
    // F = F0 sin(omega_1 t) from rest. The first mode: u_1 = s1 c F0 (sin th - th cos th) / 2, th = omega_1 t (the
    // resonant solution, growing as F0 omega t / (2 k)); a higher mode p at r = omega_1 / omega_p <= 1/9: (1 - s1) c
    // F0 / (1 - r^2) (sin th - r sin(th / r)), within (1 - s1)(sin th +- 0.25). u_qs peaks at c F0 from th = pi / 2.
    // |ratio| first passes 2.04 in (3 pi / 2, 2 pi), where both terms are negative:
    //   upper g(th) = (th cos th - sin th) / 2 (s1 = 1; s1 < 1 only lowers it while g > 1.25),
    //   lower (s1 g + (1 - s1)(|sin th| - 0.25)) (1 - x th / 2) at s1 = 0.9, backward Euler's damping zeta = x / 2
    //   taking at most x th / 2 off the growth.
    // The flag's th lies between the two crossings, give or take the one-step lag and the sampling (3 x).
    const CarriedRig rig = loaded_rig(0.0);
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, 0.5);
    const double h = 2e-5;
    const double omega = chain_first_frequency(rig);
    const double x = omega * h;
    const int steps = static_cast<int>(std::ceil(5.0 * 2.0 * std::numbers::pi / x));
    const Run run =
        run_forced(provider, state, rig.carriage, steps, h, [omega](double t) { return std::sin(omega * t); });
    const auto g = [](double th) { return 0.5 * ((th * std::cos(th)) - std::sin(th)); };
    double early = -1.0;
    double late = -1.0;
    for (double th = 1.5 * std::numbers::pi; th <= 2.0 * std::numbers::pi; th += 1e-5) {
        const double low =
            ((first_share_lower * g(th)) + ((1.0 - first_share_lower) * (std::abs(std::sin(th)) - 0.25))) *
            (1.0 - (0.5 * x * th));
        if (early < 0.0 && g(th) > carried_ratio_limit) {
            early = th;
        }
        if (late < 0.0 && low > carried_ratio_limit) {
            late = th;
        }
    }
    const double flagged = omega * run.flagged_at;
    std::printf("resonant: x %.6f, flagged at omega t %.6f (step %.0f) in [%.6f, %.6f]; peak ratio %.4f after five "
                "periods (th / 2 = %.4f), work %.6e J\n",
                x, flagged, run.flagged_at / h, early, late, run.last.peak_ratio, 5.0 * std::numbers::pi,
                run.last.carried_work);
    REQUIRE(run.last.unresolved);
    CHECK(flagged >= early - (3.0 * x));
    CHECK(flagged <= late + (3.0 * x));
    CHECK(run.last.carried_work > 0.0);
}

TEST_CASE("carried validity: the same force at half the first frequency is not flagged", "[ENG-052]") {
    // At r = 1/2 from rest: u_1 = s1 c F0 (sin a - sin(2 a) / 2) / (3 / 4), a = omega_1 t / 2, at most
    // s1 1.299 / 0.75 = 1.732 s1 (a = 2 pi / 3); the higher modes add at most (1 - s1)(1 + r)/(1 - r^2) <= 1.06 (1 - s1)
    // at r <= 1/18: the ratio stays below 1.732 for any s1 <= 1, under the limit 2.04, and backward Euler only
    // lowers it. Lower bound: 0.9 of the first mode's peak less backward Euler's damping over 4 pi / 3 periods,
    // x (2 pi) / 2.
    const CarriedRig rig = loaded_rig(0.0);
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, 0.5);
    const double h = 5e-5;
    const double omega = chain_first_frequency(rig);
    const double x = omega * h;
    const int steps = static_cast<int>(std::ceil(8.0 * 2.0 * std::numbers::pi / x));
    const Run run =
        run_forced(provider, state, rig.carriage, steps, h, [omega](double t) { return std::sin(0.5 * omega * t); });
    const double peak = (std::sin(2.0 * std::numbers::pi / 3.0) - (0.5 * std::sin(4.0 * std::numbers::pi / 3.0))) / 0.75;
    std::printf("half frequency: peak ratio %.6f against %.6f\n", run.last.peak_ratio, peak);
    CHECK(run.last.peak_ratio <= peak * (1.0 + 1e-6));
    CHECK(run.last.peak_ratio >= first_share_lower * peak * (1.0 - (x * std::numbers::pi)));
    CHECK_FALSE(run.last.unresolved);
}

TEST_CASE("carried validity: the window is ln(1 / delta) / (pi x + 2 pi zeta) periods, capped, in quarter-period "
          "buckets",
          "[ENG-052]") {
    // member_loads.hpp, Validity (brief M3-E7e): N = ln 50 / (pi x + 2 pi zeta), at most 100; b = floor(pi / (2 x)).
    const double ln50 = std::log(1.0 / carried_validity_margin);
    CHECK(std::abs(carried_window_periods(0.028, 0.01) - (ln50 / (std::numbers::pi * 0.048))) <= 1e-12);
    CHECK(std::abs(carried_window_periods(0.5, 0.0) - (ln50 / (std::numbers::pi * 0.5))) <= 1e-12);
    CHECK(carried_window_periods(1e-4, 0.0) == carried_window_periods_max);
    CHECK(carried_window_periods(0.0, 0.0) == carried_window_periods_max);
    CHECK(carried_window_periods(0.028, -1.0) == carried_window_periods(0.028, 0.0));
    const double bound = ln50 / (100.0 * std::numbers::pi);
    CHECK(carried_window_periods(bound * 1.001, 0.0) < carried_window_periods_max);
    CHECK(carried_window_periods(bound * 0.999, 0.0) == carried_window_periods_max);
    CHECK(carried_window_bucket(0.028) == 56);
    CHECK(carried_window_bucket(std::numbers::pi / 2.0) == 1);
    CHECK(carried_window_bucket(10.0) == 1);
    CHECK(carried_window_bucket(0.0) == 1000000000000000);
}

TEST_CASE("carried validity: an impulse-like load is forgotten, and a matched resonance long after is flagged within "
          "the predicted time",
          "[ENG-052]") {
    // The review's case (ruling 73, finding 1). The loaded rig with light Rayleigh damping, zeta_1 = 0.01 (the node
    // dampers at 0.01 of carried_helpers' critical setting: zeta_1 = 0.01 beta omega_1 / 2). A load of 20 N held for
    // half a first period (the worst case for what it leaves: the member at 2 u_s at rest when it is removed, a free
    // ringing of 2 u_s = 40 c), then nothing for 70 periods, then F0 sin(omega_1 t'), F0 = 1 N, t' from the start of
    // the drive. A running peak base would sit at 20 c for the rest of the run and never flag the drive (the largest
    // deflection is below 2.04 x 20 c: asserted). The window forgets: N = ln 50 / (pi x + 2 pi zeta) periods (x =
    // omega_1 h = 0.028), 26 here, so at the drive's start the base is the drive's own u_qs and the old ringing is
    // below r = 40 c e^(-(pi x + 2 pi zeta) 69.5) (the decay per period of member_loads.hpp, Validity). The flag's
    // time: the bounds of the resonant test above with the old ringing r on either side and the node damping in the
    // lower bound's factor (1 - (x / 2 + zeta) th) (the envelope (1 - e^(-zeta th)) / (2 zeta) >= th (1 - zeta th / 2)
    // / 2, backward Euler's x / 2 as before), give or take 3 x.
    const double zeta = 0.01;
    const CarriedRig rig = loaded_rig(zeta);
    MultibodyProvider provider(rig.model);
    State state = start_state(provider, rig.carriage, 0.5);
    const double omega = chain_first_frequency(rig);
    const double damping_ratio = zeta * rig.beta * omega / 2.0;
    const double h = 0.028 / omega;
    const double x = omega * h;
    const double period = 2.0 * std::numbers::pi / omega;
    const double drive_start = 70.0 * period;
    const double pulse = 20.0;
    const int steps = static_cast<int>(std::ceil((drive_start + (3.0 * period)) / h));
    double largest = 0.0;
    double before_drive = 0.0;
    SemiImplicitEuler integrator(StepConfig{h});
    Run run;
    for (int n = 0; n < steps; ++n) {
        double force = 0.0;
        if (state.t < 0.5 * period) {
            force = -pulse;
        }
        if (state.t >= drive_start) {
            force = std::sin(omega * (state.t - drive_start));
        }
        provider.apply_body_wrench(rig.carriage, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, force, 0.0));
        static_cast<void>(provider.step(integrator, state));
        run.last = provider.outputs().members.front();
        largest = std::max(largest, run.last.carried_deflection);
        if (state.t < drive_start) {
            before_drive = std::max(before_drive, run.last.peak_ratio);
        }
        if (run.last.unresolved && run.flagged_at < 0.0) {
            run.flagged_at = run.last.unresolved_time;
        }
    }
    const double c = pinned_chain_compliance(8, 0.5, 1.0) / flexural_rigidity(rig.spec);
    const double residual = 2.0 * pulse * std::exp(-std::numbers::pi * (x + (2.0 * damping_ratio)) * 69.5);
    const auto g = [](double th) { return 0.5 * ((th * std::cos(th)) - std::sin(th)); };
    double early = -1.0;
    double late = -1.0;
    for (double th = 1.5 * std::numbers::pi; th <= 2.0 * std::numbers::pi; th += 1e-5) {
        const double low =
            (((first_share_lower * g(th)) + ((1.0 - first_share_lower) * (std::abs(std::sin(th)) - 0.25))) *
             (1.0 - (((0.5 * x) + damping_ratio) * th))) -
            residual;
        if (early < 0.0 && g(th) + residual > carried_ratio_limit) {
            early = th;
        }
        if (late < 0.0 && low > carried_ratio_limit) {
            late = th;
        }
    }
    const double flagged = omega * (run.flagged_at - drive_start);
    const double expected_periods = carried_window_periods(x, damping_ratio);
    std::printf("window: x %.4f, zeta %.5f, N %.4f periods (engine %.4f, %ld steps); old ringing at the drive below "
                "%.3e c; peak ratio before the drive %.4f; flagged at omega t' %.4f in [%.4f, %.4f]; largest "
                "deflection over the since-load base %.4f (a running peak would not flag)\n",
                x, damping_ratio, expected_periods, run.last.window_periods,
                static_cast<long>(run.last.window_steps), residual, before_drive, flagged, early, late,
                largest / run.last.peak_quasi_static);
    CHECK(std::abs(run.last.window_periods / expected_periods - 1.0) <= 1e-6);
    CHECK(run.last.window_steps == 4 * static_cast<Eigen::Index>(std::ceil(expected_periods)) * carried_window_bucket(x));
    CHECK(run.last.peak_quasi_static >= pulse * c * (1.0 - 1e-6));
    CHECK(largest / run.last.peak_quasi_static < carried_ratio_limit);
    REQUIRE(run.last.unresolved);
    CHECK(run.flagged_at >= drive_start);
    CHECK(flagged >= early - (3.0 * x));
    CHECK(flagged <= late + (3.0 * x));
}

namespace {

// The finding's shape (ruling 69): member A on the world carries a frame T (fixed to the world at A's midspan, on
// A's axis); member B, an identical bar, hangs on T 5 cm below A's axis and carries P (1 kg, prismatic along B, on
// A's axis height, 5 cm above B's). Both undamped; gravity off. P runs at 1 m/s and a damper brakes it (time
// constant 1 / omega_0, omega_0 the bare chain's first frequency): the braking force along B, 5 cm above B's axis,
// bends B, and B's ringing reaches A only through T's reaction, never the other way.
// With the carried mass (brief M3-E7d) the bare chains' equal frequencies no longer coincide: A carries T's whole
// subtree (T, B and P, m_cA = 3.8 kg on a 0.785 kg bar) and rings near 0.3 omega_0, B carries P at s_P and rings near
// 0.55 omega_0. The two loaded frequencies, each the engine's own: omega_A from A alone carrying one body of mass
// m_cA at midspan (A's block and modes involve A's coordinates only: T is fixed to the world, so B's coordinates are
// independent of A's in H and in B), omega_B from B alone on T with P at its braked position. That position is
// exact for the scheme: P coasts 0.02 s at 1 m/s, then the implicit damper c = omega_0 on 1 kg gives v_{n+1} = v_n /
// (1 + h c), a distance sum h v_n = 1 / c: s_P = 0.32 + 1 / omega_0. The matched rig scales A's Young's modulus by
// (omega_B / omega_A)^2 (K is linear in E, M independent of it), which puts A's loaded frequency on B's.
struct TwoMembers {
    MultibodyModel model;
    Eigen::Index frame = -1;
    Eigen::Index carriage = -1;
    double omega = 0.0;           // omega_0, the bare chain's first frequency, rad/s
    double carried_by_a = 0.0;    // m_cA, kg
    double member_mass = 0.0;     // m_A, kg
};

constexpr double coast = 0.02;
constexpr double coast_start = 0.3;
constexpr double push_amplitude = 10.0;  // N

TwoMembers two_members(double a_stiffness_scale = 1.0, bool with_a = true) {
    TwoMembers rig;
    rig.model.gravity = Eigen::Vector3d::Zero();
    const mechsim::engine::BeamSpec spec = mechsim::engine::test::steel_bar(1.0, BeamEnds::SimplySupported, 8);
    mechsim::engine::BeamSpec spec_a = spec;
    spec_a.youngs_modulus *= a_stiffness_scale;
    rig.member_mass = spec.density * spec.area * spec.length;
    if (with_a) {
        static_cast<void>(add_lumped_beam(rig.model, spec_a, -1));
    }
    const auto first_body = static_cast<Eigen::Index>(rig.model.bodies.size());
    Body frame;
    frame.parent = -1;
    frame.joint.type = JointType::Fixed;
    frame.joint.position = Eigen::Vector3d(0.5, 0.0, 0.0);
    frame.mass = 2.0;
    frame.inertia = 0.01 * Eigen::Matrix3d::Identity();
    rig.model.bodies.push_back(frame);
    rig.frame = static_cast<Eigen::Index>(rig.model.bodies.size()) - 1;
    static_cast<void>(add_lumped_beam(rig.model, spec, rig.frame, Eigen::Vector3d(-0.5, -0.05, 0.0)));
    Body carriage;
    carriage.parent = rig.frame;
    carriage.joint.type = JointType::Prismatic;
    carriage.joint.axis = Eigen::Vector3d::UnitX();
    carriage.joint.position = Eigen::Vector3d(-0.5, 0.0, 0.0);
    carriage.mass = 1.0;
    carriage.inertia = 1e-3 * Eigen::Matrix3d::Identity();
    rig.model.bodies.push_back(carriage);
    rig.carriage = static_cast<Eigen::Index>(rig.model.bodies.size()) - 1;
    for (Eigen::Index b = first_body; b < static_cast<Eigen::Index>(rig.model.bodies.size()); ++b) {
        rig.carried_by_a += rig.model.bodies[static_cast<std::size_t>(b)].mass;
    }
    if (with_a) {
        rig.model.members.front().carried.push_back(rig.frame);
    }
    rig.model.members.back().carried.push_back(rig.carriage);
    MultibodyModel bare;
    bare.gravity = Eigen::Vector3d::Zero();
    static_cast<void>(add_lumped_beam(bare, spec, -1));
    rig.omega = lumped_first_frequency(MultibodyProvider(bare));
    return rig;
}

// omega_A at scale 1: A alone carrying one fixed body of T's subtree mass at midspan.
double loaded_a_frequency(const TwoMembers& rig) {
    MultibodyModel alone;
    alone.gravity = Eigen::Vector3d::Zero();
    static_cast<void>(add_lumped_beam(alone, mechsim::engine::test::steel_bar(1.0, BeamEnds::SimplySupported, 8), -1));
    Body lump;
    lump.parent = -1;
    lump.joint.type = JointType::Fixed;
    lump.joint.position = Eigen::Vector3d(0.5, 0.0, 0.0);
    lump.mass = rig.carried_by_a;
    lump.inertia = 0.01 * Eigen::Matrix3d::Identity();
    alone.bodies.push_back(lump);
    alone.members.front().carried.push_back(static_cast<Eigen::Index>(alone.bodies.size()) - 1);
    return lumped_first_frequency(MultibodyProvider(alone));
}

// omega_B: B alone on T with P at s (the stop: s_P = 0.32 + 1 / omega_0).
double loaded_b_frequency(double s) {
    const TwoMembers alone = two_members(1.0, false);
    const MultibodyProvider provider(alone.model);
    Eigen::VectorXd q = provider.neutral_positions();
    q[provider.position_offset(alone.carriage)] = s;
    return lumped_first_frequency(provider, q);
}

struct TwoRun {
    MemberOutput carrying;
    MemberOutput carried;
};

TwoRun run_two(double h, double duration, double a_stiffness_scale = 1.0, double drive = 0.0) {
    const TwoMembers rig = two_members(a_stiffness_scale);
    MultibodyProvider provider(rig.model);
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    state.q[provider.position_offset(rig.carriage)] = coast_start;
    if (drive == 0.0) {
        state.v[provider.velocity_offset(rig.carriage)] = 1.0;
    }
    SemiImplicitEuler integrator(StepConfig{h});
    const auto steps = static_cast<int>(std::ceil(duration / h));
    for (int n = 0; n < steps; ++n) {
        if (drive == 0.0 && n == static_cast<int>(std::lround(coast / h))) {
            provider.set_joint_drive(rig.carriage, 0, 0.0, 0.0, rig.omega * 1.0);
        }
        if (drive > 0.0) {
            Eigen::VectorXd push = Eigen::VectorXd::Zero(provider.velocity_size());
            push[provider.velocity_offset(rig.carriage)] = push_amplitude * std::sin(drive * state.t);
            provider.set_generalized_forces(push);
        }
        static_cast<void>(provider.step(integrator, state));
    }
    return TwoRun{provider.outputs().members.front(), provider.outputs().members.back()};
}

}  // namespace

TEST_CASE("carried validity: loaded, the carrying member's frequency drops as the one-mode estimate and B's ringing "
          "no longer pumps it",
          "[ENG-052]") {
    // omega_A against the one-mode estimate sqrt(48 E I / L^3 / (m_cA + 17/35 m_A)) (test_carried_mass.cpp, 2 % at
    // eight segments). Off resonance (omega_B / omega_A >= 1.5 asserted) B's ringing cannot grow A's response over
    // the twenty periods the matched rig below needs to be flagged: not flagged.
    const TwoMembers rig = two_members();
    const double omega_a = loaded_a_frequency(rig);
    const double omega_b = loaded_b_frequency(coast_start + coast + (1.0 / rig.omega));
    const double k = 48.0 * flexural_rigidity(mechsim::engine::test::steel_bar(1.0, BeamEnds::SimplySupported, 8));
    const double estimate = std::sqrt(k / (rig.carried_by_a + ((17.0 / 35.0) * rig.member_mass)));
    const double h = 5e-4;
    const double duration = 20.0 * 2.0 * std::numbers::pi / omega_a;
    const TwoRun run = run_two(h, duration);
    std::printf("two members, loaded: omega_0 %.4f, omega_A %.4f (estimate %.4f), omega_B %.4f rad/s; A work %.6e J, "
                "peak ratio %.4f, flagged %d; B peak ratio %.4f\n",
                rig.omega, omega_a, estimate, omega_b, run.carrying.carried_work, run.carrying.peak_ratio,
                static_cast<int>(run.carrying.unresolved), run.carried.peak_ratio);
    CHECK(std::abs(omega_a / estimate - 1.0) <= 0.02);
    CHECK(omega_b / omega_a >= 1.5);
    CHECK_FALSE(run.carrying.unresolved);
}

TEST_CASE("carried validity: a member driven through another carried member at its own loaded frequency is flagged",
          "[ENG-052]") {
    // Measured on the stop rig: with the mass on both members, A's largest quasi-static load is the brake's first
    // steps and B's ringing (BE-damped, x = omega_B h) reaches A at about 1/50 of it, so even matched it would need
    // some hundred undamped periods to pass 2.04: the stop no longer shows the pumping. The flag is shown on the same
    // two members with a sustained source instead: P pushed along B through its joint (a joint force, so P's
    // reaction, 5 cm above B's axis, bends B) by 10 N sin(omega_B t), omega_B B's loaded frequency with P at 0.3 m
    // (it moves by 10 / (1 kg omega_B^2) = 1.4 mm). B rings up at its own frequency and
    // its ringing reaches A only through T's reaction. A matched (E scaled by (omega_B / omega_A)^2): resonant growth
    // on top of a growing load, ratio ~ omega t / 2, flagged within twenty periods; A at its own loaded frequency,
    // omega_B / omega_A >= 1.5 off it, follows the load with a gain near 1 / ((omega_B / omega_A)^2 - 1), under one,
    // and is not flagged. W_A > 0 in the matched case: the pair pumps energy into A that B never loses.
    const TwoMembers rig = two_members();
    const double omega_b = loaded_b_frequency(coast_start);
    const double omega_a = loaded_a_frequency(rig);
    const double scale = std::pow(omega_b / omega_a, 2.0);
    const double h = 2e-4;
    const double duration = 20.0 * 2.0 * std::numbers::pi / omega_b;
    const TwoRun matched = run_two(h, duration, scale, omega_b);
    const TwoRun apart = run_two(h, duration, 1.0, omega_b);
    std::printf("two members driven at omega_B %.4f rad/s (omega_A %.4f): matched (A's E x %.4f): A work %.6e J, peak "
                "ratio %.4f, flagged %d at %.4f s; apart: A work %.6e J, peak ratio %.4f, flagged %d; B peak ratio %.4f, "
                "flagged %d\n",
                omega_b, omega_a, scale, matched.carrying.carried_work, matched.carrying.peak_ratio,
                static_cast<int>(matched.carrying.unresolved), matched.carrying.unresolved_time,
                apart.carrying.carried_work, apart.carrying.peak_ratio, static_cast<int>(apart.carrying.unresolved),
                apart.carried.peak_ratio, static_cast<int>(apart.carried.unresolved));
    CHECK(omega_b / omega_a >= 1.5);
    CHECK(matched.carrying.unresolved);
    CHECK(matched.carrying.carried_work > 0.0);
    CHECK_FALSE(apart.carrying.unresolved);
}

TEST_CASE("carried validity: the work and the flag are deterministic", "[ENG-052]") {
    const TwoMembers rig = two_members();
    const double duration = 5.0 * 2.0 * std::numbers::pi / rig.omega;
    const TwoRun first = run_two(1e-4, duration);
    const TwoRun second = run_two(1e-4, duration);
    CHECK(first.carrying.carried_work == second.carrying.carried_work);
    CHECK(first.carrying.peak_ratio == second.carrying.peak_ratio);
    CHECK(first.carrying.unresolved == second.carrying.unresolved);
    CHECK(first.carrying.unresolved_time == second.carrying.unresolved_time);
    CHECK(first.carried.carried_work == second.carried.carried_work);
    CHECK(first.carried.peak_quasi_static == second.carried.peak_quasi_static);
}
