#include <Eigen/Core>
#include <algorithm>
#include <array>
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
using mechsim::engine::test::pinned_chain_largest;
using mechsim::engine::test::pinned_chain_member_max;
using mechsim::engine::test::pinned_chain_transfer;
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

TEST_CASE("carried validity: the window is ln(1 / delta) / (2 pi zeta) periods of physical time, capped, in "
          "quarter-period buckets",
          "[ENG-052]") {
    // member_loads.hpp, Definition (a) (brief M3-E7f, ruling 74): N = ln 50 / (2 pi zeta) from the node damping
    // alone, the cap of 100 for an undamped member and below zeta = ln 50 / (200 pi); b = floor(pi / (2 x)).
    const double ln50 = std::log(1.0 / carried_validity_margin);
    CHECK(std::abs(carried_window_periods(0.01) - (ln50 / (2.0 * std::numbers::pi * 0.01))) <= 1e-12);
    CHECK(std::abs(carried_window_periods(0.5) - (ln50 / std::numbers::pi)) <= 1e-12);
    CHECK(carried_window_periods(0.0) == carried_window_periods_max);
    CHECK(carried_window_periods(-1.0) == carried_window_periods_max);
    const double bound = ln50 / (200.0 * std::numbers::pi);
    CHECK(carried_window_periods(bound * 1.001) < carried_window_periods_max);
    CHECK(carried_window_periods(bound * 0.999) == carried_window_periods_max);
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
    // deflection is below 2.04 x 20 c: asserted). The window forgets: N = ln 50 / (2 pi zeta) periods (ruling 74:
    // node damping only), 63 here, under the 69.5 periods between the pulse and the drive (x = omega_1 h = 0.028), so
    // at the drive's start the base is the drive's own u_qs and the old ringing is
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
    const double expected_periods = carried_window_periods(damping_ratio);
    REQUIRE(std::ceil(expected_periods) < 69.5);
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

// The gate of the flag's definition (briefs M3-E7f, M3-E7g, rulings 74, 77; member_loads.hpp, Definition). The
// pinned-pinned bar under gravity (-y, 9.81) carries a 1 kg carriage standing at s0 = L / 4, released from straight,
// node damping at 0.1 of carried_helpers' critical setting (D = 0.1 beta K, beta = 2 / omega_rig: zeta_p omega_p =
// 0.1 omega_p^2 / omega_rig, smallest for the first mode), at h = 1 ms and 0.1 ms and at 8 and 16 segments.
//   Settled. Every mode's release transient (start: the member straight, at most 1 + g / (C W) of the base in each
// mode's share, below 2 (1 + g / (C W)) in all) decays at least as e^(-r t), r = 0.1 omega_1^2 / omega_rig
// (backward Euler only adds loss); the evaluation start T_s = ln(1e6) / r leaves below 2 (1 + g / (C W)) 1e-6 of it.
// At rest d_a - g_a = C(a, s0) W exactly at every load node a of the linear chain (Definition (b)), so the member-wide
// sample is max over nodes of C(a, s0) W (pinned_chain_member_max) and the base is the floor F = C_max W (ruling 76:
// the carriage has no limits; the window's peak, the same nodal maximum, is at most C_max W since abs(C(a, s)) <=
// sqrt(C(a, a) C(s, s))); the ratio reads max_a C(a, s0) / C_max to the transient left plus the geometric
// nonlinearity, relative at most 4 theta^2 with theta <= 4 u / L the largest segment rotation (u the midspan
// deflection under own weight and the carried weight: 5 w L^4 / (384 E I) + W L^3 / (48 E I)). Checked at every step
// over two windows (the window turns over twice) and never flagged; before T_s the ratio is 0 and nothing is flagged.
//   The window. N depends on the node damping only, so it is the same at both steps; each of the 4 ceil(N) buckets
// holds b = floor(pi / (2 x)) steps, b h in (T / 4 - h, T / 4], so the two windows' lengths in time differ by less
// than 4 ceil(N) times the coarse step.
//   Moving. Then the carriage coasts at c = 0.25 m/s from L / 4 to 3 L / 4 (frictionless prismatic across gravity: c
// is constant). For mode p, q_s = W phi_p(s) / k_p, and the error e = q - q_s obeys e'' + 2 zeta w e' + w^2 e =
// -(q_s'' + 2 zeta w q_s'): the velocity jump at the start leaves a free ringing of at most q_s' / w, and for a slowly
// varying right side the forced part is to leading order -(q_s'' + 2 zeta w q_s') / w^2. With alpha = pi c / (L
// omega_1), omega_1 the loaded frequency at mid-span (the lowest on the path), q_s' / q_s = (pi c / L) cot(pi s / L)
// <= pi c / L on [L / 4, L / 2] and abs(q_s'' / q_s) <= (pi c / L)^2, so per mode abs(e_p) <= delta abs(q_s,p) with
// delta = alpha (1 + 2 zeta_1) + alpha^2: higher modes have alpha_p <= alpha / p and zeta_p alpha_p = zeta_1 alpha_1
// (zeta_p proportional to omega_p here). zeta_1 = 0.1 omega_1 / omega_rig <= 0.1 omega_start / omega_rig (0.13 here:
// delta near 1.26 alpha). At a node a the error is at most delta W sum_p abs(phi_p(a) phi_p(s)) / omega_p^2 <= delta
// W sqrt(C(a, a) C(s, s)) <= delta C_max W (Cauchy-Schwarz over the modes), one delta of the base: at every step
// abs(ratio - max_a C(a, s) / C_max) <= delta + the settled tolerance, s at the step's start. Never flagged.
TEST_CASE("carried validity gate: under gravity a settled loaded member reads its member-wide static ratio at two "
          "steps and two meshes, its window spans the same time at both steps, and a slow move stays within its "
          "derived dynamic term",
          "[ENG-052]") {
    struct Case {
        int segments;
        double h;
    };
    std::vector<double> window_time;
    std::vector<double> window_periods;
    for (const Case c : {Case{8, 1e-3}, Case{8, 1e-4}, Case{16, 1e-3}, Case{16, 1e-4}}) {
        const CarriedRig rig = carried_rig(BeamEnds::SimplySupported, c.segments, 1.0, 0.0, true,
                                           mechsim::engine::test::ParentKind::World, 0.1);
        const double omega_rig = 2.0 / rig.beta;
        const double ei = flexural_rigidity(rig.spec);
        const double length = rig.spec.length;
        const double w = rig.spec.density * rig.spec.area * 9.81;
        const double weight = 9.81;
        const double midspan = (5.0 * w * std::pow(length, 4) / (384.0 * ei)) + (weight * std::pow(length, 3) / (48.0 * ei));
        const double theta = 4.0 * midspan / length;
        double omega_start = 0.0;
        double omega_mid = 0.0;
        {
            const MultibodyProvider probe(rig.model);
            omega_start = lumped_first_frequency(probe, start_state(probe, rig.carriage, 0.25).q);
            omega_mid = lumped_first_frequency(probe, start_state(probe, rig.carriage, 0.5).q);
        }
        const double rate = 0.1 * omega_start * omega_start / omega_rig;
        const double settle = std::log(1e6) / rate;
        mechsim::engine::MultibodyConfig config;
        config.carried_evaluation_start = settle;
        MultibodyProvider provider(rig.model, config);
        State state = start_state(provider, rig.carriage, 0.25);
        SemiImplicitEuler integrator(StepConfig{c.h});
        bool early_zero = true;
        while (state.t + c.h < settle) {
            static_cast<void>(provider.step(integrator, state));
            const MemberOutput& out = provider.outputs().members.front();
            early_zero = early_zero && out.dynamic_ratio == 0.0 && !out.unresolved;
        }
        static_cast<void>(provider.step(integrator, state));
        const MemberOutput first = provider.outputs().members.front();
        const double chain = 1.0 / ei;
        const double chain_max = pinned_chain_largest(c.segments, length);
        const auto expected = [&](double s) { return pinned_chain_member_max(c.segments, s, length) / chain_max; };
        const double local = chain * pinned_chain_compliance(c.segments, 0.25, length) * weight;
        const double g_over_cw = (first.quasi_static_deflection - local) / local;
        const double tolerance = (2.0 * (2.0 + g_over_cw) * 1e-6) + (4.0 * theta * theta);
        const auto window = static_cast<int>(first.window_steps);
        window_time.push_back(window * c.h);
        window_periods.push_back(first.window_periods);
        double settled_error = 0.0;
        bool flagged = false;
        for (int n = 0; n < 2 * window; ++n) {
            static_cast<void>(provider.step(integrator, state));
            const MemberOutput& out = provider.outputs().members.front();
            settled_error = std::max(settled_error, std::abs((out.dynamic_ratio / expected(0.25)) - 1.0));
            flagged = flagged || out.unresolved;
        }
        const double speed = 0.25;
        const double alpha = std::numbers::pi * speed / (length * omega_mid);
        const double zeta_first = 0.1 * omega_start / omega_rig;
        const double dynamic = (alpha * (1.0 + (2.0 * zeta_first))) + (alpha * alpha);
        const Eigen::Index slot = provider.position_offset(rig.carriage);
        state.v[provider.velocity_offset(rig.carriage)] = speed;
        double moving_error = 0.0;
        while (state.q[slot] < 0.75 * length) {
            const double s = state.q[slot];
            static_cast<void>(provider.step(integrator, state));
            const MemberOutput& out = provider.outputs().members.front();
            moving_error = std::max(moving_error, std::abs(out.dynamic_ratio - expected(s)));
            flagged = flagged || out.unresolved;
        }
        const double floor = chain * chain_max * weight;
        std::printf("gravity gate: %d segments, h %.0e: settle %.3f s, window %d steps (%.3f s, %.2f periods), "
                    "floor %.6e m against C_max W %.6e m, g / (C W) %.4f; settled relative error of the ratio "
                    "against max_a C(a, s0) / C_max = %.4f: %.3e within %.3e; moving at %.2f m/s: alpha %.5f, "
                    "zeta_1 %.4f, delta %.5f (%.3f alpha), abs error %.3e within %.3e; flagged %d\n",
                    c.segments, c.h, settle, window, window * c.h, first.window_periods, first.static_floor, floor,
                    g_over_cw, expected(0.25), settled_error, tolerance, speed, alpha, zeta_first, dynamic,
                    dynamic / alpha, moving_error, dynamic + tolerance, static_cast<int>(flagged));
        std::fflush(stdout);
        CHECK(early_zero);
        CHECK(std::abs(first.static_floor - floor) <= 1e-6 * floor);
        CHECK(settled_error <= tolerance);
        CHECK(moving_error <= dynamic + tolerance);
        CHECK_FALSE(flagged);
    }
    for (const std::size_t mesh : {std::size_t{0}, std::size_t{2}}) {
        const double buckets = 4.0 * std::ceil(window_periods[mesh]);
        std::printf("gravity gate window: %.6f s at 1 ms, %.6f s at 0.1 ms, within %.6f s\n", window_time[mesh],
                    window_time[mesh + 1], buckets * 1e-3);
        CHECK(window_periods[mesh] == window_periods[mesh + 1]);
        CHECK(std::abs(window_time[mesh] - window_time[mesh + 1]) < buckets * 1e-3);
    }
}

// The floor over the travel (ruling 76) and the member-wide sample (ruling 77). The same bar under gravity, light node
// damping (0.02 of the critical setting: zeta_1 = 0.02 beta omega_1 / 2), the carriage standing at s0 = 0.05 L (no
// limits: travel = span), a moment M_0 sin(omega_1 t) about the bending axis on the carriage from t = 0, omega_1 the
// loaded first frequency there; evaluated from the release's settle time ln(1e6) / r (r = 0.02 omega_1^2 /
// omega_rig), over two windows. One-mode estimate of the steady resonant ring (phi = sin(pi x / L), m_1 = rho A L / 2
// + m_c phi(s0)^2, k_1 = m_1 omega_1^2, generalized force M_0 phi'(s0), total damping zeta_t = zeta_1 + x / 2 with
// backward Euler's x / 2): at mid-span A = M_0 (pi / L) cos(pi s0 / L) / (2 zeta_t k_1), at the foot A phi(s0). M_0 is
// sized from it for a target mid-span ring A in C_max W.
//   Expected ratio. The sample is the largest nodal abs(d - g): the carried weight's static C(a, s0) W plus the ring A
// phi(a) plus the moment's in-phase quasi-static term, largest near mid-span, so about A / (C_max W) + C(L/2, s0) /
// C_max (0.15 at s0 = 0.05 L) at the ring's crest; the base is the floor (the window's peak, the moment's nodal
// quasi-static deflection near M_0 L^2 / (9 sqrt 3 E I), is near 2 zeta_t A by the sizing, below it: asserted), so
// ratio = A / (C_max W) + 0.15 up to the one-mode estimate's error.
//   (1) A = C_max W: ratio near 1.15, not flagged, though at the foot the ring is A phi(s0) = 4.3 C(s0) W (flagged
// under ruling 74's local floor). The flag fires only if the estimate is 1.6 times low.
//   (2) A = 2.31 C_max W, the mid-span ring that the E7f foot sample let pass (10 C(s0) W at the foot, 0.36 C_max W):
// ratio near 2.46, flagged; the estimate would have to be 1.2 times high to miss.
TEST_CASE("carried validity: a ring at mid-span below the limit of the floor over the travel is not flagged; one past "
          "it is flagged although the foot near a support stays small",
          "[ENG-052]") {
    struct Case {
        double h;
        double target_of_max;  // the mid-span ring's target in C_max W
        bool flag;
    };
    const double s0 = 0.05;
    for (const Case c : {Case{1e-3, 1.0, false}, Case{1e-4, 1.0, false}, Case{1e-3, 2.31, true},
                         Case{1e-4, 2.31, true}}) {
        const CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1.0, 0.0, true,
                                           mechsim::engine::test::ParentKind::World, 0.02);
        const double ei = flexural_rigidity(rig.spec);
        const double length = rig.spec.length;
        const double weight = 9.81;
        const double chain_max = pinned_chain_largest(8, length);
        const double floor = chain_max / ei * weight;
        const double carried_mid = pinned_chain_transfer(8, s0, 0.5 * length, length) / chain_max;
        double omega = 0.0;
        {
            const MultibodyProvider probe(rig.model);
            omega = lumped_first_frequency(probe, start_state(probe, rig.carriage, s0).q);
        }
        const double zeta = (0.02 * rig.beta * omega / 2.0) + (omega * c.h / 2.0);
        const double phi = std::sin(std::numbers::pi * s0 / length);
        const double slope = std::numbers::pi / length * std::cos(std::numbers::pi * s0 / length);
        const double modal_mass = (0.5 * rig.spec.density * rig.spec.area * length) + (phi * phi);
        const double target = c.target_of_max * floor;
        const double moment = target * 2.0 * zeta * modal_mass * omega * omega / slope;
        const double expected = c.target_of_max + carried_mid;
        const double settle = std::log(1e6) / (0.02 * omega * omega * rig.beta / 2.0);
        mechsim::engine::MultibodyConfig config;
        config.carried_evaluation_start = settle;
        MultibodyProvider provider(rig.model, config);
        State state = start_state(provider, rig.carriage, s0);
        SemiImplicitEuler integrator(StepConfig{c.h});
        double largest = 0.0;
        double foot = 0.0;
        double published_floor = 0.0;
        Eigen::Index window = 0;
        MemberOutput last;
        while (window == 0 || state.t < settle + (2.0 * static_cast<double>(window) * c.h)) {
            provider.apply_body_wrench(rig.carriage, Eigen::Vector3d(0.0, 0.0, moment * std::sin(omega * state.t)),
                                       Eigen::Vector3d::Zero());
            static_cast<void>(provider.step(integrator, state));
            last = provider.outputs().members.front();
            if (last.window_steps > 0 && state.t > settle) {
                window = last.window_steps;
                published_floor = last.static_floor;
                largest = std::max(largest, last.relative_deflection);
                foot = std::max(foot, std::abs(provider.outputs().carried.front().deflection));
            }
        }
        std::printf("travel floor, member-wide: h %.0e, mid-span target %.3e m (C_max W %.3e m), M_0 %.4e N m, "
                    "zeta_t %.4f: largest nodal abs(d - g) %.3e m = %.3f C_max W, largest foot deflection %.3f C_max "
                    "W; peak ratio %.4f (expected near %.3f), at s %.4f m, window peak %.3e m, sample %.3e m; flagged "
                    "%d (expected %d)\n",
                    c.h, target, floor, moment, zeta, largest, largest / floor, foot / floor, last.peak_ratio,
                    expected, last.peak_position, last.peak_window_peak, last.peak_relative_deflection,
                    static_cast<int>(last.unresolved), static_cast<int>(c.flag));
        std::fflush(stdout);
        CHECK(std::abs(published_floor - floor) <= 1e-6 * floor);
        CHECK(last.unresolved == c.flag);
        CHECK(last.peak_window_peak < floor);
        CHECK(std::abs(last.peak_position - (s0 * length)) <= 1e-12);
        CHECK(std::abs(last.peak_ratio - (last.peak_relative_deflection / floor)) <= 1e-12 * last.peak_ratio);
        CHECK(foot < carried_ratio_limit * floor);
        if (c.flag) {
            CHECK(largest > carried_ratio_limit * floor);
        } else {
            CHECK(largest <= carried_ratio_limit * floor);
        }
    }
}

// The floor moves the flag time (member_loads.hpp, Definition (c)): a force resonance under gravity. The same bar,
// node damping 0.02 of the critical setting, the 1 kg carriage standing on the load node s = 7 L / 16 (8 segments)
// with a constant upward force equal to its weight, so the rerouted load is the drive alone while the floor still
// counts the carried weight: F = C_max W. From the evaluation start t0 (the release's settle time) the drive F_0
// sin(omega_1 (t - t0)), F_0 = W / 2. As in the resonant test above (collocated: every mode's share positive, s1 >=
// 0.9; omega_1 the loaded frequency there), the deflection at the foot follows c F_0 (s1 g(th) + (1 - s1)(sin th
// +- 0.25)), c = C(s, s), g(th) = (th cos th - sin th) / 2, th = omega_1 (t - t0), with the node damping zeta_1 =
// 0.02 beta omega_1 / 2 and backward Euler's x / 2 taking at most zeta_t th off the growth. The window's peak is the
// drive's nodal quasi-static deflection, at most c F_0 = F / 2 (s is a node of the largest collocated compliance, so
// c = C_max and abs(C(a, s)) <= c), so the base is the floor F throughout and the ratio is the sample over F: the flag
// fires where c F_0 abs(g) first passes 2.04 F, g(th_F) = 2.04 F / (c F_0) = 4.08, not at th_1 (g = 2.04) as
// without the floor: later by (th_F - th_1) / omega_1. The member-wide sample is at least the foot's and, the other
// nodes' first-mode values being at most the foot's, at most it plus the higher modes' (1 - s1)(1 + 0.25) c F_0.
//   upper: abs(g) + (1 - s1) 1.25 with s1 = 0.9 (the earliest the flag can fire),
//   lower: (s1 abs(g) - (1 - s1) 1.25) (1 - zeta_t th) (the latest), both against 2.04 F / (c F_0),
// give or take 3 x in th for the lag and the sampling.
TEST_CASE("carried validity: under gravity a force resonance is flagged where the floor is first passed, later than "
          "without the floor by the header's shift",
          "[ENG-052]") {
    const double h = 1e-4;
    const double s = 7.0 / 16.0;
    const CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1.0, 0.0, true,
                                       mechsim::engine::test::ParentKind::World, 0.02);
    const double length = rig.spec.length;
    const double ei = flexural_rigidity(rig.spec);
    const double weight = 9.81;
    const double drive = 0.5 * weight;
    const double c = pinned_chain_compliance(8, s * length, length) / ei;
    const double floor = pinned_chain_largest(8, length) / ei * weight;
    double omega = 0.0;
    {
        const MultibodyProvider probe(rig.model);
        omega = lumped_first_frequency(probe, start_state(probe, rig.carriage, s).q);
    }
    const double x = omega * h;
    const double zeta = (0.02 * rig.beta * omega / 2.0) + (x / 2.0);
    const double settle = std::log(1e6) / (0.02 * omega * omega * rig.beta / 2.0);
    mechsim::engine::MultibodyConfig config;
    config.carried_evaluation_start = settle;
    MultibodyProvider provider(rig.model, config);
    State state = start_state(provider, rig.carriage, s);
    const double start = std::ceil(settle / h) * h;
    const auto steps = static_cast<int>(std::ceil((settle / h) + (6.0 * 2.0 * std::numbers::pi / x)));
    const Run run = run_forced(provider, state, rig.carriage, steps, h, [&](double t) {
                                   if (t < start - (0.5 * h)) {
                                       return weight;
                                   }
                                   return weight + (drive * std::sin(omega * (t - start)));
                               });
    const auto g = [](double th) { return 0.5 * ((th * std::cos(th)) - std::sin(th)); };
    const double share = 1.0 - first_share_lower;
    const auto crossings = [&](double level) {
        double early = -1.0;
        double late = -1.0;
        for (double th = 0.0; th <= 12.0 * std::numbers::pi && late < 0.0; th += 1e-5) {
            if (early < 0.0 && std::abs(g(th)) + (share * 1.25) > level) {
                early = th;
            }
            if (((first_share_lower * std::abs(g(th))) - (share * 1.25)) * (1.0 - (zeta * th)) > level) {
                late = th;
            }
        }
        return std::array<double, 2>{early, late};
    };
    const std::array<double, 2> shifted = crossings(carried_ratio_limit * floor / (c * drive));
    const std::array<double, 2> unshifted = crossings(carried_ratio_limit);
    const double flagged = omega * (run.flagged_at - start);
    std::printf("force resonance under gravity: x %.6f, zeta_t %.5f, c F_0 / F %.6f; flagged at th %.5f in [%.5f, "
                "%.5f]; without the floor [%.5f, %.5f]; shift at least %.4f s; peak ratio %.4f, floor %.4e m, window "
                "peak at the peak %.4e m\n",
                x, zeta, c * drive / floor, flagged, shifted[0], shifted[1], unshifted[0], unshifted[1],
                (shifted[0] - unshifted[1]) / omega, run.last.peak_ratio, run.last.static_floor,
                run.last.peak_window_peak);
    REQUIRE(run.last.unresolved);
    CHECK(std::abs(run.last.static_floor - floor) <= 1e-6 * floor);
    CHECK(run.last.peak_window_peak <= 0.5 * floor * (1.0 + 1e-6));
    CHECK(shifted[0] > unshifted[1]);
    CHECK(flagged >= shifted[0] - (3.0 * x));
    CHECK(flagged <= shifted[1] + (3.0 * x));
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
