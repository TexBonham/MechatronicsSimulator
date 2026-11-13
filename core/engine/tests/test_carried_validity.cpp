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
// 1 kg carriage at midspan on the member axis (no moment arm), loaded by a force along y applied to the carriage each
// step; its joint passes the force to the member one step late. The member's first mode at midspan: the continuum
// share of the collocated static compliance is 96 / pi^4 = 0.9855 (pinned-pinned modes sin(n pi x / L), odd n only at
// midspan, shares 1 / n^4 over sum 1 / n^4); the chain's differs by its frequency error (2.5 % at 8 segments, the
// share goes as omega^-2) and its static error (2 %, ENG-022): s1 >= 0.9 is taken as the bound. The next mode with a
// share at midspan is n = 3, omega_3 / omega_1 near 9. omega_1 is the chain's own (lumped_first_frequency of the bare
// chain, the carriage's mass sits on the world).

namespace {

constexpr double first_share_lower = 0.9;

CarriedRig loaded_rig(double damping_scale) {
    CarriedRig rig = carried_rig(BeamEnds::SimplySupported, 8, 1.0, 0.0, true, mechsim::engine::test::ParentKind::World,
                                 damping_scale);
    rig.model.gravity = Eigen::Vector3d::Zero();
    return rig;
}

double chain_first_frequency(const CarriedRig& rig) {
    MultibodyModel bare;
    bare.gravity = Eigen::Vector3d::Zero();
    static_cast<void>(add_lumped_beam(bare, rig.spec, -1));
    const MultibodyProvider provider(bare);
    return lumped_first_frequency(provider);
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

namespace {

// The finding's shape (ruling 69): member A on the world carries a frame T (fixed to the world at A's midspan, on
// A's axis); member B, an identical bar, hangs on T 5 cm below A's axis and carries P (1 kg, prismatic along B, on
// A's axis height, 5 cm above B's). Both undamped, equal chains, so equal frequencies; gravity off. P runs at 1 m/s
// and a damper brakes it (time constant 1 / omega_1): the braking force along B, 5 cm above B's axis, bends B, and
// B's ringing reaches A only through T's reaction, at A's own frequency, never the other way.
struct TwoMembers {
    MultibodyModel model;
    Eigen::Index frame = -1;
    Eigen::Index carriage = -1;
    double omega = 0.0;
};

TwoMembers two_members() {
    TwoMembers rig;
    rig.model.gravity = Eigen::Vector3d::Zero();
    const mechsim::engine::BeamSpec spec = mechsim::engine::test::steel_bar(1.0, BeamEnds::SimplySupported, 8);
    static_cast<void>(add_lumped_beam(rig.model, spec, -1));
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
    rig.model.members.front().carried.push_back(rig.frame);
    rig.model.members.back().carried.push_back(rig.carriage);
    MultibodyModel bare;
    bare.gravity = Eigen::Vector3d::Zero();
    static_cast<void>(add_lumped_beam(bare, spec, -1));
    rig.omega = lumped_first_frequency(MultibodyProvider(bare));
    return rig;
}

struct TwoRun {
    MemberOutput carrying;
    MemberOutput carried;
};

TwoRun run_two(double h, double duration) {
    const TwoMembers rig = two_members();
    MultibodyProvider provider(rig.model);
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    state.q[provider.position_offset(rig.carriage)] = 0.3;
    state.v[provider.velocity_offset(rig.carriage)] = 1.0;
    SemiImplicitEuler integrator(StepConfig{h});
    const double brake = 0.02;
    const auto steps = static_cast<int>(std::ceil(duration / h));
    for (int n = 0; n < steps; ++n) {
        if (n == static_cast<int>(std::lround(brake / h))) {
            provider.set_joint_drive(rig.carriage, 0, 0.0, 0.0, rig.omega * 1.0);
        }
        static_cast<void>(provider.step(integrator, state));
    }
    return TwoRun{provider.outputs().members.front(), provider.outputs().members.back()};
}

}  // namespace

TEST_CASE("carried validity: a member driven by another carried member at its own frequency is flagged",
          "[ENG-052]") {
    // A's load is B's ringing at A's own frequency, with no back-reaction: resonant growth, ratio ~ omega t / 2 times
    // the ringing's share of A's largest load, so it passes 2.04 within the twenty periods run. W_A > 0: the pair
    // pumps energy into A that B never loses.
    const TwoMembers rig = two_members();
    const double h = 1e-4;
    const double duration = 20.0 * 2.0 * std::numbers::pi / rig.omega;
    const TwoRun run = run_two(h, duration);
    std::printf("two members: A (carrying) work %.6e J, peak ratio %.4f, flagged %d at %.4f s; B work %.6e J, peak "
                "ratio %.4f, flagged %d\n",
                run.carrying.carried_work, run.carrying.peak_ratio, static_cast<int>(run.carrying.unresolved),
                run.carrying.unresolved_time, run.carried.carried_work, run.carried.peak_ratio,
                static_cast<int>(run.carried.unresolved));
    CHECK(run.carrying.unresolved);
    CHECK(run.carrying.unresolved_time > 0.02);
    CHECK(run.carrying.carried_work > 0.0);
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
