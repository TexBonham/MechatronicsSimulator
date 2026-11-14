#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <numbers>
#include <vector>

#include "energy_helpers.hpp"
#include "mechsim/engine/engine_error.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/multibody_model.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/rope.hpp"
#include "rope_helpers.hpp"

using mechsim::engine::DynamicsTerms;
using mechsim::engine::MultibodyConfig;
using mechsim::engine::MultibodyModel;
using mechsim::engine::MultibodyProvider;
using mechsim::engine::Rope;
using mechsim::engine::SemiImplicitEuler;
using mechsim::engine::SolverNonConvergenceError;
using mechsim::engine::State;
using mechsim::engine::StepConfig;
using mechsim::engine::test::add_bob;
using mechsim::engine::test::gravity;
using mechsim::engine::test::mechanical_energy;

// A rope segment going taut inside a step (ENG-047, brief M3-E8, ruling 87; the rule and its convergence argument in
// rope.hpp, Taut set). Rig: a point load m = 1 kg on a Translation joint hung from the world origin by a vertical
// rope of rest length L = 1 m, EA = 500 kN (series stiffness k = EA / L = 5e5 N/m), mu = 0.01 kg/m (rope mass
// mu L = 0.01 kg, 1 % of m), N = 1 or 8 segments; everything stays on the vertical, so the problem is one-dimensional.
//
// Scheme value (backward Euler on one degree of freedom). With mass M on stiffness k under gravity, x = omega h,
// omega = sqrt(k / M), the step maps the scaled state (omega z, v), z = e - e_s the stretch e past its static value
// e_s = M g / k, by rho R(theta): rho = (1 + x^2)^(-1/2), theta = atan x, so
//   z_j = rho^j (z_0 cos(j theta) + (v_0 / omega) sin(j theta)), tension T_j = k (e_s + z_j).
// Under the rule a segment slack at a step's start that ends it at or past its rest length is taut in that step with
// its spring (stretch from rest length, affine: k (e_0 + h v)) and without its damper, so the step in which the rope
// goes taut is already this map, from the state at that step's start. A load released at rest length (gap d = 0)
// starts the map at (z_0, v_0) = (-e_s, 0): T_j = M g (1 - rho^j cos(j theta)), at most 2 M g. A load dropped from
// d above the rest-length point falls free for N - 1 steps, N the least n >= 1 with g h^2 n (n + 1) / 2 >= d
// (backward Euler from rest: v_n = n g h, fallen g h^2 n (n + 1) / 2), and the map starts at the start of step N:
// e_0 = -d + g h^2 (N - 1) N / 2, v_0 = (N - 1) g h. For one segment the load carries M = m + mu L / 2 (its rope
// share) and the engine is this map exactly, up to round-off (1e-9 relative asserted).
//
// Physical bounds (the energy oracle: kinetic plus gravitational plus elastic energy never increases under the
// scheme, asserted step by step below). A drop d onto a slack rope: (1/2) k e^2 <= M g (d + e) at any later time, so
// T = k e <= M g (1 + sqrt(1 + 2 d k / (M g))); d = 0 gives 2 M g. A load at static stretch given an upward speed u:
// (1/2) k e^2 - M g e <= (1/2) M u^2 - (1/2) k e_s^2, so T <= M g + u sqrt(M k), however long the rope is slack.
// Eight segments: the interior node masses mu L / 8 on segment stiffness 8 k have x = sqrt(16 k / (mu L / 8)) h >= 5.6
// at 0.1 ms, so the scheme removes at least 1 - rho^2 > 0.96 of their energy per step; the chain acts as the series
// spring k carrying the load and the node weights. Its modes other than the load's carry at most the rope's own
// weight mu L g statically, and a step response at most doubles a static share, so the bounds hold with M = m + mu L
// plus 2 mu L g, and the scheme value with M = m less 2 mu L g is a floor.
// Speed at going taut: v_{N-1} = (N - 1) g h < sqrt(2 g d) (step N - 1 had not reached the rest length) and
// v_N = N g h >= sqrt(2 g d + (g h / 2)^2) - g h / 2 (step N reached it), so abs(v_{N-1} - sqrt(2 g d)) <= 1.5 g h.

namespace {

constexpr double load_mass = 1.0;
constexpr double rest = 1.0;
constexpr double axial = 5e5;
constexpr double mu = 0.01;
constexpr double stiffness = axial / rest;
// Energies here are at most about 10 J (m g L); each step's evaluation carries about 1e-15 relative (test_energy.cpp).
constexpr double round_off = 1e-12;

struct Rig {
    MultibodyModel model;
    Eigen::Index bob = -1;
    std::vector<Eigen::Index> nodes;
};

// The load at depth rest - gap below the origin, the chain's nodes evenly between, at rest.
Rig rig(int segments, double gap, double damping) {
    Rig out;
    out.model.gravity = Eigen::Vector3d(0.0, 0.0, -gravity);
    const Eigen::Vector3d at(0.0, 0.0, -(rest - gap));
    out.bob = add_bob(out.model, at, load_mass);
    Rope rope;
    rope.to_body = out.bob;
    rope.segments = segments;
    rope.axial_rigidity = axial;
    rope.damping = damping;
    rope.mass_per_length = mu;
    rope.rest_length = rest;
    static_cast<void>(add_rope(out.model, rope, Eigen::Vector3d::Zero(), at));
    out.nodes = out.model.ropes[0].nodes;
    return out;
}

// The chain hanging at its static stretch: segment s (from the top) carries g (m + mu l / 2 + (N - 1 - s) mu l).
void hang_static(const Rig& r, const MultibodyProvider& provider, State& state) {
    const auto n = static_cast<int>(r.nodes.size()) + 1;
    const double l = rest / n;
    double depth = 0.0;
    for (int s = 0; s < n; ++s) {
        const double weight = gravity * (load_mass + (0.5 * mu * l) + ((n - 1 - s) * mu * l));
        depth += l + (weight / (axial / l));
        Eigen::Index body = r.bob;
        if (s < n - 1) {
            body = r.nodes[static_cast<std::size_t>(s)];
        }
        const double built = r.model.bodies[static_cast<std::size_t>(body)].joint.position.z();
        state.q[provider.position_offset(body) + 2] = -depth - built;
    }
}

struct Run {
    double peak = 0.0;              // largest segment tension at a step's end, N
    double taut_speed = -1.0;       // load speed at the start of the first step that ends with tension, m/s
    double worst_rise = -1.0;       // largest step-to-step rise of the mechanical energy, J
    bool went_slack = false;        // a step after the first loaded one ended with a segment slack
    bool ends_taut = false;         // the last step ended with every segment taut
    std::vector<double> history;    // q and v after every step
};

Run run(MultibodyProvider& provider, State& state, double h, int steps) {
    SemiImplicitEuler integrator(StepConfig{h});
    Run out;
    bool loaded = false;
    double energy = mechanical_energy(provider, state);
    const Eigen::Index z = provider.velocity_offset(provider.model().ropes[0].to_body) + 2;
    for (int n = 0; n < steps; ++n) {
        const double speed = -state.v[z];
        static_cast<void>(provider.step(integrator, state));
        const std::vector<double>& tension = provider.outputs().ropes[0].tension;
        const double top = *std::max_element(tension.begin(), tension.end());
        const double low = *std::min_element(tension.begin(), tension.end());
        out.peak = std::max(out.peak, top);
        if (!loaded && top > 0.0) {
            loaded = true;
            out.taut_speed = speed;
        }
        if (loaded && low == 0.0) {
            out.went_slack = true;
        }
        out.ends_taut = low > 0.0;
        const double next = mechanical_energy(provider, state);
        out.worst_rise = std::max(out.worst_rise, next - energy);
        energy = next;
        out.history.insert(out.history.end(), state.q.begin(), state.q.end());
        out.history.insert(out.history.end(), state.v.begin(), state.v.end());
    }
    return out;
}

// The scheme's peak tension for mass on k after a drop of gap from rest (header, Scheme value).
double scheme_peak(double mass, double gap, double h) {
    int first = 1;
    while (gravity * h * h * first * (first + 1) / 2.0 < gap) {
        ++first;
    }
    const double e0 = -gap + (gravity * h * h * (first - 1) * first / 2.0);
    const double v0 = (first - 1) * gravity * h;
    const double omega = std::sqrt(stiffness / mass);
    const double x = omega * h;
    const double rho = 1.0 / std::sqrt(1.0 + (x * x));
    const double theta = std::atan(x);
    const double es = mass * gravity / stiffness;
    const double z0 = e0 - es;
    double peak = 0.0;
    const auto count = static_cast<int>(std::ceil(4.0 * std::numbers::pi / theta));
    for (int j = 1; j <= count; ++j) {
        const double zj = std::pow(rho, j) * ((z0 * std::cos(j * theta)) + ((v0 / omega) * std::sin(j * theta)));
        peak = std::max(peak, stiffness * (es + zj));
    }
    return peak;
}

double drop_bound(double mass, double gap) {
    return mass * gravity * (1.0 + std::sqrt(1.0 + (2.0 * gap * stiffness / (mass * gravity))));
}

int steps_for(double h, double seconds) { return static_cast<int>(std::ceil(seconds / h)); }

}  // namespace

TEST_CASE("a rope released at its rest length carries at most twice the weight it holds", "[ENG-047]") {
    for (const int segments : {1, 8}) {
        for (const double h : {1e-3, 1e-4}) {
            CAPTURE(segments, h);
            const Rig r = rig(segments, 0.0, 0.0);
            MultibodyProvider provider(r.model);
            State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
            const Run out = run(provider, state, h, steps_for(h, 0.05));
            const double share = mu * rest / (2.0 * segments);
            if (segments == 1) {
                const double scheme = scheme_peak(load_mass + share, 0.0, h);
                const double bound = 2.0 * (load_mass + share) * gravity;
                std::printf("ENG-047 release at rest length, N = 1, h = %g: peak %.9g N, scheme %.9g N, bound 2 M g "
                            "%.9g N\n", h, out.peak, scheme, bound);
                CHECK(std::abs(out.peak / scheme - 1.0) <= 1e-9);
                CHECK(out.peak <= bound);
            } else {
                const double floor = scheme_peak(load_mass, 0.0, h) - (2.0 * mu * rest * gravity);
                const double bound = 2.0 * (load_mass + (mu * rest)) * gravity;
                std::printf("ENG-047 release at rest length, N = 8, h = %g: peak %.9g N in [%.9g, %.9g] N\n", h,
                            out.peak, floor, bound);
                CHECK(out.peak >= floor);
                CHECK(out.peak <= bound);
            }
            CHECK(out.worst_rise <= round_off);
            CHECK(out.ends_taut);
        }
    }
}

TEST_CASE("a load dropped onto a slack rope peaks within the drop bracket", "[ENG-047]") {
    for (const double gap : {0.0123, 0.0471}) {
        for (const double h : {1e-3, 1e-4}) {
            CAPTURE(gap, h);
            const Rig r = rig(1, gap, 0.0);
            MultibodyProvider provider(r.model);
            State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
            const Run out = run(provider, state, h, steps_for(h, 0.15));
            const double mass = load_mass + (0.5 * mu * rest);
            const double scheme = scheme_peak(mass, gap, h);
            const double bound = drop_bound(mass, gap);
            const double speed = std::sqrt(2.0 * gravity * gap);
            std::printf("ENG-047 drop %.4f m, N = 1, h = %g: peak %.9g N, scheme %.9g N, bound %.9g N; speed at taut "
                        "%.6f m/s, sqrt(2 g d) %.6f m/s\n", gap, h, out.peak, scheme, bound, out.taut_speed, speed);
            CHECK(std::abs(out.peak / scheme - 1.0) <= 1e-9);
            CHECK(out.peak <= bound);
            CHECK(std::abs(out.taut_speed - speed) <= 1.5 * gravity * h);
            CHECK(out.worst_rise <= round_off);
        }
    }
    for (const double h : {1e-3, 1e-4}) {
        CAPTURE(h);
        const double gap = 0.0123;
        const Rig r = rig(8, gap, 0.0);
        MultibodyProvider provider(r.model);
        State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
        const Run out = run(provider, state, h, steps_for(h, 0.15));
        const double bound = drop_bound(load_mass + (mu * rest), gap) + (2.0 * mu * rest * gravity);
        std::printf("ENG-047 drop %.4f m, N = 8, h = %g: peak %.9g N, bound %.9g N\n", gap, h, out.peak, bound);
        CHECK(out.peak <= bound);
        CHECK(out.worst_rise <= round_off);
    }
}

TEST_CASE("a hanging load lifted until its rope unloads reloads it without a blow", "[ENG-047]") {
    // Upward speed u = 0.3 m/s from the static stretch: the load rises u^2 / (2 g) = 4.6 mm, far past the static
    // stretch (2e-5 m), so the rope goes slack, and falls back onto it. Undamped, the bracket (header) bounds the
    // tension. Damped (0.2 sqrt(EA m L), the load's axial mode at 10 % of critical; the damper engaged in the steps
    // that start taut), a segment's published tension adds c d_dot, which the energy bound does not cover (a
    // Kelvin-Voigt segment pulls c v the instant it is struck at v), so that run asserts the energy oracle and the
    // set's return only.
    const double lift = 0.3;
    for (const double damping : {0.0, 0.2 * std::sqrt(axial * load_mass * rest)}) {
        for (const int segments : {1, 8}) {
            for (const double h : {1e-3, 1e-4}) {
                CAPTURE(damping, segments, h);
                const Rig r = rig(segments, 0.0, damping);
                MultibodyProvider provider(r.model);
                State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
                hang_static(r, provider, state);
                state.v[provider.velocity_offset(r.bob) + 2] = lift;
                const Run out = run(provider, state, h, steps_for(h, 0.6));
                double bound = 0.0;
                if (segments == 1) {
                    const double mass = load_mass + (0.5 * mu * rest);
                    bound = (mass * gravity) + (lift * std::sqrt(mass * stiffness));
                } else {
                    const double mass = load_mass + (mu * rest);
                    bound = (mass * gravity) + (lift * std::sqrt(mass * stiffness)) + (2.0 * mu * rest * gravity);
                }
                std::printf("ENG-047 lift %.2f m/s, damping %.4g N s, N = %d, h = %g: peak %.9g N, bound %.9g N, "
                            "slack %d, ends taut %d\n", lift, damping, segments, h, out.peak, bound,
                            static_cast<int>(out.went_slack), static_cast<int>(out.ends_taut));
                CHECK(out.went_slack);
                CHECK(out.ends_taut);
                if (damping == 0.0) {
                    CHECK(out.peak <= bound);
                }
                CHECK(out.worst_rise <= round_off);
            }
        }
    }
}

TEST_CASE("a rope going taut within a step is deterministic", "[ENG-047][ENG-023]") {
    const auto once = [] {
        const Rig r = rig(8, 0.0123, 1.0);
        MultibodyProvider provider(r.model);
        State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
        return run(provider, state, 1e-3, 150).history;
    };
    const std::vector<double> first = once();
    REQUIRE(first == once());
}

// Rope.hpp, Taut set: at its rest length a segment is in the set, with its spring from rest length (force 0, K's
// axial block k) and no damper (its law is not taut at the start), so the first step already holds the load.
TEST_CASE("a rope segment at its rest length is taut within the step, spring only", "[ENG-047]") {
    const Rig r = rig(1, 0.0, 5.0);
    MultibodyProvider provider(r.model);
    const State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    DynamicsTerms terms;
    provider.evaluate(0.0, state.q, state.v, terms);
    const Eigen::Index z = provider.velocity_offset(r.bob) + 2;
    REQUIRE(terms.stiffness(z, z) == stiffness);
    REQUIRE(terms.stiffness.norm() == stiffness);
    REQUIRE(terms.damping.norm() == 0.0);
    REQUIRE(terms.force.norm() == 0.0);
}

// Bit-identity of a rope that stays taut (rope.hpp, Taut set): its start set is consistent with every step's end,
// so each step is the one pass the start-of-step rule took. The ACC-037 form at 1 ms and a heavier rope, 2000 steps.
TEST_CASE("a rope that stays taut settles its taut set in one pass every step", "[ENG-047]") {
    for (const double ratio : {1e-3, 0.1}) {
        CAPTURE(ratio);
        mechsim::engine::test::RopePendulum p = mechsim::engine::test::rope_pendulum(8, 1.0, ratio, 0.3, 0.0);
        MultibodyProvider provider(p.model);
        SemiImplicitEuler integrator(StepConfig{1e-3});
        State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
        int most = 0;
        for (int n = 0; n < 2000; ++n) {
            static_cast<void>(provider.step(integrator, state));
            most = std::max(most, provider.outputs().rope_passes);
        }
        REQUIRE(most == 1);
    }
}

// The set's iteration cap is the run configuration's iteration cap (ENG-025). A drop needs two passes in the step
// the rope goes taut (slack from the start, inconsistent with its end; taut, consistent), every step before it one
// (the load then rebounds past rest length and lands again, two passes each time); with
// the cap at 1 that step ends the run with a solver non-convergence error and leaves the state as it was.
TEST_CASE("a rope taut set that does not settle within the iteration cap ends the run", "[ENG-047][ENG-025]") {
    const Rig r = rig(1, 0.0123, 0.0);
    const double h = 1e-3;
    MultibodyProvider provider(r.model);
    SemiImplicitEuler integrator(StepConfig{h});
    State state{provider.neutral_positions(), Eigen::VectorXd::Zero(provider.velocity_size()), 0.0};
    std::vector<int> passes;
    for (int n = 0; n < 100; ++n) {
        static_cast<void>(provider.step(integrator, state));
        passes.push_back(provider.outputs().rope_passes);
    }
    const auto contact = std::find(passes.begin(), passes.end(), 2) - passes.begin();
    std::printf("ENG-047 drop: the rope goes taut in step %d (two passes)\n", static_cast<int>(contact) + 1);
    REQUIRE(contact < 100);
    REQUIRE(std::count(passes.begin(), passes.begin() + contact, 1) == contact);
    REQUIRE(*std::max_element(passes.begin(), passes.end()) == 2);

    MultibodyConfig capped;
    capped.iteration_cap = 1;
    MultibodyProvider limited(r.model, capped);
    State run_state{limited.neutral_positions(), Eigen::VectorXd::Zero(limited.velocity_size()), 0.0};
    for (Eigen::Index n = 0; n < contact; ++n) {
        static_cast<void>(limited.step(integrator, run_state));
    }
    const State held = run_state;
    REQUIRE_THROWS_AS(limited.step(integrator, run_state), SolverNonConvergenceError);
    REQUIRE(run_state.q == held.q);
    REQUIRE(run_state.v == held.v);
    REQUIRE(run_state.t == held.t);
}
