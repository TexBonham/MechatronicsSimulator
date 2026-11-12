#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

#include "mechsim/engine/member_loads.hpp"
#include "mechsim/engine/multibody_model.hpp"

namespace mechsim::engine {

// Lumped flexible member (ENG-022, ADR 0013): N rigid segments of length l = L / N, each with its rotational spring
// k = E I / l at the segment midpoint, so the spring nodes sit at x_j = (j + 1/2) l and the rigid bodies span
// node to node. Clamped root: the stub [0, l/2] is part of the parent. Pinned root: a free revolute at x = 0. The
// far end is free (Cantilever), pinned on a roller (SimplySupported), or clamped on a roller (Clamped, ruling 16):
// the roller's point constraint becomes a Hinge about the beam axis x, two rows more, locking the last segment's
// rotation about z (and y, a dependent row for the planar chain) to the roller, so the far stub [L - l/2, L] moves
// with the roller as the root stub does with the parent. The roller keeps the axial translation free: a fully
// fixed far end would lock the rigid-segment chain at second order in the deflection, and the closed forms below
// carry no axial force. Planar bending about the root frame's z axis only; two-axis bending plus torsion is the
// next tier (ADR 0001, invariant 6).
//
// Static discretization error, derived. A hinge chain deflects at a under a load P at b by
//   delta = sum_j M(x_j) m(x_j) / k_j    (unit-load method; M from P, m from a unit load at a),
// the continuum by delta = integral M m / (E I) dx. With one spring per segment at its midpoint and k = E I / l the
// chain is the midpoint rule of that integral, exact for constant M m and short by l^3 M' m' / (12 E I) per segment
// where M m is linear.
//   Cantilever, tip load: M = P (L - x), m = L - x, so delta = P l^3 / (E I) sum_{j<N} (j + 1/2)^2
//     = P L^3 / (3 E I) (1 - 1 / (4 N^2)): N = 4 gives 21/64 P L^3 / (E I), -1.5625 %; N = 8 gives -0.390625 %.
//     (Springs at segment starts j l instead: (N + 1)(2N + 1) / (6 N^2) P L^3 / (E I), 30/64 at N = 4, +40.6 %.)
//   Simply supported, midspan load, even N: M = (P / 2) min(x, L - x), so each half is a cantilever of N / 2
//     segments carrying P / 2: delta = P L^3 / (48 E I) (1 - 1 / N^2): N = 4 gives -6.25 %, N = 8 gives -1.5625 %.
//     (Springs at interior segment ends, the trapezoid rule: +2 / N^2, +12.5 % at N = 4.)
//   Clamped-clamped, midspan load, even N = 2n: the chain is symmetric (springs at (j + 1/2) l, j < N), so M(x) =
//     M0 + (P / 2) min(x, L - x) and the far rotation lock sum_j M(x_j) = 0 gives M0 = -(P / 2) l n / 2 = -P L / 8,
//     the continuum value. Midspan deflection sum_{j<n} theta_j (L/2 - x_j) with theta_j = M(x_j) l / (E I), a = j +
//     1/2: sum (a - n/2)(n - a) = -(n^3 - n) / 12, so delta = P l^3 (n^3 - n) / (24 E I) = P L^3 / (192 E I) (1 -
//     4 / N^2): N = 8 gives -6.25 %, outside the 2 % tolerance; N = 16 gives -1.5625 %. First mode (chain
//     eigenproblem, rods integrated exactly, null space of the two end rows): 22.373^2 scale times 1.10789 at N = 4,
//     1.024865 at N = 8, 1.006091 at N = 16 (closed form beta_1 L = 4.730041, cos x cosh x = 1). Clamped-clamped
//     members therefore default to N = 16 for the 2 % references; a calibrated clamped-clamped member below 16 is a
//     build error (ruling 32).
// Mass: each body is a uniform rod of its span s, m = rho A s, I_zz = m s^2 / 12 about its center (Euler-Bernoulli:
// no section rotary inertia in the bending plane).
enum class BeamEnds : std::uint8_t { Cantilever, SimplySupported, Clamped };

struct BeamSpec {
    static constexpr int default_segments = 4;                    // ADR 0013; docs/spec/eng.md, Open parameters
    static constexpr double default_roller_mass_fraction = 1e-3;  // docs/spec/eng.md, Open parameters
    // Clamped far end floor (ruling 31): N <= 2 is rigid; the chain's deflection error is -4 / N^2.
    static constexpr int min_clamped_segments = 8;

    double youngs_modulus = 0.0;      // E, Pa, finite and > 0
    double area_moment = 0.0;         // I about the bending axis, m^4, finite and > 0
    double density = 0.0;             // rho, kg/m^3, finite and > 0
    double area = 0.0;                // A, m^2, finite and > 0
    double length = 0.0;              // L, m, finite and > 0
    int segments = default_segments;  // N >= 1
    double node_damping = 0.0;        // per spring node, N m s/rad, finite and >= 0
    BeamEnds ends = BeamEnds::Cantilever;
    double roller_mass_fraction = default_roller_mass_fraction;  // roller mass / member mass, > 0 (roller ends)
};

// A material point of the beam: the body holding it (the parent on a clamped stub) and the point in that frame, m.
struct BeamPoint {
    Eigen::Index body = -1;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
};

// Appends the beam to model along +x of a root frame placed at position, orientation in the parent body (-1 is the
// world), and registers it in model.members (ENG-030). The roller of a SimplySupported or Clamped beam slides along
// the root x axis at x = L with roller_mass_fraction of the member's mass: a roller is massless, and its axial motion
// is second order in the deflection. Throws InvalidModelError.
LumpedBeam add_lumped_beam(MultibodyModel& model, const BeamSpec& spec, Eigen::Index parent = -1,
                           const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
                           const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity());

// The material point at x along the beam, 0 <= x <= L; throws InvalidModelError otherwise.
[[nodiscard]] BeamPoint beam_point(const LumpedBeam& beam, double x);

}  // namespace mechsim::engine
