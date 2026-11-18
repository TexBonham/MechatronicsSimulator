#pragma once

#include <Eigen/Core>
#include <limits>
#include <vector>

namespace mechsim::engine {

struct MultibodyModel;

// Rope as a lumped chain (ENG-047, PHY-004, ADR 0016 option A).
//
// State. N segments of rest length l = rest_length / N; N - 1 interior nodes, each a point mass mass_per_length * l
// on its own Translation joint to the world (3 DOF, no rotation: a point mass has no use for one, and the nodes
// stay ordinary bodies, so H, constraints, outputs and the integrator need nothing rope-specific). The two end
// half-masses mass_per_length * l / 2 are added to from_body and to_body at their attach points (the world takes
// none), so the rope mass is conserved.
//
// Segment force. d the segment length, n the unit vector from its upper end a (towards from_body) to its lower end b:
//   T = (EA / l)(d - l) + (damping / l) d_dot when d > l and that sum is positive; else T = 0 (slack),
// T n on a and -T n on b. This law is what RopeOutput publishes at the end state; within a step a segment pulls by the
// taut set below. Implicit treatment (ADR 0013): a segment taut by the law at the step's start adds
// J^T (k n n^T + (T / d)(I - n n^T)) J to K and J^T (c n n^T) J to D, J = J_b - J_a the relative point Jacobian; the
// geometric term (T / d) carries the transverse chain modes.
//
// Taut set (brief M3-E8, ruling 87). A segment's state within a step is consistent with the step's end state: it is
// taut exactly when its end-of-step length is at or above its rest length (equality taut, so a rope at its rest length
// holds its load from the first step). A segment in the step's set pulls
//   - with the law's T, spring and damper, terms as above, when the law is taut at the step's start (d > l, T > 0);
//   - else (it goes taut within the step, or is past rest length while its damper reads slack) with its spring alone,
//     k (d - l) from rest length, affine (negative while it starts short of rest length, so its end force is k times
//     its end stretch: a segment that ends the step just past rest length carries a small force, not the accumulated
//     fall), adding k n n^T, plus (k (d - l) / d)(I - n n^T) when d > l, to K and nothing to D: the damper acts only
//     in steps that start taut. A segment struck at speed v publishes the law's c v at the step's end; the next step
//     applies it.
// MultibodyProvider::step solves with the set read from the start-of-step lengths, reads the set from the end-of-step
// lengths, and re-solves from the same start with that set until two agree (every inconsistent segment flips at once,
// rope and segment order: deterministic). The cap is MultibodyConfig::iteration_cap solves (ENG-025, 50); a set still
// changing at the cap throws SolverNonConvergenceError and leaves the state as it was. A rope that stays taut agrees
// at the first solve, which is the start-of-step rule's step bit for bit. The top attachment's torque reads the top
// segment's start-of-step direction and the swing rate, not the set, so slack or taut it acts as before; a rest length
// set mid-run applies from the next step and the set is judged against it, so a winch needs no special case.
// Convergence. One segment, along its line, mass m, start stretch e0 = d - l, speed v0, free acceleration g: the slack
// solve ends at e_S = e0 + h (v0 + h g); taut with the spring alone (e0 < 0), at e_T = m e_S / (m + h^2 k), the sign of
// e_S; taut with the damper (e0 > 0), at e_T = (m e_S + h c e0) / (m + h c + h^2 k), at or above 0 whenever e_S is, and
// below 0 only if e_S is. Either way the second solve agrees: at most two. (A damper on a segment starting slack would
// give e_T < 0 <= e_S when m e_S < -h c e0, a two-cycle; that is why it waits for a step that starts taut.) A chain
// whose segments meet at less than 90 degrees: the end stretches are e = e_free - h^2 W K e_+, W = J M^-1 J^T
// tridiagonal with positive diagonal and off-diagonal -n_i . n_(i+1) / m_node <= 0, so the set solves a linear
// complementarity problem with the M-matrix I + h^2 W K (with dampers, k + c / h for the start-taut ones); flipping
// every inconsistent segment at once (the primal-dual active set method) is monotone from the second solve and ends
// within n + 2 solves for n segments in the model (Hintermueller, Ito and Kunisch, SIAM J. Optim. 13, 2003): 10 for
// an 8 segment rope, under the cap. Bends past 90 degrees, other degrees of freedom and the exact end lengths are
// outside that proof; there the cap stands and reaching it ends the run.
//
// Attachment (round 5 ruling 14): the top segment hangs from from_body through a damped spherical joint. With
// omega_rel = P (n x (v_b - v_a) / d - omega_from), P = I - n n^T, the joint torque on the segment is
// tau = -(attachment_damping + attachment_friction / |omega_rel|) omega_rel (Coulomb friction as a damping frozen at
// the start-of-step rate, zero at zero rate: static friction is the next tier); it acts as (tau x n) / d on b,
// -(tau x n) / d at a and -tau on from_body, and adds its coefficient times A^T A to D, A the map v -> omega_rel.
// A small-swing pendulum on it decays with the logarithmic decrement of I theta'' + c theta' + g S theta = 0.
//
// Snap (ENG-034). When a segment's end-of-step tension exceeds breaking_tension, the last segment (to to_body) is
// removed from the next step on; the chain stays hung from from_body. Only from MultibodyConfig::
// rope_snap_evaluation_start (INT-031, ruling 75; counted in steps: the first judged step is n = ceil((start - t_0) /
// h - 1e-9) after load): in a step starting before it the exceedance is published as
// RopeOutput::held_tension and the load stays attached.
//
// Snatch resolution (rulings 15 and 22, ADR 0016 Consequences): the rope's fundamental with the load,
// omega_s = sqrt(EA / (rest_length M)), M the to_body mass; above omega_s h = 0.2 the first snatch peak is about
// 15 % low (implicit Euler), published as a flag the adapter records as an approximation line.
// Swing (ruling 21): the swing mode runs through K, so backward Euler removes (1 + x^2)^(-pi / atan x) of its
// amplitude per period (about pi x) and runs x^2 / 3 slow, x = omega h; published as scheme_decay = pi omega h with
// omega = sqrt(|g| / d), d the summed segment lengths.
struct Rope {
    static constexpr int default_segments = 8;                // ADR 0016
    static constexpr double snatch_resolution_limit = 0.2;    // omega_s h, round 5 ruling 15

    Eigen::Index from_body = -1;                                // -1 is the world
    Eigen::Vector3d from_point = Eigen::Vector3d::Zero();       // from_body frame (world for -1), m
    Eigen::Index to_body = -1;                                  // -1 is the world
    Eigen::Vector3d to_point = Eigen::Vector3d::Zero();         // to_body frame (world for -1), m
    int segments = default_segments;                            // N >= 1
    double axial_rigidity = 0.0;                                // EA, N, finite and > 0
    double damping = 0.0;                                       // axial force per strain rate, N s, finite, >= 0
    double mass_per_length = 0.0;                               // kg/m, finite and > 0
    double rest_length = 0.0;                                   // m, finite and > 0; changeable per step
    double breaking_tension = std::numeric_limits<double>::infinity();  // N, > 0; infinity never snaps
    double attachment_damping = 0.0;                            // N m s/rad, finite, >= 0
    double attachment_friction = 0.0;                           // N m, finite, >= 0
    std::vector<Eigen::Index> nodes;                            // interior node bodies, from end first (add_rope)
};

// Published per rope every step (PHY-004), at the end-of-step state.
struct RopeOutput {
    std::vector<double> tension;  // per segment, from end first, N; the released last segment reads 0
    double max_tension = 0.0;        // N
    double load_end_tension = 0.0;   // the segment at to_body, N; 0 after a snap
    bool attached = true;            // false once the to_body attachment is released
    double rest_length = 0.0;        // current, m
    double snatch_omega_h = 0.0;     // omega_s h; 0 when to_body is the world
    bool snatch_underestimated = false;  // snatch_omega_h > Rope::snatch_resolution_limit: first snatch peak
                                         // about 15 % low
    double scheme_decay = 0.0;           // pi omega h of the swing mode (ruling 21); 0 without gravity
    // The largest max_tension above breaking_tension in a step before MultibodyConfig::rope_snap_evaluation_start,
    // since load, N: a snap held back (not released); 0 when none.
    double held_tension = 0.0;
};

// Appends the chain's nodes evenly between from_world and to_world (the attach points' world positions in the model's
// neutral configuration), moves the end half-masses onto the end bodies and registers the rope in model.ropes;
// returns its index. Throws InvalidModelError.
Eigen::Index add_rope(MultibodyModel& model, Rope rope, const Eigen::Vector3d& from_world,
                      const Eigen::Vector3d& to_world);

// One segment's state from its end points (a upper, b lower), rest length l, stiffness k = EA / l, damping c.
struct RopeSegment {
    Eigen::Vector3d direction = Eigen::Vector3d::Zero();  // a to b, unit; zero when the ends coincide
    double length = 0.0;   // d, m
    double rate = 0.0;     // d_dot, m/s
    double tension = 0.0;  // T >= 0, N
    bool taut = false;     // T > 0
};

[[nodiscard]] RopeSegment rope_segment(const Eigen::Vector3d& a, const Eigen::Vector3d& a_velocity,
                                       const Eigen::Vector3d& b, const Eigen::Vector3d& b_velocity, double rest,
                                       double stiffness, double damping);

// Throws InvalidModelError when a rope parameter is out of range (also used for the per-step rest length).
void validate_rope_parameters(const Rope& rope, Eigen::Index index);

}  // namespace mechsim::engine
