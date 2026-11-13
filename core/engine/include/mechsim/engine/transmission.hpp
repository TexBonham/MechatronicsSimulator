#pragma once

#include <Eigen/Core>
#include <cstdint>

namespace mechsim::engine {

// Transmissions (ENG-043 to ENG-046, ENG-048): a compliant coupling from a motor shaft (the input body's Revolute
// joint, angle theta) to a driven joint (the output body's joint, coordinate x: Prismatic for a belt or a lead
// screw, Revolute for a gear pair).
//
// Kinematics. n is the output travel per input radian: belt n = ratio (pulley radius, m), lead screw n = ratio /
// (2 pi) (ratio is the lead, m per revolution), gear pair n = 1 / ratio (ratio is input speed over output speed);
// `reverses` flips its sign (external mesh, opposite hand). Slip sigma in [0, 1] is a fraction of the shaft motion
// that does not reach the output: the slip offset s grows by sigma n dtheta each step, so the stretch
//   E = n theta - x - s,  dE/dt = n_eff omega - xdot,  n_eff = (1 - sigma) n.
// Backlash b is a total dead band at the output: d(E) = E - b/2 at and above b/2, E + b/2 at and below -b/2 (in
// contact, closed), else 0 (free, strictly inside). A coupling with b = 0 is always in contact, zero stretch too.
//
// Forces. In contact F = k d(E) + c dE/dt acts on the output joint; out of contact F = 0. The lossless reaction on
// the shaft is L = -n F (tension times radius: slip dissipates sigma of the input power, it does not cut the torque).
//
// Efficiency (ruling 47), regularised through a threshold shaft speed omega_s (MultibodyConfig::
// friction_slip_threshold, the joints' friction threshold, frozen at the step's start rate as ruling 38 does for
// friction). P = L omega is the power the coupling delivers to the shaft, s_w = min(|omega| / omega_s, 1):
//   P < 0, Forward (shaft drives load):  g = n (1 + s_w (1 / eta_f - 1))
//   P > 0, Backward (load drives shaft): g = n (1 + s_w (eta_b - 1))
//   P = 0, Static (omega = 0, or F = 0):  g = n
// reaction -g F. At and above omega_s the gain is n / eta_f or n eta_b exactly, the hard switch of brief M3-E4, so
// every moving case is unchanged (iii). Below omega_s, with alpha = 1 / eta_f - 1 (forward) or eta_b - 1 (backward)
// and |omega| = sign(P) sign(L) omega on the side the step starts on, the loss term -n alpha s_w F equals
// -c omega with c = |alpha L| / omega_s >= 0: a viscous shaft torque. It is applied as the joints' friction is,
// implicitly with c frozen (f gets -g F at the start state, D gets c on the shaft) while the coupling keeps the
// lossless gain n in K and D (a = [n, -1]); above omega_s, c = 0 and a = [g, -1] as before. At omega = 0 the gain is
// n from both sides (continuous); the side the shaft will move to is not known at the step start, so c takes the
// larger loss, max(1 / eta_f - 1, 1 - eta_b) |L| / omega_s, as the joints' friction takes its full c at zero rate.
// With c = 0 there a screw lifted by a torque between n F and n F / eta_f would leave rest at a lossless step,
// overshoot omega_s, reverse into the lock row and start again: a ratchet instead of the creep.
//   (i) Static equilibrium. omega = 0 makes g = n and c = 0 whatever F is, so a shaft held by a drive spring K
//       against an output force F rests only at K theta = n F: the sag theta = n F / K is unique (the hard switch
//       made every K theta in [n eta_b F, n F / eta_f] a fixed point).
//  (ii) Creep. Within the old band the drive's excess torque K e (e = theta - theta_static) is below |alpha L|, so
//       the shaft speed stays below omega_s and, with inertia negligible against the drive, K e + (c + D_d) omega = 0
//       with c = |alpha| n F / omega_s: e decays with time constant tau_f = (n F (1 / eta_f - 1) / omega_s + D_d) / K
//       on the forward side (drive lifting the load toward the static sag) and tau_b = (n F (1 - eta_b) / omega_s +
//       D_d) / K on the backward side (load back-driving the shaft); backward Euler contracts e by 1 / (1 + h / tau)
//       per step. tau_f and tau_b are the approximation the adapter prints (ruling 47).
// (iii) Above omega_s the gains are the old ones bit for bit; only motion slower than omega_s changes.
// PowerFlow keeps its values; Static now means the shaft starts the step at exactly zero speed (or F = 0).
//
// Self-locking lead screw (ruling 48, ENG-044). The drive T is the shaft's own torque at the step start: its
// generalized-force input (set_generalized_forces), every set_shaft_torque on the shaft and its joint drive's
// spring-damper torque (set_joint_drive); the model joint spring and gravity on the shaft are not part of it. In
// contact with L != 0 the load would back-drive the shaft along sign(L). Unless T L > 0 (the drive itself turns
// the shaft the back-driving way: lowering), the provider adds a one-sided velocity row J = s e_shaft, s = -sign(L),
// impulse lambda >= 0, s omega >= 0 at the step's end, in the velocity-level pass after the joint-limit rows, in
// transmission order (the joint-limit pattern of ruling 43 with target 0; a binding row sets omega to 0 exactly).
// Case analysis:
//   - No positive work: lambda > 0 only where s omega_end = 0, so the row's power lambda s omega_end / h is zero; it
//     only removes back-driving speed.
//   - It cannot fight the drive: it is present only when T s >= 0 (drive zero or lifting) and pushes only along s.
//     When T pushes along sign(L) (T L > 0) the row is absent and, with P >= 0, the applied gain is 0: lowering
//     under a drive is unresisted (no nut-friction model in this tier; a zero load torque while lowering is a
//     documented limit, ENG-044).
//   - Held at rest with T s >= 0: the static gain n applies, so the load torque n F pushes back-driving and the row
//     carries it (row torque n F - T s): omega stays exactly 0 and theta does not drift, at any shaft inertia.
//     The old hold set the gain to 0 and relied on omega staying bitwise 0, true only while the LU of
//     S = H + h D + h^2 K kept the shaft row as pivot, J > n (h^2 k + h c).
//   - Lifting (P < 0) uses the forward gain; a lift that stops reverses into the row and is held.
//   - F = 0 in contact with T = 0 (a load just coming onto the screw): the back-driving side is not known at the
//     step start, but the implicit coupling would still turn the shaft by the end-of-step force, so the row is
//     bilateral there (omega = 0, impulse of either sign; zero work, no drive to fight). With T != 0 and F = 0 there
//     is no row: the drive turns an unloaded screw either way.
// Locked (published load torque 0) whenever P >= 0 and T L >= 0: the screw, not the shaft, carries the load.
//
// Implicit treatment (ADR 0013). With the coupling gain g_k frozen (n below omega_s, g above), -df/d(theta, x) =
// k a b^T and -df/d(omega, xdot) = c a b^T with a = [g_k, -1], b = [n_eff, -1], added to K and D in contact. The
// only non-zero eigenvalue of H^-1 K is k (g_k n_eff / J + 1 / m) > 0 for a shaft inertia J and an output mass m, so
// backward Euler is stable at any finite stiffness and step. The coupling is the rigid limit only as k grows: the
// output lags n theta by F / k, so pick k with F / k below the position accuracy. Infinite stiffness is rejected; a
// rigid coupling is a constraint (next tier). Static friction of the coupling proper is the next tier.
enum class TransmissionType : std::uint8_t { BeltPulley, LeadScrew, GearPair };

enum class PowerFlow : std::uint8_t { Static, Forward, Backward, Locked };

struct Transmission {
    TransmissionType type = TransmissionType::BeltPulley;
    Eigen::Index input_body = 0;    // motor shaft, Revolute joint
    Eigen::Index output_body = 1;   // Prismatic (belt, lead screw) or Revolute (gear pair)
    double ratio = 1.0;             // finite, > 0: radius m, lead m/rev, or speed ratio
    bool reverses = false;          // output moves opposite to n theta's positive sense
    double efficiency = 1.0;        // forward, in (0, 1]
    double back_efficiency = 1.0;   // backward, in (0, 1]
    bool self_locking = false;      // LeadScrew only
    double stiffness = 1e6;         // finite, > 0; N/m or N m/rad at the output
    double damping = 0.0;           // finite, >= 0; N s/m or N m s/rad at the output
    double backlash = 0.0;          // finite, >= 0; total dead band, m or rad at the output
    double slip = 0.0;              // in [0, 1]; initial value, settable mid-run (ENG-046)
};

struct TransmissionOutput {
    double force = 0.0;              // F on the output joint: belt tension, screw thrust (N), gear torque (N m)
    double shaft_load_torque = 0.0;  // couple(): torque applied to the shaft, -g F; published by
                                     // transmission_states()/outputs() negated, g F: the torque the shaft must
                                     // supply along its + axis (PHY-033, ENG-048), N m
    double stretch = 0.0;            // E, m or rad at the output
    double backlash_position = 0.0;  // E clamped to [-b/2, b/2]
    bool in_contact = false;
    double slip = 0.0;
    bool slip_active = false;        // slip > 0
    double slip_offset = 0.0;        // s, m or rad at the output
    PowerFlow power_flow = PowerFlow::Static;
    double lock_torque = 0.0;        // self-locking row torque on the shaft, s lambda / h, N m; step() outputs only
    // Ruling 21 approximation value: pi omega h, the amplitude fraction backward Euler removes per period of the
    // coupling mode (omega^2 = k b^T H^-1 a at the end state; 0 out of contact); set in step() outputs only.
    double scheme_decay = 0.0;
};

// Coupling at one state, with the linearization the provider adds to f, K and D.
struct TransmissionCoupling {
    TransmissionOutput output;
    double n = 0.0;       // output travel per input radian
    double n_eff = 0.0;   // (1 - slip) n
    double gain = 0.0;              // g: reaction -g F at the step start (0 when lowering a locked screw)
    double stiffness_gain = 0.0;    // g_k, the shaft entry of a in K and D: n below omega_s, else g
    double shaft_damping = 0.0;     // c on the shaft below omega_s, N m s/rad
    double shaft_torque = 0.0;      // -g F, applied to the shaft (output.shaft_load_torque reads 0 when Locked)
    bool lock = false;              // self-locking screw: one-sided velocity row on the shaft
    double lock_sign = 0.0;         // s: the row keeps s omega >= 0
    bool lock_bilateral = false;    // F = 0 and T = 0 in contact: omega = 0 (impulse of either sign)
};

[[nodiscard]] double output_per_input(const Transmission& transmission);

// Throws InvalidModelError naming transmission `index` for a parameter outside its range.
void validate_transmission_parameters(const Transmission& transmission, Eigen::Index index);

// theta, omega: shaft; x, xdot: output joint; drive: the shaft's own torque T at the step start (N m), see above;
// slip_threshold: omega_s, finite and > 0 (rad/s).
[[nodiscard]] TransmissionCoupling couple(const Transmission& transmission, double slip, double slip_offset,
                                          double theta, double omega, double x, double xdot, double drive,
                                          double slip_threshold);

}  // namespace mechsim::engine
