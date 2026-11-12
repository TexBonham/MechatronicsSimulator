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
// Backlash b is a total dead band at the output: d(E) = E - b/2 above b/2, E + b/2 below -b/2, else 0 (no contact).
//
// Forces. In contact F = k d(E) + c dE/dt acts on the output joint; out of contact F = 0. The lossless reaction on
// the shaft is r = -n F (tension times radius: slip dissipates sigma of the input power, it does not cut the torque).
// Efficiency follows the power flow P = r omega, the power the coupling delivers to the shaft:
//   P < 0, forward (shaft drives load):  reaction = r / eta_forward
//   P > 0, backward (load drives shaft): reaction = r eta_backward; a self-locking lead screw passes 0 (Locked)
//   P = 0, static (shaft at rest):       reaction = r (no motion, no loss); a self-locking screw whose drive torque
//                                        does not oppose r (drive r >= 0, zero drive included) passes 0 (Locked)
// so the shaft gain g (reaction = -g F) is n / eta_f, n eta_b, n or 0, frozen at the step start. Static and
// backward reactions for a locked screw are zero: the load cannot turn the shaft; lowering it is an active drive.
// Lowering a self-locking screw under an active drive is unresisted (no nut-friction model in this tier): the load
// torque reads 0 while lowering, a documented limit, not a defect.
// "drive" for the lock decision is the shaft DOF's generalized-force input (set_generalized_forces) plus every
// set_shaft_torque on that shaft; joint spring torque and gravity on the shaft DOF are not part of it.
//
// Implicit treatment (ADR 0013). With the gain frozen, -df/d(theta, x) = k a b^T and -df/d(omega, xdot) = c a b^T
// with a = [g, -1], b = [n_eff, -1], added to K and D in contact. The only non-zero eigenvalue of H^-1 K is
// k (g n_eff / J + 1 / m) > 0 for a shaft inertia J and an output mass m, so backward Euler is stable at any finite
// stiffness and step. The coupling is the rigid limit only as k grows: the output lags n theta by F / k, so pick k
// with F / k below the position accuracy. Infinite stiffness is rejected; a rigid coupling is a constraint (next
// tier). The efficiency switch is a hard sign test with no threshold speed, so a lossy coupling at rest under load
// may alternate between forward and backward gain; static friction proper is the next tier. A lossy coupling
// holding a static load therefore alternates Forward and Backward gain on numerical sign chatter (no threshold
// speed by spec); the load torque jumps by up to 1 / (eta_f eta_b), and the stepper consumer must not treat that
// jump as a fault.
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
    // Ruling 21 approximation value: pi omega h, the amplitude fraction backward Euler removes per period of the
    // coupling mode (omega^2 = k b^T H^-1 a at the end state; 0 out of contact); set in step() outputs only.
    double scheme_decay = 0.0;
};

// Coupling at one state, with the linearization the provider adds to f, K and D.
struct TransmissionCoupling {
    TransmissionOutput output;
    double n = 0.0;       // output travel per input radian
    double n_eff = 0.0;   // (1 - slip) n
    double gain = 0.0;    // g: reaction = -g F
};

[[nodiscard]] double output_per_input(const Transmission& transmission);

// Throws InvalidModelError naming transmission `index` for a parameter outside its range.
void validate_transmission_parameters(const Transmission& transmission, Eigen::Index index);

// theta, omega: shaft; x, xdot: output joint; drive: generalized force applied to the shaft this step (N m).
[[nodiscard]] TransmissionCoupling couple(const Transmission& transmission, double slip, double slip_offset,
                                          double theta, double omega, double x, double xdot, double drive);

}  // namespace mechsim::engine
