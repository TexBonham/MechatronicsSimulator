#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "mechsim/kernel/physics_types.hpp"

#include "mechsim/signals/component_base.hpp"

namespace mechsim::signals {

// Catalog type power_supply: output `voltage`, V (SIG-015). A power_loss injection outputs 0 V for [start, end) and
// the voltage again after, so every rail it feeds is off for the window (SIG-019).
class PowerSupply final : public BuiltinComponent {
public:
    // Throws SignalConfigError on a negative or non-finite voltage.
    PowerSupply(std::string instance, double output_voltage);

    void sample(const PhysicalState& state) override;
    void latch_outputs() override;

private:
    double voltage_;
};

// Catalog type estop_relay.
struct EStopRelayParams {
    double cutoff_delay = 0.0;  // s
    bool manual_reset = true;
};

// Inputs `estop_loop` (1 = loop closed, 0 = E-stop pressed), `reset` (level). Outputs `status` and `contact_closed`
// (1 = contacts closed). A press seen at sample k opens the contacts at sample k + floor(cutoff_delay / period), so
// within the cutoff delay (SIG-017); a press always completes. Contacts reclose once the loop is closed again: at
// once without manual reset, on a reset level with it. No controller signal reaches it.
class EStopRelay final : public BuiltinComponent {
public:
    EStopRelay(std::string instance, EStopRelayParams params);

    void sample(const PhysicalState& state) override;
    void latch_outputs() override;

private:
    void on_start() override;

    EStopRelayParams params_;
    std::int64_t cutoff_periods_ = 0;
    std::optional<std::int64_t> pressed_at_;
    bool closed_ = true;
};

// Catalog type watchdog.
struct WatchdogParams {
    double timeout = 0.0;       // s
    double cutoff_delay = 0.0;  // s
    bool manual_reset = false;
};

// Input `heartbeat` (level). Outputs `tripped`, `contact_closed`. With e the sample of the last heartbeat edge (the
// first sample counts as one), it trips at the first sample j with (j - e) > floor(timeout / period), i.e. no
// heartbeat for longer than the timeout (SIG-018), and opens the contacts floor(cutoff_delay / period) samples later.
// Without manual reset the next edge re-arms it and recloses the contacts.
class Watchdog final : public BuiltinComponent {
public:
    Watchdog(std::string instance, WatchdogParams params);

    void sample(const PhysicalState& state) override;
    void latch_outputs() override;

private:
    void on_start() override;

    WatchdogParams params_;
    std::int64_t timeout_periods_ = 0;
    std::int64_t cutoff_periods_ = 0;
    bool primed_ = false;
    bool last_level_ = false;
    std::int64_t last_edge_ = 0;
    std::optional<std::int64_t> tripped_at_;
    bool closed_ = true;
};

// Catalog type brake (SIG-020, LIB-027, ruling 1 placeholder). joint is the neutral index of the joint it acts on.
struct BrakeParams {
    std::size_t joint = 0;
    double brake_torque = 0.0;   // N m (N on a prismatic joint), > 0
    double engage_delay = 0.0;   // s
    double release_delay = 0.0;  // s
    bool power_off_engaged = true;
    double damping = 0.0;  // N m s/rad of the drive while engaged
    // A0, the joint's inverse inertia (MuJoCo dof_invweight0) from the wiring; absent: the default stiffness.
    std::optional<double> inverse_inertia;
};

inline constexpr double brake_envelope_product = 100.0;  // K_b A0 h^2: a quarter of the adapter envelope's 400
inline constexpr double brake_default_compliance = 0.01;  // rad (m) of drive travel at brake_torque, without A0

// Inputs `release` (level), `vcc` (V, the coil rail). Outputs `engaged` (level), `torque` (the drive torque physics
// applied in the step just taken, read back from joint_drive_torques; 0 while released).
// Coil energised = vcc > 0 and release high. With power_off_engaged the brake engages while the coil is not
// energised (release low or the rail off); without it, it engages while vcc > 0 and release is low. The first sample
// takes the demanded state at once; after that a change of demand first seen at sample k takes effect at k +
// periods_floor(engage_delay or release_delay), cancelled if the demand reverts first.
// While engaged it publishes one joint drive (PHY-034) on joint: stiffness K_b = brake_envelope_product / (A0 h^2)
// with A0 given (h the layer period, so K_b A0 h^2 = 100 < 400), else brake_torque / brake_default_compliance;
// damping as given. Holding: rest is the joint coordinate at engagement. When the read-back torque exceeds
// brake_torque in magnitude it slips: with s the sign of that torque, each step rest = theta_p + s brake_torque / K_b,
// theta_p = theta + omega h + alpha h^2 (omega, alpha the backward differences of the joint coordinate), so the drive
// torque at the end of the step is s brake_torque when the acceleration holds over it (the fixed point of the
// semi-implicit step; deviations decay by K_b h^2 / (J + K_b h^2) per step). It sticks again (rest = theta_p) once
// the predicted velocity omega + alpha h no longer moves against s.
class Brake final : public BuiltinComponent {
public:
    // Throws SignalConfigError on a non-positive brake torque or inverse inertia, or a negative delay or damping.
    Brake(std::string instance, BrakeParams params);

    void sample(const PhysicalState& state) override;
    void latch_outputs() override;
    [[nodiscard]] std::span<const kernel::JointDrive> joint_drives() const override;
    [[nodiscard]] double stiffness() const noexcept;  // K_b, after start()

private:
    void on_start() override;

    BrakeParams params_;
    double stiffness_ = 0.0;
    std::int64_t engage_periods_ = 0;
    std::int64_t release_periods_ = 0;
    bool primed_ = false;
    bool engaged_ = false;
    bool slipping_ = false;
    double slip_sign_ = 0.0;
    std::optional<std::int64_t> pending_since_;
    double theta_ = 0.0;
    double omega_ = 0.0;
    double alpha_ = 0.0;
    std::int64_t readings_ = 0;
    double torque_ = 0.0;
    double drive_rest_ = 0.0;
    std::vector<kernel::JointDrive> drive_;
};

}  // namespace mechsim::signals
