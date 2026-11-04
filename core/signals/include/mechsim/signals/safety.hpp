#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "mechsim/signals/component_base.hpp"

namespace mechsim::signals {

// Catalog type power_supply: output `voltage`, V (SIG-015).
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

}  // namespace mechsim::signals
