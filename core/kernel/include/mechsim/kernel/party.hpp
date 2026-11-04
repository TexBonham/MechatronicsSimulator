#pragma once

#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

// A party's place in the KER-009 per-tick order. The kernel uses it for rates: a controller party runs at
// RunConfig::control_rate and a physics party must run at RunConfig::physics_rate.
enum class PartyRole : std::uint8_t {
    physics,
    sensors,
    integrity,
    controller,
    outputs,
    drivers,
    recorder,
    other,
};

// The one error type that crosses from a party into the kernel (KER-015). The kernel wraps any other exception a
// party throws into a PartyFault carrying the party's name and the simulated time of the failing step.
class PartyFault : public std::runtime_error {
public:
    PartyFault(std::string party, SimTime time, std::string cause);

    [[nodiscard]] const std::string& party() const noexcept;
    [[nodiscard]] SimTime time() const noexcept;
    [[nodiscard]] const std::string& cause() const noexcept;

private:
    std::string party_;
    SimTime time_;
    std::string cause_;
};

// One interface for every party: physics, signal layer, controller runtime, tier 1 integrity, later parties
// (KER-018, KER-025). A Python slow-path party implements this same class through the binding (KER-019).
class Party {
public:
    Party() = default;
    virtual ~Party() = default;
    Party(const Party&) = delete;
    Party& operator=(const Party&) = delete;
    Party(Party&&) = delete;
    Party& operator=(Party&&) = delete;

    // Unique within a kernel; used in fault reports and to look up a rate override in RunConfig.
    [[nodiscard]] virtual std::string_view name() const = 0;
    [[nodiscard]] virtual Rate nominal_rate() const = 0;

    // Dotted names of the coupling variables this party publishes and consumes at sync points (KER-008, KER-018).
    [[nodiscard]] virtual std::span<const std::string> published() const = 0;
    [[nodiscard]] virtual std::span<const std::string> consumed() const = 0;

    // Called once per run, in registration order, before t = 0, with the run's base rate; t and dt in step() count
    // ticks of 1 / base_rate (to_seconds() converts).
    virtual void start(const Rate& base_rate) = 0;

    // Advance this party from t to t + dt. dt is the party's period; every consumed coupling variable holds its value
    // from the last sync point for the whole interval (KER-011).
    virtual void step(SimTime t, Duration dt) = 0;

    // Called once on every registered party, in registration order, when any party faults; the run then ends.
    virtual void on_fault(const PartyFault& fault) noexcept = 0;

    // Defaulted hooks added in milestone 1; a party that needs none of them overrides nothing.

    [[nodiscard]] virtual PartyRole role() const {
        return PartyRole::other;
    }
    // Before start(): this party's child seed, derived from RunConfig::seed in registration order (KER-012).
    virtual void set_seed(std::uint64_t /*child_seed*/) {}
    // At the start of each step, before step(): the consumed variables as of the last sync point (KER-008, KER-011).
    virtual void read_inputs(const CouplingReader& /*inputs*/) {}
    // At the end of each step, after step(): this party's published variables; unwritten ones hold (KER-011).
    virtual void write_outputs(CouplingWriter& /*outputs*/) {}
    // Checked at every sync point when EndCondition::party_state names this party (KER-017).
    [[nodiscard]] virtual bool finished() const {
        return false;
    }
    // Once per physics tick, in registration order, after every step of that tick: fill this party's slice of the
    // recorder row (KER-009 step 7, KER-022).
    virtual void record(TickRecord& /*row*/) const {}
};

}  // namespace mechsim::kernel
