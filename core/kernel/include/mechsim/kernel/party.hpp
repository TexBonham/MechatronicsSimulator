#pragma once

#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

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
};

}  // namespace mechsim::kernel
