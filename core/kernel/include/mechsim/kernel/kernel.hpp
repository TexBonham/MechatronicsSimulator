#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/recorder_sink.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

// The run ends at end_time, or earlier when stop returns true at a sync point (KER-017). stop sees the time at the
// end of the base tick just completed. end_time must be a whole number of base ticks, else run() throws.
struct EndCondition {
    Seconds end_time;
    std::function<bool(SimTime)> stop;
};

enum class RunOutcome : std::uint8_t {
    end_condition_met,
    run_error,
};

// How the run ended (KER-015, KER-017, REC-025).
struct RunReport {
    RunOutcome outcome = RunOutcome::end_condition_met;
    Rate base_rate;          // the run's base tick is 1 / base_rate
    SimTime end_time;        // ticks reached: the failing step's start time on a run error
    std::int64_t ticks = 0;  // completed base ticks
    std::optional<PartyFault> fault;
};

// Owns simulated time (KER-001) and steps registered parties (KER-018).
//
// Time starts at t = 0 (KER-002) and advances only in run() (KER-001), never from the wall clock (KER-024). At run
// start the kernel sets the base rate to the lcm of the physics rate and every party rate, so any rational rate works
// and a party may run finer than physics (KER-003); it rejects an end time off the base tick (invariant 7), then calls
// start(base_rate) on every party. At each base tick t, every party whose period divides t is stepped with
// step(t, period), in registration order (KER-013); the caller registers in the KER-009 order:
//   1. physics advances one step and publishes poses, velocities, forces and member end-loads;
//   2. the tier 1 integrity party evaluates the published end-loads;
//   3. sensor models sample the new state;
//   4. the controller runtime advances by the same interval;
//   5. the signal layer collects pin outputs;
//   6. driver and stepper models turn them into motor shaft torque gated by the power rail, and transmissions into
//      joint force, which physics applies in the next step (KER-010);
//   7. the kernel writes the tick's recorder row last, once per physics step.
// A party not stepped at t holds every coupling variable it published at its last sync point (zero-order hold,
// KER-011).
//
// Any exception a party throws ends the run (KER-015): the kernel wraps it in a PartyFault naming the party, the
// step's start time and the cause, calls on_fault on every party in registration order, writes no row for the
// failed tick, and returns a RunReport with outcome run_error. The kernel itself does not throw from run() for a
// party fault; it throws KernelConfigError for configuration found before t = 0.
class Kernel {
public:
    Kernel() = default;
    virtual ~Kernel() = default;
    Kernel(const Kernel&) = delete;
    Kernel& operator=(const Kernel&) = delete;
    Kernel(Kernel&&) = delete;
    Kernel& operator=(Kernel&&) = delete;

    // Throws KernelConfigError on a duplicate or empty name or registration after run().
    virtual void register_party(std::unique_ptr<Party> party) = 0;
    [[nodiscard]] virtual RunReport run(const EndCondition& end, RecorderSink& sink) = 0;
    [[nodiscard]] virtual SimTime now() const = 0;
    [[nodiscard]] virtual const RunConfig& config() const = 0;
};

// Steps registered parties in order at their rates and writes one row per physics step holding only the time. No
// coupling variables are exchanged; milestone 1 replaces it.
class NullKernel final : public Kernel {
public:
    explicit NullKernel(RunConfig config);

    void register_party(std::unique_ptr<Party> party) override;
    [[nodiscard]] RunReport run(const EndCondition& end, RecorderSink& sink) override;
    [[nodiscard]] SimTime now() const override;
    [[nodiscard]] const RunConfig& config() const override;

private:
    struct Entry {
        std::unique_ptr<Party> party;
        Rate rate;
        Duration period;
    };

    void prepare_rates();
    void fan_out(const PartyFault& fault) noexcept;

    RunConfig config_;
    std::vector<Entry> parties_;
    SimTime now_;
    bool ran_ = false;
};

}  // namespace mechsim::kernel
