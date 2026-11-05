#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/kernel.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/recorder_sink.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

// SplitMix64: advances state and returns the next output. Child seeds are its outputs from RunConfig::seed, one per
// party in registration order (KER-012).
[[nodiscard]] std::uint64_t splitmix64_next(std::uint64_t& state) noexcept;
[[nodiscard]] std::vector<std::uint64_t> derive_child_seeds(std::uint64_t seed, std::size_t count);

// The milestone 1 co-simulation kernel: the Kernel contract plus the coupling table (KER-008, KER-011), child seeds
// (KER-012), end conditions (KER-017) and one recorder row per physics step carrying every party's record() slice
// and the coupling table (KER-022).
//
// Effective rate per party: party_rates[name], else control_rate for a controller-role party, else nominal_rate().
// Before t = 0, in this order, any of which throws KernelConfigError: a physics-role party off physics_rate; rates and
// the base rate; a party_rates entry
// naming no party; end_time on the base tick and at_time on a physics step (EndTriggers); every published name declared
// once; every consumed name published; on_signal published; party_state registered. Then set_seed and start on every
// party; an exception there is a fault at t = 0.
//
// A step at t is read_inputs, step, write_outputs; an exception from any of them, from record(), or from the sink is a
// PartyFault (the sink's under the name "recorder") with fan-out and a run_error report. End conditions are checked
// only at physics-step boundaries, right after the row, so a run never ends between rows; ticks counts base ticks.
class CoSimKernel final : public Kernel {
public:
    explicit CoSimKernel(RunConfig config);

    void register_party(std::unique_ptr<Party> party) override;
    [[nodiscard]] RunReport run(const EndCondition& end, RecorderSink& sink) override;
    [[nodiscard]] SimTime now() const override;
    [[nodiscard]] const RunConfig& config() const override;

    void set_end_triggers(const EndTriggers& triggers) override;
    [[nodiscard]] const CouplingTable& coupling() const noexcept;

private:
    struct Entry {
        std::unique_ptr<Party> party;
        std::string name;
        Rate rate;
        RateSource source = RateSource::nominal;
        Duration period;
        std::int64_t steps = 0;
    };

    struct Limits {
        SimTime end;
        std::optional<SimTime> at;
        Duration physics_period;
    };

    void assign_rates();
    [[nodiscard]] Limits prepare(const EndCondition& end);
    [[nodiscard]] std::optional<PartyFault> start_parties();
    [[nodiscard]] std::optional<PartyFault> step_party(Entry& entry);
    [[nodiscard]] std::optional<PartyFault> write_row(RecorderSink& sink);
    [[nodiscard]] std::optional<EndReason> end_reached(const EndCondition& end, const Limits& limits,
                                                       std::optional<PartyFault>& fault) const;
    [[nodiscard]] RunReport finish(RunReport report, const RecorderSink& sink) const;
    [[nodiscard]] RunReport fail(RunReport report, const PartyFault& fault, const RecorderSink& sink) const;

    RunConfig config_;
    std::vector<Entry> parties_;
    CouplingTable coupling_;
    EndTriggers triggers_;
    SimTime now_;
    bool ran_ = false;
};

}  // namespace mechsim::kernel
