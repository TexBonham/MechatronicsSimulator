#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

// Payload kinds the physics party exchanges (KER-008).
inline constexpr std::string_view physics_outputs_kind = "PhysicsOutputs";
inline constexpr std::string_view physics_inputs_kind = "PhysicsInputs";

// Recorded event of each model change in the outputs (ruling 26, ruling 40): source "physics", kind "model_change",
// detail "<kind> <source> <index> value <value>" (snake case names, value in shortest round-trip form).
inline constexpr std::string_view model_change_source = "physics";
inline constexpr std::string_view model_change_kind = "model_change";
[[nodiscard]] std::string model_change_detail(const ModelChangeEvent& change);

struct PhysicsPartyConfig {
    std::string name = "physics";
    Rate rate{RunConfig::default_physics_rate_hz};
    // PhysicsInputs payloads applied for the next step, summed: generalized forces element by element (equal sizes
    // or empty), body wrenches concatenated in this order.
    std::vector<std::string> inputs{"signals.physics_inputs"};
    DriftThresholds drift;
};

// Scheduled payload changes (ENG-035, PHY-006): a new mass, or a new carrier (kWorldBody releases).
struct PayloadMass {
    double kg = 0.0;
};
struct PayloadCarrier {
    std::size_t body = kWorldBody;
};

// Wraps a loaded PhysicsAdapter as the physics-role party (KER-009 step 1, KER-016). Publishes `<name>.outputs`
// (PhysicsOutputs) after every step; consumes the configured PhysicsInputs names. A health report that did not
// converge, or whose drift exceeds the larger of the two drift thresholds (health carries one m-or-rad maximum),
// throws AdapterError, which the kernel reports as a run error (KER-016).
//
// Scheduled faults, known before t = 0, in kernel ticks: before the first step at or after a slip window's start the
// party calls set_transmission_slip(transmission, fraction), and set_transmission_slip(transmission, 0) before the
// first step at or after its end; a payload change applies before the first step at or after its tick. Each start,
// end and change is a row event (source "kernel", kind "fault"). Overlapping slip windows on one transmission, or two
// changes to one payload at one tick, or a slip window holding no step at the party's rate, are a KernelConfigError at
// start(); with slip windows, a step period other than the nominal one faults the run.
class PhysicsParty final : public Party {
public:
    // Throws std::invalid_argument on a null adapter.
    PhysicsParty(std::unique_ptr<PhysicsAdapter> adapter, PhysicsPartyConfig config);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] Rate nominal_rate() const override;
    [[nodiscard]] PartyRole role() const override;
    [[nodiscard]] std::span<const std::string> published() const override;
    [[nodiscard]] std::span<const std::string> consumed() const override;
    void start(const Rate& base_rate) override;
    void read_inputs(const CouplingReader& inputs) override;
    void step(SimTime t, Duration dt) override;
    void write_outputs(CouplingWriter& outputs) override;
    void on_fault(const PartyFault& fault) noexcept override;
    void record(TickRecord& row) const override;

    // Throw KernelConfigError after start(), on a window with end <= start or a negative start, a fraction outside
    // (0, 1], a mass not finite and positive.
    void schedule_slip(std::size_t transmission, SimTime start, SimTime end, double fraction);
    void schedule_payload_change(std::size_t payload, SimTime at, PayloadMass mass);
    void schedule_payload_change(std::size_t payload, SimTime at, PayloadCarrier carrier);

    [[nodiscard]] PhysicsAdapter& adapter() noexcept;

private:
    struct SlipWindow {
        std::size_t transmission = 0;
        SimTime start;
        SimTime end;
        double fraction = 0.0;
        bool started = false;
        bool ended = false;
    };
    struct PayloadChange {
        std::size_t payload = 0;
        SimTime at;
        bool is_mass = true;
        double kg = 0.0;
        std::size_t carrier = kWorldBody;
        bool applied = false;
    };

    void check_schedulable(SimTime start) const;
    void apply_faults(SimTime t);

    std::unique_ptr<PhysicsAdapter> adapter_;
    PhysicsPartyConfig config_;
    std::vector<std::string> published_;
    PhysicsInputs pending_;
    bool has_pending_ = false;
    std::vector<SlipWindow> slips_;
    std::vector<PayloadChange> payload_changes_;
    bool started_ = false;
    Duration period_{1};
    // Written in step(), handed to the next row by record(), which the kernel calls const once per row.
    mutable std::vector<RecordedEvent> pending_events_;
    mutable std::int64_t recorded_changes_step_ = -1;
};

}  // namespace mechsim::kernel
