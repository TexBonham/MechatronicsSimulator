#pragma once

#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

// Payload kinds the physics party exchanges (KER-008).
inline constexpr std::string_view physics_outputs_kind = "PhysicsOutputs";
inline constexpr std::string_view physics_inputs_kind = "PhysicsInputs";

struct PhysicsPartyConfig {
    std::string name = "physics";
    Rate rate{RunConfig::default_physics_rate_hz};
    // PhysicsInputs payloads applied for the next step, summed: generalized forces element by element (equal sizes
    // or empty), body wrenches concatenated in this order.
    std::vector<std::string> inputs{"signals.physics_inputs"};
    DriftThresholds drift;
};

// Wraps a loaded PhysicsAdapter as the physics-role party (KER-009 step 1, KER-016). Publishes `<name>.outputs`
// (PhysicsOutputs) after every step; consumes the configured PhysicsInputs names. A health report that did not
// converge, or whose drift exceeds the larger of the two drift thresholds (health carries one m-or-rad maximum),
// throws AdapterError, which the kernel reports as a run error (KER-016).
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

    [[nodiscard]] PhysicsAdapter& adapter() noexcept;

private:
    std::unique_ptr<PhysicsAdapter> adapter_;
    PhysicsPartyConfig config_;
    std::vector<std::string> published_;
    PhysicsInputs pending_;
    bool has_pending_ = false;
};

}  // namespace mechsim::kernel
