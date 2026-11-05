#pragma once

#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/pins/pins.hpp"

namespace mechsim::controller {

struct ControllerPartyConfig {
    std::string name = "controller";
    kernel::Rate rate{1000};
    std::string pin_inputs = "signals.pin_inputs";  // PinInputs payload
    std::string rails = "signals.rails";            // PowerRails payload; consumed only when rail is set
    std::string rail;                                // the controller's supply rail; empty: none
};

// Wraps a loaded ControllerRuntime as the controller-role party (KER-009 step 4, KER-020). Consumes the pin inputs
// and, with a rail named, the rails; publishes `<name>.pin_outputs` (PinOutputs). With a rail named, the first step
// calls reset(rail) and later steps call brownout(voltage, 0 V when off) whenever the rail's voltage changes
// (CTL-013); without one the caller resets the runtime before the run. Until the pin inputs are first published the
// tick sees zeroed inputs. record() writes the state code as the row's controller state (REC-007).
class ControllerParty final : public kernel::Party {
public:
    // Throws std::invalid_argument on a null runtime.
    ControllerParty(std::unique_ptr<ControllerRuntime> runtime, ControllerPartyConfig config);

    [[nodiscard]] std::string_view name() const override;
    [[nodiscard]] kernel::Rate nominal_rate() const override;
    [[nodiscard]] kernel::PartyRole role() const override;
    [[nodiscard]] std::span<const std::string> published() const override;
    [[nodiscard]] std::span<const std::string> consumed() const override;
    void start(const kernel::Rate& base_rate) override;
    void read_inputs(const kernel::CouplingReader& inputs) override;
    void step(kernel::SimTime t, kernel::Duration dt) override;
    void write_outputs(kernel::CouplingWriter& outputs) override;
    void on_fault(const kernel::PartyFault& fault) noexcept override;
    void record(kernel::TickRecord& row) const override;

    [[nodiscard]] ControllerRuntime& runtime() noexcept;
    [[nodiscard]] const PinOutputs& pin_outputs() const noexcept;

private:
    std::unique_ptr<ControllerRuntime> runtime_;
    ControllerPartyConfig config_;
    std::vector<std::string> published_;
    std::vector<std::string> consumed_;
    PinInputs inputs_;
    PinOutputs outputs_;
    std::optional<pins::PowerRail> rail_;
    std::optional<double> applied_voltage_;
};

}  // namespace mechsim::controller
