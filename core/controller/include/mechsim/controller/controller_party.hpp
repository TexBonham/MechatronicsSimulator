#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
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
    // Control ticks between the runtime producing outputs and the party publishing them (CTL-020); zeros until the
    // first delayed output arrives.
    std::size_t latency_ticks = 0;
};

// Wraps a loaded ControllerRuntime as the controller-role party (KER-009 step 4, KER-020). Consumes the pin inputs
// and, with a rail named, the rails; publishes `<name>.pin_outputs` (PinOutputs). With a rail named, the first step
// calls reset(rail) and later steps call brownout(voltage, 0 V when off) whenever the rail's voltage changes
// (CTL-013); without one the caller resets the runtime before the run. Until the pin inputs are first published the
// tick sees zeroed inputs. record() writes the state code as the row's controller state (REC-007).
//
// Scheduled faults (TST-018, CTL-021, KER-021), known before t = 0: windows are kernel ticks [start, end) and act on
// the party's steps inside them. A freeze window calls freeze(true) on the runtime at its first step and freeze(false)
// at the first step at or after its end; the kernel keeps stepping every other party. A slow-down window ticks the
// runtime on its first step and then once per `factor` steps; between those the outputs hold their levels with the
// per-tick buffers empty (held_levels). Each window start and end is a row event (source "kernel", kind "fault").
// Overlapping windows of one kind are a KernelConfigError at start().
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

    // Throw KernelConfigError after start(), on end <= start, a negative start, or (slow-down) a factor below 2.
    void schedule_freeze(kernel::SimTime start, kernel::SimTime end);
    void schedule_slowdown(kernel::SimTime start, kernel::SimTime end, std::int64_t factor);

    [[nodiscard]] ControllerRuntime& runtime() noexcept;
    [[nodiscard]] const PinOutputs& pin_outputs() const noexcept;

private:
    struct FaultEvent {
        kernel::SimTime time;
        std::string detail;
    };

    struct Window {
        kernel::SimTime start;
        kernel::SimTime end;
        std::int64_t factor = 1;
        bool started = false;
        bool ended = false;
        std::int64_t steps = 0;
    };

    void check_schedulable(kernel::SimTime start, kernel::SimTime end) const;
    // Advances one window to step time t; returns whether t lies inside it.
    bool advance(Window& window, kernel::SimTime t, const std::string& what);
    void publish(const PinOutputs& produced);

    std::unique_ptr<ControllerRuntime> runtime_;
    ControllerPartyConfig config_;
    std::vector<std::string> published_;
    std::vector<std::string> consumed_;
    PinInputs inputs_;
    PinOutputs outputs_;
    std::optional<pins::PowerRail> rail_;
    std::optional<double> applied_voltage_;
    PinOutputs produced_;
    std::deque<PinOutputs> delayed_;
    std::vector<Window> freezes_;
    std::vector<Window> slowdowns_;
    bool started_ = false;
    bool frozen_ = false;
    // Written in step(), handed to the next row by record(), which the kernel calls const once per row.
    mutable std::vector<FaultEvent> pending_events_;
};

}  // namespace mechsim::controller
