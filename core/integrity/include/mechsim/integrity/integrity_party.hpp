#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "mechsim/integrity/limits.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/physics_party.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::integrity {

inline constexpr std::string_view event_source = "integrity";
inline constexpr std::string_view event_kind = "integrity";
// A crossing before the evaluation start (TST-044): recorded, never counted, so it does not fail the case.
inline constexpr std::string_view before_evaluation_kind = "integrity_before_evaluation";

// Quantity names in a crossing's detail (INT-024).
inline constexpr std::string_view stress_quantity = "stress";
inline constexpr std::string_view buckling_quantity = "buckling_margin";
inline constexpr std::string_view deflection_quantity = "deflection";
inline constexpr std::string_view fastener_quantity = "fastener_load";

struct Crossing {
    kernel::SimTime time;
    std::string element;  // member or fastening
    std::string quantity;
    double value = 0.0;
    double limit = 0.0;
};

// What the party saw, kept outside it so a caller that handed the party to the kernel can still read it.
struct IntegrityTrace {
    std::vector<double> peak_stress;      // per member, Pa; NaN when stress is off (not evaluated)
    std::vector<double> peak_deflection;  // per member, m; NaN for a member without nodes (not evaluated)
    std::vector<double> min_buckling_margin;  // per member, infinite when never in compression
    std::vector<double> peak_fastener_load;   // per fastening, N
    std::vector<Crossing> crossings;          // counted crossings (at or after the evaluation start)
    std::vector<Crossing> before_evaluation;  // crossings before the evaluation start, recorded and not counted
    std::vector<kernel::ModelChangeRequest> requests;  // every model change requested, in order
};

struct IntegrityPartyConfig {
    std::string name = "integrity";
    kernel::Rate rate{1000};
    std::string physics_outputs = "physics.outputs";
    bool halt_on_crossing = false;  // INT-026: finished() after the first crossing
    // TST-044: rows stamped before it update no peak and their crossings are recorded as before_evaluation_kind
    // events (one per element and quantity, no re-arm) that neither count nor halt.
    kernel::SimTime evaluate_from{0};
};

// The tier 1 integrity party (INT-023 to INT-028, ADR 0010, round 5 rulings 24 to 26, 34, 35). Every step it reads
// the physics outputs, evaluates each member's stress, buckling margin and deflection and each fastening's load
// against the limits table, and raises one event per (element, quantity) at the first tick strictly past the limit
// (no re-arm). With yield on it publishes `<name>.model_requests` (ModelChangeRequests): the permanent offset of the
// node spring nearest the end with the larger moment.
class IntegrityParty final : public kernel::Party {
public:
    IntegrityParty(Limits limits, IntegrityPartyConfig config, std::shared_ptr<IntegrityTrace> trace = nullptr);

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
    [[nodiscard]] bool finished() const override;
    void record(kernel::TickRecord& row) const override;

    [[nodiscard]] const IntegrityTrace& trace() const noexcept;

private:
    struct MemberState {
        bool stress = false;
        bool buckling = false;
        bool deflection = false;
        bool early_stress = false;
        bool early_buckling = false;
        bool early_deflection = false;
        double start_peak = 0.0;
        double end_peak = 0.0;
        // Each node spring's yield offset in effect for the outputs read (INT-028): the party's own requests, applied
        // by physics before its next step, so from the next tick on; empty until the first request.
        std::vector<double> offsets;
    };

    struct FasteningState {
        bool counted = false;
        bool early = false;
    };

    void cross(kernel::SimTime t, const std::string& element, std::string_view quantity, double value, double limit);
    void crossed(kernel::SimTime t, bool counts, bool& counted, bool& early, const std::string& element,
                 std::string_view quantity, double value, double limit);
    void yield(const MemberLimits& member, const kernel::MemberEndLoads& loads, MemberState& state);
    void apply_offset(const kernel::ModelChangeRequest& request);

    Limits limits_;
    IntegrityPartyConfig config_;
    std::shared_ptr<IntegrityTrace> trace_;
    std::vector<std::string> published_;
    std::vector<std::string> consumed_;
    std::vector<MemberState> members_;
    std::vector<FasteningState> fastenings_;
    kernel::PhysicsOutputs outputs_;
    bool has_outputs_ = false;
    kernel::ModelChangeRequests requests_;
    bool new_requests_ = false;
    bool written_ = false;
    bool halted_ = false;
    mutable std::vector<kernel::RecordedEvent> pending_events_;
};

// The event detail of a crossing: the JSON of mechsim.testing.driver.IntegrityCrossing (keys sorted, tier 1).
[[nodiscard]] std::string crossing_detail(const Crossing& crossing);

}  // namespace mechsim::integrity
