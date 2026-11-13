#include "mechsim/integrity/integrity_party.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/integrity/beam.hpp"
#include "mechsim/integrity/limits.hpp"
#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/physics_party.hpp"
#include "mechsim/kernel/physics_types.hpp"

namespace mechsim::integrity {

namespace {

std::string shortest(double value) {
    std::array<char, 32> buffer{};
    const std::to_chars_result written = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    return {buffer.data(), written.ptr};
}

std::string quoted(const std::string& text) {
    std::string out = "\"";
    for (const char c : text) {
        if (c == '"' || c == '\\') {
            out.push_back('\\');
        }
        out.push_back(c);
    }
    out.push_back('"');
    return out;
}

}  // namespace

std::string crossing_detail(const Crossing& crossing) {
    return R"({"limit": )" + shortest(crossing.limit) + R"(, "member": )" + quoted(crossing.element) +
           R"(, "quantity": )" + quoted(crossing.quantity) + R"(, "tier": 1, "value": )" + shortest(crossing.value) +
           "}";
}

IntegrityParty::IntegrityParty(Limits limits, IntegrityPartyConfig config, std::shared_ptr<IntegrityTrace> trace)
    : limits_(std::move(limits)), config_(std::move(config)), trace_(std::move(trace)) {
    if (trace_ == nullptr) {
        trace_ = std::make_shared<IntegrityTrace>();
    }
    if (config_.physics_outputs.empty()) {
        throw LimitsError("integrity party '" + config_.name + "': no physics outputs name");
    }
    for (const MemberLimits& member : limits_.members) {
        if (member.supports.empty()) {
            throw LimitsError("member '" + member.name + "': no supports");
        }
    }
    consumed_ = {config_.physics_outputs};
    if (limits_.yield_offset) {
        published_ = {config_.name + ".model_requests"};
    }
    members_.assign(limits_.members.size(), MemberState{});
    fastenings_.assign(limits_.fastenings.size(), false);
    trace_->peak_stress.assign(limits_.members.size(), 0.0);
    trace_->peak_deflection.assign(limits_.members.size(), 0.0);
    for (std::size_t m = 0; m < limits_.members.size(); ++m) {
        if (!limits_.members[m].nodes.has_value()) {
            trace_->peak_deflection[m] = std::numeric_limits<double>::quiet_NaN();
        }
    }
    trace_->min_buckling_margin.assign(limits_.members.size(), std::numeric_limits<double>::infinity());
    trace_->peak_fastener_load.assign(limits_.fastenings.size(), 0.0);
}

std::string_view IntegrityParty::name() const {
    return config_.name;
}

kernel::Rate IntegrityParty::nominal_rate() const {
    return config_.rate;
}

kernel::PartyRole IntegrityParty::role() const {
    return kernel::PartyRole::integrity;
}

std::span<const std::string> IntegrityParty::published() const {
    return published_;
}

std::span<const std::string> IntegrityParty::consumed() const {
    return consumed_;
}

void IntegrityParty::start(const kernel::Rate& /*base_rate*/) {}

void IntegrityParty::read_inputs(const kernel::CouplingReader& inputs) {
    has_outputs_ = inputs.has_value(config_.physics_outputs);
    if (has_outputs_) {
        outputs_ = inputs.payload<kernel::PhysicsOutputs>(config_.physics_outputs);
    }
}

void IntegrityParty::cross(kernel::SimTime t, const std::string& element, std::string_view quantity, double value,
                           double limit) {
    Crossing crossing{t, element, std::string(quantity), value, limit};
    pending_events_.push_back(kernel::RecordedEvent{t, std::string(event_source), std::string(event_kind),
                                                    crossing_detail(crossing)});
    trace_->crossings.push_back(std::move(crossing));
    if (config_.halt_on_crossing) {
        halted_ = true;
    }
}

void IntegrityParty::yield(const MemberLimits& member, const kernel::MemberEndLoads& loads, MemberState& state) {
    const double l = member.segment_length;
    std::size_t spring = member.first_spring;
    double moment = start_node_moment(loads.start, l);
    double* peak = &state.start_peak;
    if (std::abs(loads.end.moment[2]) > std::abs(loads.start.moment[2])) {
        spring = member.first_spring + member.segments - 1;
        moment = end_node_moment(loads.end, l);
        peak = &state.end_peak;
    }
    if (!(std::abs(moment) > *peak)) {
        return;
    }
    *peak = std::abs(moment);
    const double yield_moment = member.yield_strength * member.i_z / member.c_y;
    if (!(*peak > yield_moment)) {
        return;
    }
    double offset = (*peak - yield_moment) / member.node_stiffness;
    if (moment < 0.0) {
        offset = -offset;
    }
    const kernel::ModelChangeRequest request{kernel::ModelChangeKind::SpringOffset, spring, offset};
    requests_.requests.push_back(request);
    trace_->requests.push_back(request);
    new_requests_ = true;
}

void IntegrityParty::step(kernel::SimTime t, kernel::Duration dt) {
    if (!has_outputs_) {
        return;
    }
    const kernel::SimTime stamp = t + dt;
    requests_.requests.clear();
    for (std::size_t m = 0; m < limits_.members.size(); ++m) {
        const MemberLimits& member = limits_.members[m];
        if (member.index >= outputs_.members.size()) {
            throw LimitsError("member '" + member.name + "': physics publishes " +
                              std::to_string(outputs_.members.size()) + " members");
        }
        const kernel::MemberEndLoads& loads = outputs_.members[member.index];
        MemberState& state = members_[m];
        const double stress = peak_normal_stress(loads, member);
        trace_->peak_stress[m] = std::max(trace_->peak_stress[m], stress);
        if (limits_.stress && !state.stress && stress > member.allowable_stress) {
            state.stress = true;
            cross(stamp, member.name, stress_quantity, stress, member.allowable_stress);
        }
        const double load = compression(loads);
        if (load > 0.0) {
            const double margin = critical_load(member) / load;
            trace_->min_buckling_margin[m] = std::min(trace_->min_buckling_margin[m], margin);
            if (limits_.buckling && !state.buckling && margin < limits_.minimum_safety_factor) {
                state.buckling = true;
                cross(stamp, member.name, buckling_quantity, margin, limits_.minimum_safety_factor);
            }
        }
        if (limits_.deflection && member.nodes.has_value()) {
            const double deflection = node_deflection(outputs_.bodies, member);
            trace_->peak_deflection[m] = std::max(trace_->peak_deflection[m], deflection);
            if (!state.deflection && member.deflection_limit.has_value() &&
                deflection > member.deflection_limit.value()) {
                state.deflection = true;
                cross(stamp, member.name, deflection_quantity, deflection, member.deflection_limit.value());
            }
        }
        if (limits_.yield_offset) {
            yield(member, loads, state);
        }
    }
    for (std::size_t f = 0; f < limits_.fastenings.size(); ++f) {
        const FasteningLimits& fastening = limits_.fastenings[f];
        if (fastening.joint >= outputs_.joints.size()) {
            throw LimitsError("fastening '" + fastening.name + "': physics publishes " +
                              std::to_string(outputs_.joints.size()) + " joints");
        }
        const kernel::Vec3& force = outputs_.joints[fastening.joint].reaction.force;
        const double magnitude = std::sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2]);
        trace_->peak_fastener_load[f] = std::max(trace_->peak_fastener_load[f], magnitude);
        if (limits_.fastener && !fastenings_[f] && magnitude > fastening.load_limit) {
            fastenings_[f] = true;
            cross(stamp, fastening.name, fastener_quantity, magnitude, fastening.load_limit);
        }
    }
}

void IntegrityParty::write_outputs(kernel::CouplingWriter& outputs) {
    if (published_.empty()) {
        return;
    }
    if (new_requests_ || !written_) {
        if (new_requests_) {
            ++requests_.sequence;
        }
        outputs.set(published_[0], kernel::make_payload(requests_, std::string(kernel::model_requests_kind)));
        written_ = true;
        new_requests_ = false;
    }
}

void IntegrityParty::on_fault(const kernel::PartyFault& /*fault*/) noexcept {}

bool IntegrityParty::finished() const {
    return halted_;
}

void IntegrityParty::record(kernel::TickRecord& row) const {
    row.events.insert(row.events.end(), pending_events_.begin(), pending_events_.end());
    pending_events_.clear();
}

const IntegrityTrace& IntegrityParty::trace() const noexcept {
    return *trace_;
}

}  // namespace mechsim::integrity
