#include "mechsim/kernel/physics_party.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/tick_record.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::kernel {

PhysicsParty::PhysicsParty(std::unique_ptr<PhysicsAdapter> adapter, PhysicsPartyConfig config)
    : adapter_(std::move(adapter)), config_(std::move(config)) {
    if (adapter_ == nullptr) {
        throw std::invalid_argument("physics party: null adapter");
    }
    published_ = {config_.name + ".outputs"};
}

std::string_view PhysicsParty::name() const {
    return config_.name;
}

Rate PhysicsParty::nominal_rate() const {
    return config_.rate;
}

PartyRole PhysicsParty::role() const {
    return PartyRole::physics;
}

std::span<const std::string> PhysicsParty::published() const {
    return published_;
}

std::span<const std::string> PhysicsParty::consumed() const {
    return config_.inputs;
}

namespace {

const std::string fault_source = "kernel";
const std::string fault_kind = "fault";

std::string window_text(SimTime start, SimTime end) {
    return "[" + std::to_string(start.count()) + ", " + std::to_string(end.count()) + ")";
}

bool holds_step(SimTime start, SimTime end, Duration period) {
    const std::int64_t p = period.count();
    const std::int64_t first = ((start.count() + p - 1) / p) * p;
    return first < end.count();
}

}  // namespace

void PhysicsParty::check_schedulable(SimTime start) const {
    if (started_) {
        throw KernelConfigError("physics party '" + config_.name + "': faults are scheduled before start()");
    }
    if (start.count() < 0) {
        throw KernelConfigError("physics party '" + config_.name + "': fault tick " + std::to_string(start.count()) +
                                " is before 0");
    }
}

void PhysicsParty::schedule_slip(std::size_t transmission, SimTime start, SimTime end, double fraction) {
    check_schedulable(start);
    if (end <= start) {
        throw KernelConfigError("physics party '" + config_.name + "': slip window " + window_text(start, end) +
                                " must end after it starts");
    }
    if (!std::isfinite(fraction) || fraction <= 0.0 || fraction > 1.0) {
        throw KernelConfigError("physics party '" + config_.name + "': slip fraction " + std::to_string(fraction) +
                                " is outside (0, 1]");
    }
    slips_.push_back(SlipWindow{transmission, start, end, fraction});
}

void PhysicsParty::schedule_payload_change(std::size_t payload, SimTime at, PayloadMass mass) {
    check_schedulable(at);
    if (!std::isfinite(mass.kg) || mass.kg <= 0.0) {
        throw KernelConfigError("physics party '" + config_.name + "': payload mass " + std::to_string(mass.kg) +
                                " kg must be finite and positive");
    }
    PayloadChange change;
    change.payload = payload;
    change.at = at;
    change.kg = mass.kg;
    payload_changes_.push_back(change);
}

void PhysicsParty::schedule_payload_change(std::size_t payload, SimTime at, PayloadCarrier carrier) {
    check_schedulable(at);
    PayloadChange change;
    change.payload = payload;
    change.at = at;
    change.is_mass = false;
    change.carrier = carrier.body;
    payload_changes_.push_back(change);
}

void PhysicsParty::start(const Rate& base_rate) {
    std::stable_sort(slips_.begin(), slips_.end(),
                     [](const SlipWindow& a, const SlipWindow& b) { return a.start < b.start; });
    std::stable_sort(payload_changes_.begin(), payload_changes_.end(),
                     [](const PayloadChange& a, const PayloadChange& b) { return a.at < b.at; });
    for (std::size_t i = 0; i < slips_.size(); ++i) {
        for (std::size_t j = i + 1; j < slips_.size(); ++j) {
            const SlipWindow& a = slips_[i];
            const SlipWindow& b = slips_[j];
            if (a.transmission == b.transmission && a.start < b.end && b.start < a.end) {
                throw KernelConfigError("physics party '" + config_.name + "': slip windows " +
                                        window_text(a.start, a.end) + " and " + window_text(b.start, b.end) +
                                        " on transmission " + std::to_string(a.transmission) + " overlap");
            }
        }
    }
    for (std::size_t i = 0; i < payload_changes_.size(); ++i) {
        for (std::size_t j = i + 1; j < payload_changes_.size(); ++j) {
            const PayloadChange& a = payload_changes_[i];
            const PayloadChange& b = payload_changes_[j];
            if (a.payload == b.payload && a.at == b.at) {
                throw KernelConfigError("physics party '" + config_.name + "': two changes to payload " +
                                        std::to_string(a.payload) + " at tick " + std::to_string(a.at.count()));
            }
        }
    }
    if (!slips_.empty()) {
        period_ = period_in_ticks(config_.rate, base_rate);
        for (const SlipWindow& window : slips_) {
            if (!holds_step(window.start, window.end, period_)) {
                throw KernelConfigError("physics party '" + config_.name + "': slip window " +
                                        window_text(window.start, window.end) + " holds none of its steps");
            }
        }
    }
    started_ = true;
}

void PhysicsParty::apply_faults(SimTime t) {
    std::vector<std::size_t> ending;
    for (SlipWindow& window : slips_) {
        if (window.started && !window.ended && t >= window.end) {
            window.ended = true;
            ending.push_back(window.transmission);
            pending_events_.push_back(RecordedEvent{t, fault_source, fault_kind,
                                                    "slip end transmission " + std::to_string(window.transmission)});
        }
    }
    std::vector<std::size_t> starting;
    for (SlipWindow& window : slips_) {
        if (!window.started && t >= window.start) {
            window.started = true;
            starting.push_back(window.transmission);
            adapter_->set_transmission_slip(window.transmission, window.fraction);
            pending_events_.push_back(RecordedEvent{t, fault_source, fault_kind,
                                                    "slip start transmission " + std::to_string(window.transmission) +
                                                        " fraction " + std::to_string(window.fraction)});
        }
    }
    for (const std::size_t transmission : ending) {
        if (std::find(starting.begin(), starting.end(), transmission) == starting.end()) {
            adapter_->set_transmission_slip(transmission, 0.0);
        }
    }
    for (PayloadChange& change : payload_changes_) {
        if (change.applied || t < change.at) {
            continue;
        }
        change.applied = true;
        std::string detail = "payload " + std::to_string(change.payload);
        if (change.is_mass) {
            adapter_->set_payload_mass(change.payload, change.kg);
            detail += " mass " + std::to_string(change.kg);
        } else if (change.carrier == kWorldBody) {
            adapter_->set_payload_attachment(change.payload, change.carrier);
            detail += " released";
        } else {
            adapter_->set_payload_attachment(change.payload, change.carrier);
            detail += " carrier " + std::to_string(change.carrier);
        }
        pending_events_.push_back(RecordedEvent{t, fault_source, fault_kind, detail});
    }
}

void PhysicsParty::read_inputs(const CouplingReader& inputs) {
    pending_ = PhysicsInputs{};
    has_pending_ = false;
    for (const std::string& source : config_.inputs) {
        if (!inputs.has_value(source)) {
            continue;
        }
        const auto& part = inputs.payload<PhysicsInputs>(source);
        if (!part.generalized_forces.empty()) {
            if (pending_.generalized_forces.empty()) {
                pending_.generalized_forces.assign(part.generalized_forces.size(), 0.0);
            }
            if (pending_.generalized_forces.size() != part.generalized_forces.size()) {
                throw AdapterError(adapter_->health().steps,
                                   "physics input '" + source + "' carries " +
                                       std::to_string(part.generalized_forces.size()) + " generalized forces, others " +
                                       std::to_string(pending_.generalized_forces.size()));
            }
            for (std::size_t i = 0; i < part.generalized_forces.size(); ++i) {
                pending_.generalized_forces[i] += part.generalized_forces[i];
            }
        }
        pending_.body_wrenches.insert(pending_.body_wrenches.end(), part.body_wrenches.begin(),
                                      part.body_wrenches.end());
        for (const JointDrive& drive : part.joint_drives) {
            for (const JointDrive& held : pending_.joint_drives) {
                if (held.joint == drive.joint) {
                    throw KernelConfigError("physics input '" + source + "': a second drive on joint " +
                                            std::to_string(drive.joint) + " in one tick (KER-027)");
                }
            }
            pending_.joint_drives.push_back(drive);
        }
        has_pending_ = true;
    }
}

void PhysicsParty::step(SimTime t, Duration dt) {
    if (!slips_.empty() && dt != period_) {
        throw KernelConfigError("physics party '" + config_.name + "': steps every " + std::to_string(dt.count()) +
                                " ticks, but its slip windows were checked at its nominal rate (" +
                                std::to_string(period_.count()) + " ticks)");
    }
    apply_faults(t);
    if (has_pending_) {
        adapter_->set_inputs(pending_);
    }
    adapter_->step();
    const PhysicsHealth health = adapter_->health();
    if (!health.converged) {
        throw AdapterError(health.steps, "solver did not converge after " + std::to_string(health.iterations) +
                                             " iterations (KER-016)");
    }
    const double limit = std::max(config_.drift.translational_m, config_.drift.rotational_rad);
    if (health.max_constraint_drift > limit) {
        throw AdapterError(health.steps, "constraint drift " + std::to_string(health.max_constraint_drift) +
                                             " exceeds the threshold " + std::to_string(limit) + " (KER-016)");
    }
}

void PhysicsParty::write_outputs(CouplingWriter& outputs) {
    outputs.set(published_[0], make_payload(adapter_->outputs(), std::string(physics_outputs_kind)));
}

void PhysicsParty::on_fault(const PartyFault& /*fault*/) noexcept {}

void PhysicsParty::record(TickRecord& row) const {
    row.physics = adapter_->outputs();
    row.events.insert(row.events.end(), pending_events_.begin(), pending_events_.end());
    pending_events_.clear();
}

PhysicsAdapter& PhysicsParty::adapter() noexcept {
    return *adapter_;
}

}  // namespace mechsim::kernel
