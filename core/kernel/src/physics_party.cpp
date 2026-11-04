#include "mechsim/kernel/physics_party.hpp"

#include <algorithm>
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

void PhysicsParty::start(const Rate& /*base_rate*/) {}

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

void PhysicsParty::step(SimTime /*t*/, Duration /*dt*/) {
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
}

PhysicsAdapter& PhysicsParty::adapter() noexcept {
    return *adapter_;
}

}  // namespace mechsim::kernel
