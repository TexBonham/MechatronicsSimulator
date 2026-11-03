#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <utility>

#include "mechsim/kernel/physics_adapter.hpp"

namespace mechsim::kernel {

AdapterError::AdapterError(std::int64_t step, std::string cause)
    : std::runtime_error("physics at step " + std::to_string(step) + ": " + cause),
      step_(step),
      cause_(std::move(cause)) {}

std::int64_t AdapterError::step() const noexcept {
    return step_;
}

const std::string& AdapterError::cause() const noexcept {
    return cause_;
}

namespace {

[[noreturn]] void no_element(std::int64_t time, const char* kind, std::size_t index) {
    throw AdapterError(time, std::string(kind) + " " + std::to_string(index) + " is not in the model");
}

}  // namespace

std::span<const std::string> NullPhysicsAdapter::supported_elements() const {
    return supported_;
}

void NullPhysicsAdapter::load(const PreparedModel& model, const Rate& physics_rate) {
    if (loaded_) {
        throw AdapterError(outputs_.steps, "load() called twice without reset()");
    }
    loaded_ = true;
    design_version_ = model.design_version;
    health_.physics_rate = physics_rate;
}

void NullPhysicsAdapter::reset() {
    loaded_ = false;
    design_version_.clear();
    outputs_ = PhysicsOutputs{};
    health_ = PhysicsHealth{};
}

void NullPhysicsAdapter::require_loaded(const char* operation) const {
    if (!loaded_) {
        throw AdapterError(outputs_.steps, std::string(operation) + " before load()");
    }
}

void NullPhysicsAdapter::step() {
    require_loaded("step()");
    ++outputs_.steps;
    health_.steps = outputs_.steps;
}

const PhysicsOutputs& NullPhysicsAdapter::outputs() const {
    return outputs_;
}

PhysicsHealth NullPhysicsAdapter::health() const {
    return health_;
}

void NullPhysicsAdapter::set_inputs(const PhysicsInputs& inputs) {
    require_loaded("set_inputs()");
    if (!inputs.generalized_forces.empty()) {
        throw AdapterError(outputs_.steps, "generalized forces for a model with no degrees of freedom");
    }
    if (!inputs.body_wrenches.empty()) {
        no_element(outputs_.steps, "body", inputs.body_wrenches.front().body);
    }
}

void NullPhysicsAdapter::set_spring_offset(std::size_t spring, double offset) {
    static_cast<void>(offset);
    require_loaded("set_spring_offset()");
    no_element(outputs_.steps, "spring", spring);
}

void NullPhysicsAdapter::set_spring_stiffness(std::size_t spring, double stiffness) {
    static_cast<void>(stiffness);
    require_loaded("set_spring_stiffness()");
    no_element(outputs_.steps, "spring", spring);
}

void NullPhysicsAdapter::disable_constraint(std::size_t constraint) {
    require_loaded("disable_constraint()");
    no_element(outputs_.steps, "constraint", constraint);
}

void NullPhysicsAdapter::release_load(std::size_t load) {
    require_loaded("release_load()");
    no_element(outputs_.steps, "load", load);
}

void NullPhysicsAdapter::set_payload_mass(std::size_t payload, double mass) {
    static_cast<void>(mass);
    require_loaded("set_payload_mass()");
    no_element(outputs_.steps, "payload", payload);
}

void NullPhysicsAdapter::set_payload_attachment(std::size_t payload, std::size_t body) {
    static_cast<void>(body);
    require_loaded("set_payload_attachment()");
    no_element(outputs_.steps, "payload", payload);
}

bool NullPhysicsAdapter::loaded() const noexcept {
    return loaded_;
}

const std::string& NullPhysicsAdapter::design_version() const noexcept {
    return design_version_;
}

}  // namespace mechsim::kernel
