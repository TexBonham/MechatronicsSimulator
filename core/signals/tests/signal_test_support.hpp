#pragma once

#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/signal_component.hpp"

namespace signal_test {

using mechsim::kernel::SimTime;
using mechsim::signals::ComponentClock;
using mechsim::signals::PhysicalState;
using mechsim::signals::SignalComponent;

// 1 kHz layer on a 1 kHz base: one period is one base tick, so sample k runs at SimTime(k).
inline ComponentClock khz_clock(std::uint64_t seed = 7) {
    return ComponentClock{mechsim::kernel::Rate(1000), mechsim::kernel::Duration(1), seed};
}

// One body turned by angle about z and displaced by x along x; one joint carrying load in its first DOF.
inline PhysicalState state_at(double angle, double x, double load = 0.0) {
    PhysicalState state;
    mechsim::kernel::BodyState body;
    body.position = {x, 0.0, 0.0};
    body.orientation = {std::cos(angle / 2.0), 0.0, 0.0, std::sin(angle / 2.0)};
    state.bodies.push_back(body);
    mechsim::kernel::JointForce joint;
    joint.generalized_force = {load};
    state.joints.push_back(joint);
    return state;
}

// Drives a component one layer sample at a time, in the layer's order.
class Stepper {
public:
    explicit Stepper(SignalComponent& component, ComponentClock clock = khz_clock()) : component_(component) {
        component_.start(clock);
    }

    std::span<const double> step(const PhysicalState& state, const std::vector<double>& inputs = {}) {
        component_.begin_tick(SimTime(tick_));
        ++tick_;
        component_.set_inputs(inputs);
        component_.sample(state);
        component_.latch_outputs();
        return component_.output_values();
    }

    [[nodiscard]] std::int64_t tick() const noexcept { return tick_; }

private:
    SignalComponent& component_;
    std::int64_t tick_ = 0;
};

}  // namespace signal_test
