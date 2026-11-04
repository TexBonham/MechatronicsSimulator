#include "factories.hpp"

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "mechsim/kernel/cosim_kernel.hpp"
#include "mechsim/physics/mujoco/mujoco_adapter.hpp"

namespace mechsim::bindings {

namespace {

const std::map<std::string, PhysicsAdapterFactory, std::less<>>& physics_adapters() {
    static const std::map<std::string, PhysicsAdapterFactory, std::less<>> factories{
        {"mujoco", [] { return std::make_unique<physics::mujoco::MujocoAdapter>(); }},
        {"null", [] { return std::make_unique<kernel::NullPhysicsAdapter>(); }},
    };
    return factories;
}

const std::map<std::string, ControllerRuntimeFactory, std::less<>>& controller_runtimes() {
    static const std::map<std::string, ControllerRuntimeFactory, std::less<>> factories{
        {"null", [](kernel::Rate rate) { return std::make_unique<controller::NullControllerRuntime>(rate); }},
    };
    return factories;
}

template <typename Map>
std::vector<std::string> names_of(const Map& map) {
    std::vector<std::string> names;
    names.reserve(map.size());
    for (const auto& entry : map) {
        names.push_back(entry.first);
    }
    return names;
}

template <typename Map>
[[noreturn]] void unknown(const char* what, std::string_view name, const Map& map) {
    std::string known;
    for (const auto& entry : map) {
        if (!known.empty()) {
            known += ", ";
        }
        known += entry.first;
    }
    throw std::invalid_argument(std::string("unknown ") + what + " '" + std::string(name) + "'; known: " + known);
}

}  // namespace

std::unique_ptr<kernel::Kernel> make_kernel(kernel::RunConfig config) {
    return std::make_unique<kernel::CoSimKernel>(std::move(config));
}

std::vector<std::string> physics_adapter_names() {
    return names_of(physics_adapters());
}

std::unique_ptr<kernel::PhysicsAdapter> make_physics_adapter(std::string_view name) {
    const auto found = physics_adapters().find(name);
    if (found == physics_adapters().end()) {
        unknown("physics adapter", name, physics_adapters());
    }
    return found->second();
}

std::vector<std::string> controller_runtime_names() {
    return names_of(controller_runtimes());
}

std::unique_ptr<controller::ControllerRuntime> make_controller_runtime(std::string_view name,
                                                                       const kernel::Rate& control_rate) {
    const auto found = controller_runtimes().find(name);
    if (found == controller_runtimes().end()) {
        unknown("controller runtime", name, controller_runtimes());
    }
    return found->second(control_rate);
}

}  // namespace mechsim::bindings
