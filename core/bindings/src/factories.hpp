#pragma once

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/kernel/kernel.hpp"
#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/time.hpp"

namespace mechsim::bindings {

// The kernel the binding hands out; the kernel brief points this at the real kernel.
[[nodiscard]] std::unique_ptr<kernel::Kernel> make_kernel(kernel::RunConfig config);

using PhysicsAdapterFactory = std::function<std::unique_ptr<kernel::PhysicsAdapter>()>;
using ControllerRuntimeFactory = std::function<std::unique_ptr<controller::ControllerRuntime>(kernel::Rate)>;

// Adapters and runtimes by name, in name order; "null" at the base. Further implementations add one entry each.
[[nodiscard]] std::vector<std::string> physics_adapter_names();
// Throws std::invalid_argument naming the known adapters on an unknown name.
[[nodiscard]] std::unique_ptr<kernel::PhysicsAdapter> make_physics_adapter(std::string_view name);

[[nodiscard]] std::vector<std::string> controller_runtime_names();
[[nodiscard]] std::unique_ptr<controller::ControllerRuntime> make_controller_runtime(std::string_view name,
                                                                                     const kernel::Rate& control_rate);

}  // namespace mechsim::bindings
