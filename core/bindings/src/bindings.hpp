#pragma once

#include <pybind11/pybind11.h>

#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "mechsim/kernel/party.hpp"

namespace mechsim::bindings {

void bind_errors(pybind11::module_& m);
void bind_time(pybind11::module_& m);
void bind_kernel(pybind11::module_& m);
void bind_physics(pybind11::module_& m);
void bind_recorder(pybind11::module_& m);
void bind_signals(pybind11::module_& m);
void bind_controller(pybind11::module_& m);
void bind_engine(pybind11::module_& m);

// The Python PartyFault exception instance for a C++ fault (RunReport.fault, Party.on_fault).
[[nodiscard]] pybind11::object party_fault_object(const kernel::PartyFault& fault);

// Rethrows a Python exception raised inside a trampoline as a C++ exception, formatted while the GIL is held: a
// Python PartyFault or AdapterError becomes its C++ class, anything else a std::runtime_error "Type: message".
[[noreturn]] void rethrow_as_cpp(pybind11::error_already_set& error);

// Calls the Python override of a pure virtual method. Holding the GIL here is what lets Kernel.run release it.
template <typename Result, typename Base, typename... Args>
Result call_override(const Base* self, const char* method, Args&&... args) {
    const pybind11::gil_scoped_acquire gil;
    const pybind11::function override = pybind11::get_override(self, method);
    if (!static_cast<bool>(override)) {
        throw std::logic_error(std::string("Python subclass does not implement ") + method + "()");
    }
    try {
        const pybind11::object result = override(std::forward<Args>(args)...);
        if constexpr (std::is_void_v<Result>) {
            return;
        } else {
            return result.cast<Result>();
        }
    } catch (pybind11::error_already_set& error) {
        rethrow_as_cpp(error);
    }
}

}  // namespace mechsim::bindings
