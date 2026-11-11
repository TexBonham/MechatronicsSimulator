// pybind11 first: it includes Python.h with _DEBUG hidden, so an MSVC Debug build links the release CPython.
#include <pybind11/pybind11.h>
#include <Python.h>

#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

#include "bindings.hpp"
#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/engine/engine_error.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/signals/pin_binding.hpp"
#include "mechsim/signals/signal_name.hpp"

namespace py = pybind11;

namespace mechsim::bindings {

namespace {

constexpr const char* module_name = "mechsim._core_ext";

py::object new_exception(py::module_& m, const char* name, py::handle base, const char* doc) {
    const std::string qualified = std::string(module_name) + "." + name;
    auto type =
        py::reinterpret_steal<py::object>(PyErr_NewExceptionWithDoc(qualified.c_str(), doc, base.ptr(), nullptr));
    if (!static_cast<bool>(type)) {
        throw py::error_already_set();
    }
    m.attr(name) = type;
    return type;
}

// The extension module, set once at import; sys.modules keeps it alive for the interpreter's lifetime.
py::handle& extension_module() {
    static py::handle module;
    return module;
}

py::object exception_type(const char* name) {
    return extension_module().attr(name);
}

void raise(const py::object& type, const py::object& instance) {
    PyErr_SetObject(type.ptr(), instance.ptr());
}

void raise_with_cause(const char* name, const std::exception& error, const std::string& cause) {
    const py::object type = exception_type(name);
    const py::object instance = type(error.what());
    instance.attr("cause") = cause;
    raise(type, instance);
}

void raise_engine(const char* name, const engine::EngineError& error) {
    const py::object type = exception_type(name);
    const py::object instance = type(error.what());
    instance.attr("time") = error.time();
    instance.attr("cause") = error.cause();
    raise(type, instance);
}

void translate(std::exception_ptr pointer) {
    if (pointer == nullptr) {
        return;
    }
    try {
        std::rethrow_exception(std::move(pointer));
    } catch (const kernel::PartyFault& error) {
        raise(exception_type("PartyFault"), party_fault_object(error));
    } catch (const kernel::AdapterError& error) {
        const py::object type = exception_type("AdapterError");
        const py::object instance = type(error.what());
        instance.attr("step") = error.step();
        instance.attr("cause") = error.cause();
        raise(type, instance);
    } catch (const kernel::KernelConfigError& error) {
        raise_with_cause("KernelConfigError", error, error.what());
    } catch (const engine::DriftError& error) {
        const py::object type = exception_type("DriftError");
        const py::object instance = type(error.what());
        instance.attr("time") = error.time();
        instance.attr("cause") = error.cause();
        instance.attr("constraint_index") = error.constraint_index();
        raise(type, instance);
    } catch (const engine::NonFiniteStateError& error) {
        raise_engine("NonFiniteStateError", error);
    } catch (const engine::SolverNonConvergenceError& error) {
        raise_engine("SolverNonConvergenceError", error);
    } catch (const engine::InvalidModelError& error) {
        raise_engine("InvalidModelError", error);
    } catch (const engine::InvalidConfigError& error) {
        raise_engine("InvalidConfigError", error);
    } catch (const engine::EngineError& error) {
        raise_engine("EngineError", error);
    } catch (const controller::ControllerConfigError& error) {
        raise_with_cause("ControllerConfigError", error, error.what());
    } catch (const controller::ControllerError& error) {
        raise_with_cause("ControllerError", error, error.what());
    } catch (const signals::PinBindingError& error) {
        raise_with_cause("PinBindingError", error, error.what());
    } catch (const signals::SignalNameError& error) {
        raise_with_cause("SignalNameError", error, error.what());
    }
}

std::string attribute_text(const py::handle& value, const char* name) {
    if (py::hasattr(value, name)) {
        return py::str(value.attr(name));
    }
    return py::str(value);
}

}  // namespace

py::object party_fault_object(const kernel::PartyFault& fault) {
    const py::object instance = exception_type("PartyFault")(fault.what());
    instance.attr("party") = fault.party();
    instance.attr("time") = fault.time();
    instance.attr("cause") = fault.cause();
    return instance;
}

void rethrow_as_cpp(py::error_already_set& error) {
    if (error.matches(exception_type("PartyFault"))) {
        const py::object& value = error.value();
        kernel::SimTime time;
        if (py::hasattr(value, "time") && py::isinstance<kernel::SimTime>(value.attr("time"))) {
            time = value.attr("time").cast<kernel::SimTime>();
        }
        std::string party;
        if (py::hasattr(value, "party")) {
            party = py::str(value.attr("party"));
        }
        throw kernel::PartyFault(party, time, attribute_text(value, "cause"));
    }
    if (error.matches(exception_type("AdapterError"))) {
        const py::object& value = error.value();
        std::int64_t step = 0;
        if (py::hasattr(value, "step") && py::isinstance<py::int_>(value.attr("step"))) {
            step = value.attr("step").cast<std::int64_t>();
        }
        throw kernel::AdapterError(step, attribute_text(value, "cause"));
    }
    const std::string type_name = py::str(error.type().attr("__name__"));
    const std::string message = py::str(error.value());
    throw std::runtime_error(type_name + ": " + message);
}

void bind_errors(py::module_& m) {
    extension_module() = m;
    const py::handle runtime_error = PyExc_RuntimeError;
    const py::handle value_error = PyExc_ValueError;
    new_exception(m, "KernelConfigError", value_error,
                  "Configuration rejected before t = 0 (bad rate, end time off the base tick, duplicate party); "
                  "attribute cause.");
    new_exception(m, "PartyFault", runtime_error,
                  "A party fault that ended the run (KER-015); attributes party, time (SimTime) and cause.");
    new_exception(m, "AdapterError", runtime_error,
                  "A physics adapter failure (PHY-009, PHY-019); attributes step (completed steps) and cause.");
    const py::object engine_error = new_exception(
        m, "EngineError", runtime_error, "An own-engine failure; attributes time (s) and cause (ENG-024 to ENG-027).");
    new_exception(m, "NonFiniteStateError", engine_error, "Non-finite state or terms.");
    new_exception(m, "SolverNonConvergenceError", engine_error, "Constraint solver hit its iteration cap.");
    new_exception(m, "DriftError", engine_error, "Constraint drift past threshold; attribute constraint_index.");
    new_exception(m, "InvalidModelError", engine_error, "Rejected multibody model.");
    new_exception(m, "InvalidConfigError", engine_error, "Rejected engine configuration.");
    new_exception(m, "ControllerError", runtime_error, "Controller runtime failure during a run; attribute cause.");
    new_exception(m, "ControllerConfigError", value_error, "Controller runtime rejected at load; attribute cause.");
    new_exception(m, "PinBindingError", value_error, "Rejected pin binding; attribute cause.");
    new_exception(m, "SignalNameError", value_error, "Rejected signal or instance name; attribute cause.");
    py::register_exception_translator(&translate);
}

}  // namespace mechsim::bindings
