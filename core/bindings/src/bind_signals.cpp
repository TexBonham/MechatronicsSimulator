#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "bindings.hpp"
#include "mechsim/signals/pin_binding.hpp"
#include "mechsim/signals/power_rail.hpp"
#include "mechsim/signals/signal_component.hpp"
#include "mechsim/signals/signal_name.hpp"

namespace py = pybind11;

namespace mechsim::bindings {

namespace {

using signals::PhysicalState;
using signals::SignalComponent;

std::vector<std::string> copy_names(std::span<const std::string> view) {
    return {view.begin(), view.end()};
}

// The Python tier (ADR 0005, SIG-030): derives from PythonSlowPathComponent so a Python plugin is a SignalComponent.
// Spans handed to C++ callers point into members refreshed on each call.
class PySignalComponent : public signals::PythonSlowPathComponent, public py::trampoline_self_life_support {
public:
    [[nodiscard]] std::string_view instance_name() const override {
        instance_ = call_override<std::string>(this, "instance_name");
        return instance_;
    }

    [[nodiscard]] std::span<const std::string> inputs() const override {
        inputs_ = call_override<std::vector<std::string>>(this, "inputs");
        return inputs_;
    }

    [[nodiscard]] std::span<const std::string> outputs() const override {
        outputs_ = call_override<std::vector<std::string>>(this, "outputs");
        return outputs_;
    }

    void sample(const PhysicalState& state) override { call_override<void>(this, "sample", PhysicalState(state)); }

    void set_inputs(std::span<const double> values) override {
        call_override<void>(this, "set_inputs", std::vector<double>(values.begin(), values.end()));
    }

    void latch_outputs() override { call_override<void>(this, "latch_outputs"); }

    [[nodiscard]] std::span<const double> output_values() const override {
        output_values_ = call_override<std::vector<double>>(this, "output_values");
        return output_values_;
    }

    void inject(const signals::FailureInjection& injection) override {
        call_override<void>(this, "inject", signals::FailureInjection(injection));
    }

private:
    mutable std::string instance_;
    mutable std::vector<std::string> inputs_;
    mutable std::vector<std::string> outputs_;
    mutable std::vector<double> output_values_;
};

// One tick of a component through the C++ interface, in the signals/CLAUDE.md order: sample, set_inputs,
// latch_outputs, output_values. Both plugin tiers run through this same path for the shared tests (SIG-030).
std::vector<double> component_tick(SignalComponent& component, const PhysicalState& state,
                                   const std::vector<double>& inputs) {
    component.sample(state);
    component.set_inputs(inputs);
    component.latch_outputs();
    const std::span<const double> values = component.output_values();
    return {values.begin(), values.end()};
}

void bind_pins(py::module_& m) {
    py::enum_<signals::PeripheralMode>(m, "PeripheralMode")
        .value("digital_input", signals::PeripheralMode::digital_input)
        .value("digital_output", signals::PeripheralMode::digital_output)
        .value("counter_input", signals::PeripheralMode::counter_input)
        .value("adc_input", signals::PeripheralMode::adc_input)
        .value("pulse_output", signals::PeripheralMode::pulse_output)
        .value("uart_rx", signals::PeripheralMode::uart_rx)
        .value("uart_tx", signals::PeripheralMode::uart_tx);

    py::class_<signals::PinBinding>(m, "PinBinding", "A dotted signal name bound to one MCU pin in one mode (SIG-023).")
        .def(py::init([](std::string signal, std::string pin, signals::PeripheralMode mode) {
                 return signals::PinBinding{std::move(signal), std::move(pin), mode};
             }),
             py::arg("signal"), py::arg("pin"), py::arg("mode") = signals::PeripheralMode::digital_input)
        .def_readwrite("signal", &signals::PinBinding::signal)
        .def_readwrite("pin", &signals::PinBinding::pin)
        .def_readwrite("mode", &signals::PinBinding::mode);

    py::class_<signals::PinBindingTable>(m, "PinBindingTable",
                                         "Every signal-controller value crosses through an entry (SIG-024).")
        .def(py::init<>())
        .def("add", &signals::PinBindingTable::add, py::arg("binding"))
        .def("find_signal", &signals::PinBindingTable::find_signal, py::arg("signal"))
        .def("find_pin", &signals::PinBindingTable::find_pin, py::arg("pin"))
        .def("entries", [](const signals::PinBindingTable& table) {
            const auto view = table.entries();
            return std::vector<signals::PinBinding>(view.begin(), view.end());
        });

    py::class_<signals::PowerRail>(m, "PowerRail")
        .def(py::init([](std::string name, double voltage, bool on) {
                 return signals::PowerRail{std::move(name), voltage, on};
             }),
             py::arg("name") = "", py::arg("voltage") = 0.0, py::arg("on") = false)
        .def_readwrite("name", &signals::PowerRail::name)
        .def_readwrite("voltage", &signals::PowerRail::voltage)
        .def_readwrite("on", &signals::PowerRail::on);

    m.def("is_dotted_signal_name", &signals::is_dotted_signal_name, py::arg("name"));
    m.def("is_instance_name", &signals::is_instance_name, py::arg("name"));
    m.def(
        "signal_instance", [](std::string_view name) { return std::string(signals::signal_instance(name)); },
        py::arg("name"));
}

void bind_failures(py::module_& m) {
    py::enum_<signals::FailureMode>(m, "FailureMode")
        .value("stuck", signals::FailureMode::stuck)
        .value("noisy", signals::FailureMode::noisy)
        .value("dropped", signals::FailureMode::dropped)
        .value("forced_level", signals::FailureMode::forced_level);

    py::class_<signals::FailureInjection>(m, "FailureInjection", "Active for start <= t < end, in base ticks.")
        .def(py::init<>())
        .def_readwrite("mode", &signals::FailureInjection::mode)
        .def_readwrite("start", &signals::FailureInjection::start)
        .def_readwrite("end", &signals::FailureInjection::end)
        .def_readwrite("noise_magnitude", &signals::FailureInjection::noise_magnitude)
        .def_readwrite("forced_level", &signals::FailureInjection::forced_level);

    m.def("validate_injection", &signals::validate_injection, py::arg("injection"));
}

void bind_components(py::module_& m) {
    py::class_<SignalComponent, PySignalComponent, py::smart_holder>(
        m, "SignalComponent",
        "One interface for every signal-layer component (SIG-030); subclass it in Python for the slow path.")
        .def(py::init<>())
        .def("instance_name", [](const SignalComponent& c) { return std::string(c.instance_name()); })
        .def("inputs", [](const SignalComponent& c) { return copy_names(c.inputs()); })
        .def("outputs", [](const SignalComponent& c) { return copy_names(c.outputs()); })
        .def("sample", &SignalComponent::sample, py::arg("state"))
        .def(
            "set_inputs", [](SignalComponent& c, const std::vector<double>& values) { c.set_inputs(values); },
            py::arg("values"))
        .def("latch_outputs", &SignalComponent::latch_outputs)
        .def("output_values",
             [](const SignalComponent& c) {
                 const auto view = c.output_values();
                 return std::vector<double>(view.begin(), view.end());
             })
        .def("inject", &SignalComponent::inject, py::arg("injection"));
    m.attr("PythonSlowPathComponent") = m.attr("SignalComponent");

    py::class_<signals::NullSignalComponent, SignalComponent, py::smart_holder>(m, "NullSignalComponent")
        .def(py::init<std::string, std::vector<std::string>, std::vector<std::string>>(), py::arg("instance"),
             py::arg("inputs"), py::arg("outputs"))
        .def_property_readonly("samples", &signals::NullSignalComponent::samples)
        .def_property_readonly("latches", &signals::NullSignalComponent::latches)
        .def_property_readonly("last_sample_step", &signals::NullSignalComponent::last_sample_step)
        .def_property_readonly("injections", [](const signals::NullSignalComponent& c) {
            const auto view = c.injections();
            return std::vector<signals::FailureInjection>(view.begin(), view.end());
        });

    py::class_<signals::SignalTable>(m, "SignalTable", "Signal names unique per design (SIG-023).")
        .def(py::init<>())
        .def("add", &signals::SignalTable::add, py::arg("component"))
        .def("contains", &signals::SignalTable::contains, py::arg("signal"))
        .def("__contains__", &signals::SignalTable::contains)
        .def("__len__", &signals::SignalTable::size);

    m.def("signal_component_tick", &component_tick, py::arg("component"), py::arg("state"), py::arg("inputs"),
          "Runs one tick of a component through the C++ interface and returns its latched output values.");
}

}  // namespace

void bind_signals(py::module_& m) {
    bind_pins(m);
    bind_failures(m);
    bind_components(m);
}

}  // namespace mechsim::bindings
