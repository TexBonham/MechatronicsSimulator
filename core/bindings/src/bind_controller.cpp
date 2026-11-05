#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <string>
#include <utility>

#include "bindings.hpp"
#include "factories.hpp"
#include "mechsim/controller/controller_runtime.hpp"

namespace py = pybind11;

namespace mechsim::bindings {

void bind_controller(py::module_& m) {
    using controller::ControllerRuntime;
    using controller::PinInputs;
    using controller::PinOutputs;

    m.attr("digital_in_channels") = controller::digital_in_channels;
    m.attr("counter_channels") = controller::counter_channels;
    m.attr("adc_channels") = controller::adc_channels;
    m.attr("uart_buffer_bytes") = controller::uart_buffer_bytes;
    m.attr("stepper_channels") = controller::stepper_channels;
    m.attr("digital_out_channels") = controller::digital_out_channels;
    m.attr("event_log_entries") = controller::event_log_entries;

    py::class_<PinInputs>(m, "PinInputs", "Polled pin inputs (CTL-004); fixed sizes, no rail field (CTL-013).")
        .def(py::init<>())
        .def_readwrite("digital_in", &PinInputs::digital_in)
        .def_readwrite("counter_delta", &PinInputs::counter_delta)
        .def_readwrite("adc_sample", &PinInputs::adc_sample)
        .def_readwrite("uart_rx", &PinInputs::uart_rx)
        .def_readwrite("uart_rx_count", &PinInputs::uart_rx_count);

    py::class_<controller::EventLogEntry>(m, "EventLogEntry")
        .def(py::init<>())
        .def_readwrite("code", &controller::EventLogEntry::code)
        .def_readwrite("arg", &controller::EventLogEntry::arg);

    py::class_<PinOutputs>(m, "PinOutputs", "Pin outputs of one control tick (CTL-018 state, fault, event log).")
        .def(py::init<>())
        .def_readwrite("step_count", &PinOutputs::step_count)
        .def_readwrite("direction", &PinOutputs::direction)
        .def_readwrite("enable", &PinOutputs::enable)
        .def_readwrite("digital_out", &PinOutputs::digital_out)
        .def_readwrite("uart_tx", &PinOutputs::uart_tx)
        .def_readwrite("uart_tx_count", &PinOutputs::uart_tx_count)
        .def_readwrite("state_code", &PinOutputs::state_code)
        .def_readwrite("fault_code", &PinOutputs::fault_code)
        .def_readwrite("event_log", &PinOutputs::event_log)
        .def_readwrite("event_count", &PinOutputs::event_count);

    py::class_<controller::FirmwareImage>(m, "FirmwareImage")
        .def(py::init([](std::string path, std::uint32_t tick_hz) {
                 return controller::FirmwareImage{std::move(path), tick_hz};
             }),
             py::arg("path") = "", py::arg("tick_hz") = 0)
        .def_readwrite("path", &controller::FirmwareImage::path)
        .def_readwrite("tick_hz", &controller::FirmwareImage::tick_hz);

    py::class_<ControllerRuntime, py::smart_holder>(m, "ControllerRuntime",
                                                    "One runtime interface, native and emulated (CTL-001).")
        .def("load", &ControllerRuntime::load, py::arg("firmware"))
        .def("reset", &ControllerRuntime::reset, py::arg("rail"))
        .def("tick", &ControllerRuntime::tick, py::arg("inputs"), py::arg("outputs"), "Overwrites outputs in place.")
        .def("freeze", &ControllerRuntime::freeze, py::arg("frozen"))
        .def("brownout", &ControllerRuntime::brownout, py::arg("voltage"));

    py::class_<controller::NullControllerRuntime, ControllerRuntime, py::smart_holder>(m, "NullControllerRuntime")
        .def(py::init<kernel::Rate>(), py::arg("control_rate"))
        .def_property_readonly("firmware_path", &controller::NullControllerRuntime::firmware_path)
        .def_property_readonly("ticks_run", &controller::NullControllerRuntime::ticks_run)
        .def_property_readonly("resets", &controller::NullControllerRuntime::resets)
        .def_property_readonly("frozen", &controller::NullControllerRuntime::frozen)
        .def_property_readonly("supply_voltage", &controller::NullControllerRuntime::supply_voltage);

    m.def("controller_runtime_names", &controller_runtime_names, "Registered runtime names, in name order.");
    m.def("make_controller_runtime", &make_controller_runtime, py::arg("name"), py::arg("control_rate"),
          "A registered controller runtime by name, built with the run's control rate.");
}

}  // namespace mechsim::bindings
