#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "bindings.hpp"
#include "mechsim/controller/channel_map.hpp"
#include "mechsim/controller/controller_party.hpp"
#include "mechsim/controller/controller_runtime.hpp"
#include "mechsim/controller/native_controller_runtime.hpp"
#include "mechsim/kernel/coupling.hpp"
#include "mechsim/kernel/cosim_kernel.hpp"
#include "mechsim/kernel/kernel.hpp"
#include "mechsim/kernel/memory_recorder_sink.hpp"
#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_party.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/pins/pins.hpp"
#include "mechsim/signals/component_base.hpp"
#include "mechsim/signals/front_end.hpp"
#include "mechsim/signals/operator_link.hpp"
#include "mechsim/signals/safety.hpp"
#include "mechsim/signals/sensors.hpp"
#include "mechsim/signals/signal_component.hpp"
#include "mechsim/signals/signal_layer.hpp"
#include "mechsim/signals/stepper.hpp"

namespace py = pybind11;

namespace mechsim::bindings {

namespace {

using kernel::CouplingValue;
using kernel::make_payload;
using kernel::PayloadPtr;

// ===== Parameter readers ===================================================
template <typename T>
const T* payload_if(const PayloadPtr& payload) {
    const auto* typed = dynamic_cast<const kernel::TypedPayload<T>*>(payload.get());
    if (typed == nullptr) {
        return nullptr;
    }
    return &typed->get();
}

double number(const py::dict& params, const char* key, double fallback) {
    if (!params.contains(key)) {
        return fallback;
    }
    return params[key].cast<double>();
}

bool flag(const py::dict& params, const char* key, bool fallback) {
    if (!params.contains(key)) {
        return fallback;
    }
    return params[key].cast<bool>();
}

signals::JointCoordinate coordinate(const py::dict& params) {
    if (!params.contains("coordinate")) {
        throw std::invalid_argument("parameter 'coordinate' (JointCoordinate) is required");
    }
    return params["coordinate"].cast<signals::JointCoordinate>();
}

signals::SensorFidelity fidelity(const py::dict& params) {
    signals::SensorFidelity f;
    f.noise = number(params, "noise", f.noise);
    f.delay = number(params, "delay", f.delay);
    f.sample_rate = number(params, "sample_rate", f.sample_rate);
    return f;
}

std::optional<std::size_t> index(const py::dict& params, const char* key) {
    if (!params.contains(key)) {
        return std::nullopt;
    }
    return params[key].cast<std::size_t>();
}

void check_known(const std::string& type, const py::dict& params, const std::vector<std::string>& known) {
    for (const auto& item : params) {
        const auto key = item.first.cast<std::string>();
        bool found = false;
        for (const std::string& name : known) {
            if (name == key) {
                found = true;
            }
        }
        if (!found) {
            std::string message = "component type '";
            message += type;
            message += "' has no parameter '";
            message += key;
            message += "'";
            throw std::invalid_argument(message);
        }
    }
}

}  // namespace

// ===== Coupling values =====================================================
py::object coupling_to_python(const CouplingValue& value) {
    if (std::holds_alternative<std::monostate>(value)) {
        return py::none();
    }
    if (const auto* real = std::get_if<double>(&value)) {
        return py::float_(*real);
    }
    if (const auto* integer = std::get_if<std::int64_t>(&value)) {
        return py::int_(*integer);
    }
    if (const auto* level = std::get_if<bool>(&value)) {
        return py::bool_(*level);
    }
    const auto& payload = std::get<PayloadPtr>(value);
    if (const auto* p = payload_if<kernel::PhysicsOutputs>(payload)) {
        return py::cast(*p);
    }
    if (const auto* p = payload_if<kernel::PhysicsInputs>(payload)) {
        return py::cast(*p);
    }
    if (const auto* p = payload_if<pins::PinInputs>(payload)) {
        return py::cast(*p);
    }
    if (const auto* p = payload_if<pins::PinOutputs>(payload)) {
        return py::cast(*p);
    }
    if (const auto* p = payload_if<std::vector<pins::PowerRail>>(payload)) {
        return py::cast(*p);
    }
    if (const auto* p = payload_if<std::vector<signals::StatusMessage>>(payload)) {
        return py::cast(*p);
    }
    if (const auto* p = payload_if<signals::ExternalInputs>(payload)) {
        return py::cast(*p);
    }
    if (const auto* p = payload_if<signals::OperatorCommands>(payload)) {
        return py::cast(*p);
    }
    if (const auto* p = payload_if<kernel::ModelChangeRequests>(payload)) {
        return py::cast(*p);
    }
    throw std::invalid_argument("coupling payload '" + std::string(payload->kind()) + "' has no Python form");
}

CouplingValue coupling_from_python(const py::handle& value) {
    if (py::isinstance<py::bool_>(value)) {
        return CouplingValue{value.cast<bool>()};
    }
    if (py::isinstance<py::int_>(value)) {
        return CouplingValue{value.cast<std::int64_t>()};
    }
    if (py::isinstance<py::float_>(value)) {
        return CouplingValue{value.cast<double>()};
    }
    if (py::isinstance<kernel::PhysicsOutputs>(value)) {
        return make_payload(value.cast<kernel::PhysicsOutputs>(), std::string(kernel::physics_outputs_kind));
    }
    if (py::isinstance<kernel::PhysicsInputs>(value)) {
        return make_payload(value.cast<kernel::PhysicsInputs>(), std::string(kernel::physics_inputs_kind));
    }
    if (py::isinstance<pins::PinInputs>(value)) {
        return make_payload(value.cast<pins::PinInputs>(), pins::pin_inputs_kind);
    }
    if (py::isinstance<pins::PinOutputs>(value)) {
        return make_payload(value.cast<pins::PinOutputs>(), pins::pin_outputs_kind);
    }
    if (py::isinstance<signals::OperatorCommands>(value)) {
        return make_payload(value.cast<signals::OperatorCommands>(), signals::operator_commands_kind);
    }
    if (py::isinstance<py::dict>(value)) {
        return make_payload(value.cast<signals::ExternalInputs>(), signals::external_inputs_kind);
    }
    if (py::isinstance<py::list>(value) && py::len(value) > 0 && py::isinstance<pins::PowerRail>(value[py::int_(0)])) {
        return make_payload(value.cast<std::vector<pins::PowerRail>>(), pins::power_rails_kind);
    }
    throw py::type_error("a coupling value is a bool, int, float, PhysicsOutputs, PhysicsInputs, PinInputs, "
                         "PinOutputs, OperatorCommands, dict[str, float] or a non-empty list of PowerRail");
}

// ===== Signal components ===================================================
std::unique_ptr<signals::SignalComponent> make_signal_component(const std::string& type, const std::string& instance,
                                                                const py::dict& params) {
    if (type == "limit_switch") {
        check_known(type, params, {"coordinate", "position", "actuate_above", "hysteresis", "resolution",
                                   "debounce_time", "normally_open", "noise", "delay", "sample_rate"});
        signals::LimitSwitchParams p;
        p.coordinate = coordinate(params);
        p.position = number(params, "position", p.position);
        p.actuate_above = flag(params, "actuate_above", p.actuate_above);
        p.hysteresis = number(params, "hysteresis", p.hysteresis);
        p.resolution = number(params, "resolution", p.resolution);
        p.debounce_time = number(params, "debounce_time", p.debounce_time);
        p.normally_open = flag(params, "normally_open", p.normally_open);
        p.fidelity = fidelity(params);
        return std::make_unique<signals::LimitSwitch>(instance, p);
    }
    if (type == "incremental_encoder") {
        check_known(type, params,
                    {"coordinate", "resolution", "reversed", "linear", "noise", "delay", "sample_rate"});
        signals::EncoderParams p;
        p.coordinate = coordinate(params);
        p.resolution = number(params, "resolution", p.resolution);
        p.reversed = flag(params, "reversed", p.reversed);
        p.linear = flag(params, "linear", p.linear);
        p.fidelity = fidelity(params);
        return std::make_unique<signals::IncrementalEncoder>(instance, p);
    }
    if (type == "step_dir_driver") {
        check_known(type, params, {"microsteps", "enable_active_high"});
        signals::StepDirDriverParams p;
        if (params.contains("microsteps")) {
            p.microsteps = params["microsteps"].cast<std::int64_t>();
        }
        p.enable_active_high = flag(params, "enable_active_high", p.enable_active_high);
        return std::make_unique<signals::StepDirDriver>(instance, p);
    }
    if (type == "stepper_behavioral") {
        check_known(type, params, {"step_angle", "rotor_inertia", "holding_torque", "damping", "joint",
                                   "torque_speed", "transmission", "shaft_axial_inertia", "shaft_body"});
        signals::BehavioralStepperParams p;
        p.step_angle = number(params, "step_angle", p.step_angle);
        p.rotor_inertia = number(params, "rotor_inertia", p.rotor_inertia);
        p.holding_torque = number(params, "holding_torque", p.holding_torque);
        p.damping = number(params, "damping", p.damping);
        if (params.contains("joint")) {
            p.joint = params["joint"].cast<std::size_t>();
        }
        if (params.contains("torque_speed")) {
            for (const auto& point : params["torque_speed"].cast<std::vector<std::pair<double, double>>>()) {
                p.torque_speed.push_back(signals::TorqueSpeedPoint{point.first, point.second});
            }
        }
        if (params.contains("transmission")) {
            p.transmission = params["transmission"].cast<std::size_t>();
        }
        if (params.contains("shaft_axial_inertia")) {
            p.shaft_axial_inertia = params["shaft_axial_inertia"].cast<double>();
        }
        if (params.contains("shaft_body")) {
            p.shaft_body = params["shaft_body"].cast<std::string>();
        }
        return std::make_unique<signals::BehavioralStepper>(instance, p);
    }
    if (type == "power_supply") {
        check_known(type, params, {"voltage"});
        return std::make_unique<signals::PowerSupply>(instance, number(params, "voltage", 0.0));
    }
    if (type == "estop_relay") {
        check_known(type, params, {"cutoff_delay", "manual_reset"});
        signals::EStopRelayParams p;
        p.cutoff_delay = number(params, "cutoff_delay", p.cutoff_delay);
        p.manual_reset = flag(params, "manual_reset", p.manual_reset);
        return std::make_unique<signals::EStopRelay>(instance, p);
    }
    if (type == "watchdog") {
        check_known(type, params, {"timeout", "cutoff_delay", "manual_reset"});
        signals::WatchdogParams p;
        p.timeout = number(params, "timeout", p.timeout);
        p.cutoff_delay = number(params, "cutoff_delay", p.cutoff_delay);
        p.manual_reset = flag(params, "manual_reset", p.manual_reset);
        return std::make_unique<signals::Watchdog>(instance, p);
    }
    if (type == "load_cell") {
        check_known(type, params, {"payload", "joint", "joint_reaction", "capacity", "sensitivity",
                                   "excitation_voltage", "resolution", "overload_limit", "nonlinearity", "noise",
                                   "delay", "sample_rate"});
        signals::LoadCellParams p;
        p.payload = index(params, "payload");
        p.joint = index(params, "joint");
        p.joint_reaction = index(params, "joint_reaction");
        p.capacity = number(params, "capacity", p.capacity);
        p.sensitivity = number(params, "sensitivity", p.sensitivity);
        p.excitation_voltage = number(params, "excitation_voltage", p.excitation_voltage);
        p.resolution = number(params, "resolution", p.resolution);
        p.overload_limit = number(params, "overload_limit", p.overload_limit);
        p.nonlinearity = number(params, "nonlinearity", p.nonlinearity);
        p.fidelity = fidelity(params);
        return std::make_unique<signals::LoadCell>(instance, p);
    }
    if (type == "load_cell_amplifier") {
        check_known(type, params, {"gain", "offset", "output_full_scale", "bandwidth", "adc_bits", "adc_reference"});
        signals::AmplifierParams p;
        p.gain = number(params, "gain", p.gain);
        p.offset = number(params, "offset", p.offset);
        p.output_full_scale = number(params, "output_full_scale", p.output_full_scale);
        p.bandwidth = number(params, "bandwidth", p.bandwidth);
        if (params.contains("adc_bits")) {
            p.adc.bits = params["adc_bits"].cast<std::int64_t>();
        }
        p.adc.reference = number(params, "adc_reference", p.adc.reference);
        return std::make_unique<signals::LoadCellAmplifier>(instance, p);
    }
    if (type == "imu") {
        check_known(type, params, {"body", "gravity", "accel_range", "gyro_range", "resolution", "gyro_resolution",
                                   "gyro_noise", "noise", "delay", "sample_rate"});
        signals::ImuParams p;
        if (params.contains("body")) {
            p.body = params["body"].cast<std::size_t>();
        }
        if (params.contains("gravity")) {
            p.gravity = params["gravity"].cast<kernel::Vec3>();
        }
        p.accel_range = number(params, "accel_range", p.accel_range);
        p.gyro_range = number(params, "gyro_range", p.gyro_range);
        p.resolution = number(params, "resolution", p.resolution);
        p.gyro_resolution = number(params, "gyro_resolution", p.gyro_resolution);
        p.gyro_noise = number(params, "gyro_noise", p.gyro_noise);
        p.fidelity = fidelity(params);
        return std::make_unique<signals::Imu>(instance, p);
    }
    if (type == "brake") {
        check_known(type, params, {"joint", "brake_torque", "engage_delay", "release_delay", "power_off_engaged",
                                   "damping", "inverse_inertia"});
        signals::BrakeParams p;
        if (params.contains("joint")) {
            p.joint = params["joint"].cast<std::size_t>();
        }
        p.brake_torque = number(params, "brake_torque", p.brake_torque);
        p.engage_delay = number(params, "engage_delay", p.engage_delay);
        p.release_delay = number(params, "release_delay", p.release_delay);
        p.power_off_engaged = flag(params, "power_off_engaged", p.power_off_engaged);
        p.damping = number(params, "damping", p.damping);
        if (params.contains("inverse_inertia")) {
            p.inverse_inertia = params["inverse_inertia"].cast<double>();
        }
        return std::make_unique<signals::Brake>(instance, p);
    }
    throw std::invalid_argument("unknown built-in component type '" + type +
                                "'; known: brake, estop_relay, imu, incremental_encoder, limit_switch, load_cell, "
                                "load_cell_amplifier, power_supply, step_dir_driver, stepper_behavioral, watchdog");
}

// ===== Module binding ======================================================
void bind_cosim(py::module_& m) {
    py::class_<kernel::CouplingReader>(m, "CouplingReader", "A party's read view of its consumed names (KER-008).")
        .def("value", [](const kernel::CouplingReader& r, const std::string& name) {
            return coupling_to_python(r.value(name));
        }, py::arg("name"))
        .def("has_value", &kernel::CouplingReader::has_value, py::arg("name"))
        .def("real", &kernel::CouplingReader::real, py::arg("name"))
        .def("integer", &kernel::CouplingReader::integer, py::arg("name"))
        .def("flag", &kernel::CouplingReader::flag, py::arg("name"));
    py::class_<kernel::CouplingWriter>(m, "CouplingWriter", "A party's write view of its published names (KER-008).")
        .def("set", [](kernel::CouplingWriter& w, const std::string& name, const py::object& value) {
            w.set(name, coupling_from_python(value));
        }, py::arg("name"), py::arg("value"));
    py::class_<kernel::CouplingTable>(m, "CouplingTable")
        .def("contains", &kernel::CouplingTable::contains, py::arg("name"))
        .def("publisher", &kernel::CouplingTable::publisher, py::arg("name"))
        .def("value", [](const kernel::CouplingTable& t, const std::string& name) {
            return coupling_to_python(t.value(name));
        }, py::arg("name"))
        .def("names", [](const kernel::CouplingTable& t) {
            std::vector<std::string> names;
            for (const kernel::CouplingSample& sample : t.snapshot()) {
                names.push_back(sample.name);
            }
            return names;
        });

    py::class_<kernel::CoSimKernel, kernel::Kernel, py::smart_holder>(
        m, "CoSimKernel", "The milestone 1 co-simulation kernel (KER-008 to KER-023).")
        .def(py::init<kernel::RunConfig>(), py::arg("config") = kernel::RunConfig{})
        .def("set_end_triggers", &kernel::CoSimKernel::set_end_triggers, py::arg("triggers"))
        .def_property_readonly("coupling", &kernel::CoSimKernel::coupling, py::return_value_policy::reference_internal);

    py::class_<kernel::InMemoryRecorderSink, kernel::RecorderSink, py::smart_holder>(
        m, "InMemoryRecorderSink", "Stores every row; drain() hands them over oldest first (KER-022).")
        .def(py::init<>())
        .def("rows", [](const kernel::InMemoryRecorderSink& s) {
            return std::vector<kernel::TickRecord>(s.rows().begin(), s.rows().end());
        })
        .def("drain", &kernel::InMemoryRecorderSink::drain)
        .def("drained", &kernel::InMemoryRecorderSink::drained)
        .def("subscriber_count", &kernel::InMemoryRecorderSink::subscriber_count)
        .def("subscriber_errors", &kernel::InMemoryRecorderSink::subscriber_errors);

    py::class_<kernel::PhysicsPartyConfig>(m, "PhysicsPartyConfig")
        .def(py::init<>())
        .def_readwrite("name", &kernel::PhysicsPartyConfig::name)
        .def_readwrite("rate", &kernel::PhysicsPartyConfig::rate)
        .def_readwrite("inputs", &kernel::PhysicsPartyConfig::inputs)
        .def_readwrite("drift", &kernel::PhysicsPartyConfig::drift)
        .def_readwrite("model_requests", &kernel::PhysicsPartyConfig::model_requests);
    py::class_<kernel::PhysicsParty, kernel::Party, py::smart_holder>(
        m, "PhysicsParty", "A loaded PhysicsAdapter as the physics party (KER-016); takes ownership of the adapter.")
        .def(py::init<std::unique_ptr<kernel::PhysicsAdapter>, kernel::PhysicsPartyConfig>(), py::arg("adapter"),
             py::arg("config") = kernel::PhysicsPartyConfig{})
        .def("schedule_slip", &kernel::PhysicsParty::schedule_slip, py::arg("transmission"), py::arg("start"),
             py::arg("end"), py::arg("fraction"))
        .def(
            "schedule_payload_change",
            [](kernel::PhysicsParty& party, std::size_t payload, kernel::SimTime at, std::optional<double> mass,
               std::optional<std::int64_t> carrier) {
                if (mass.has_value() == carrier.has_value()) {
                    throw kernel::KernelConfigError("schedule_payload_change: give exactly one of mass and carrier");
                }
                if (mass.has_value()) {
                    party.schedule_payload_change(payload, at, kernel::PayloadMass{*mass});
                    return;
                }
                if (*carrier == -1) {
                    party.schedule_payload_change(payload, at, kernel::PayloadCarrier{kernel::kWorldBody});
                    return;
                }
                if (*carrier < 0) {
                    throw kernel::KernelConfigError("schedule_payload_change: carrier " + std::to_string(*carrier) +
                                                    " is neither a body index nor -1 (world)");
                }
                party.schedule_payload_change(payload, at,
                                              kernel::PayloadCarrier{static_cast<std::size_t>(*carrier)});
            },
            py::arg("payload"), py::arg("tick"), py::kw_only(), py::arg("mass") = py::none(),
            py::arg("carrier") = py::none(),
            "A new mass (kg) or a new carrier body (-1 releases to the world) before the step at `tick`.");

    py::class_<controller::ControllerPartyConfig>(m, "ControllerPartyConfig")
        .def(py::init<>())
        .def_readwrite("name", &controller::ControllerPartyConfig::name)
        .def_readwrite("rate", &controller::ControllerPartyConfig::rate)
        .def_readwrite("pin_inputs", &controller::ControllerPartyConfig::pin_inputs)
        .def_readwrite("rails", &controller::ControllerPartyConfig::rails)
        .def_readwrite("rail", &controller::ControllerPartyConfig::rail)
        .def_readwrite("latency_ticks", &controller::ControllerPartyConfig::latency_ticks);
    py::class_<controller::ControllerParty, kernel::Party, py::smart_holder>(
        m, "ControllerParty", "A loaded ControllerRuntime as the controller party (KER-020); takes ownership.")
        .def(py::init<std::unique_ptr<controller::ControllerRuntime>, controller::ControllerPartyConfig>(),
             py::arg("runtime"), py::arg("config") = controller::ControllerPartyConfig{})
        .def("pin_outputs", &controller::ControllerParty::pin_outputs)
        .def("schedule_freeze", &controller::ControllerParty::schedule_freeze, py::arg("start"), py::arg("end"))
        .def("schedule_slowdown", &controller::ControllerParty::schedule_slowdown, py::arg("start"), py::arg("end"),
             py::arg("factor"));

    py::class_<signals::ComponentClock>(m, "ComponentClock", "Base rate, layer period and child seed (SIG-030).")
        .def(py::init<>())
        .def(py::init<kernel::Rate, kernel::Duration, std::uint64_t>(), py::arg("base_rate"), py::arg("period"),
             py::arg("seed"))
        .def_readwrite("base_rate", &signals::ComponentClock::base_rate)
        .def_readwrite("period", &signals::ComponentClock::period)
        .def_readwrite("seed", &signals::ComponentClock::seed);
    m.def("derive_child_seed", &signals::derive_child_seed, py::arg("run_seed"), py::arg("instance"));

    py::enum_<signals::CoordinateKind>(m, "CoordinateKind")
        .value("linear", signals::CoordinateKind::linear)
        .value("angular", signals::CoordinateKind::angular);
    py::class_<signals::JointCoordinate>(m, "JointCoordinate")
        .def(py::init([](signals::CoordinateKind kind, std::size_t body, std::optional<std::size_t> reference,
                         kernel::Vec3 axis) { return signals::JointCoordinate{kind, body, reference, axis}; }),
             py::arg("kind"), py::arg("body"), py::arg("reference") = py::none(),
             py::arg("axis") = kernel::Vec3{1.0, 0.0, 0.0})
        .def_readwrite("kind", &signals::JointCoordinate::kind)
        .def_readwrite("body", &signals::JointCoordinate::body)
        .def_readwrite("reference", &signals::JointCoordinate::reference)
        .def_readwrite("axis", &signals::JointCoordinate::axis);
    m.def("make_signal_component", &make_signal_component, py::arg("type"), py::arg("instance"),
          py::arg("params") = py::dict(), "A built-in component by catalog type name (SIG-029).");

    py::enum_<signals::MessageDirection>(m, "MessageDirection")
        .value("to_controller", signals::MessageDirection::to_controller)
        .value("from_controller", signals::MessageDirection::from_controller);
    py::class_<signals::MessageSpec>(m, "MessageSpec")
        .def(py::init([](std::string name, std::string label, std::vector<std::string> fields,
                         signals::MessageDirection direction) {
                 return signals::MessageSpec{std::move(name), std::move(label), std::move(fields), direction};
             }),
             py::arg("name"), py::arg("label"), py::arg("fields"), py::arg("direction"))
        .def_readwrite("name", &signals::MessageSpec::name)
        .def_readwrite("label", &signals::MessageSpec::label)
        .def_readwrite("fields", &signals::MessageSpec::fields)
        .def_readwrite("direction", &signals::MessageSpec::direction);
    py::class_<signals::OperatorMessage>(m, "OperatorMessage")
        .def(py::init([](std::string name, std::vector<double> fields) {
                 return signals::OperatorMessage{std::move(name), std::move(fields)};
             }),
             py::arg("name"), py::arg("fields"))
        .def_readwrite("name", &signals::OperatorMessage::name)
        .def_readwrite("fields", &signals::OperatorMessage::fields);
    py::class_<signals::StatusMessage>(m, "StatusMessage")
        .def_readonly("label", &signals::StatusMessage::label)
        .def_readonly("message", &signals::StatusMessage::message);
    py::class_<signals::OperatorCommands>(m, "OperatorCommands",
                                          "Posted once per new sequence number (KER-011 holds the value).")
        .def(py::init([](std::int64_t sequence, std::vector<signals::OperatorMessage> messages) {
                 return signals::OperatorCommands{sequence, std::move(messages)};
             }),
             py::arg("sequence"), py::arg("messages"))
        .def_readwrite("sequence", &signals::OperatorCommands::sequence)
        .def_readwrite("messages", &signals::OperatorCommands::messages);
    m.def("make_operator_link", [](const std::string& instance, std::vector<signals::MessageSpec> messages,
                                   std::uint32_t baud) {
        return std::make_unique<signals::OperatorLink>(
            instance, std::make_unique<signals::AsciiLineCodec>(std::move(messages)), signals::UartTransport(baud));
    }, py::arg("instance"), py::arg("messages"), py::arg("baud"), "An ASCII-line operator link over UART 8N1.");
    const py::class_<signals::OperatorLink, signals::SignalComponent, py::smart_holder> link(m, "OperatorLink");

    py::enum_<signals::Stage>(m, "Stage")
        .value("power", signals::Stage::power)
        .value("sensor", signals::Stage::sensor)
        .value("driver", signals::Stage::driver)
        .value("actuator", signals::Stage::actuator)
        .value("front_end", signals::Stage::front_end);
    py::enum_<signals::PinField>(m, "PinField")
        .value("digital_in", signals::PinField::digital_in)
        .value("counter_delta", signals::PinField::counter_delta)
        .value("adc_sample", signals::PinField::adc_sample)
        .value("uart_rx", signals::PinField::uart_rx)
        .value("step_count", signals::PinField::step_count)
        .value("direction", signals::PinField::direction)
        .value("enable", signals::PinField::enable)
        .value("digital_out", signals::PinField::digital_out)
        .value("uart_tx", signals::PinField::uart_tx);
    py::class_<signals::ChannelSlot>(m, "ChannelSlot")
        .def(py::init([](std::string signal, signals::PinField field, std::size_t channel) {
                 return signals::ChannelSlot{std::move(signal), field, channel};
             }),
             py::arg("signal"), py::arg("field"), py::arg("channel"))
        .def_readwrite("signal", &signals::ChannelSlot::signal)
        .def_readwrite("field", &signals::ChannelSlot::field)
        .def_readwrite("channel", &signals::ChannelSlot::channel);
    py::class_<signals::Connection>(m, "Connection")
        .def(py::init([](std::string from, std::string to) { return signals::Connection{std::move(from), std::move(to)}; }),
             py::arg("from_"), py::arg("to"))
        .def_readwrite("from_", &signals::Connection::from)
        .def_readwrite("to", &signals::Connection::to);
    py::class_<signals::ExternalInput>(m, "ExternalInput")
        .def(py::init([](std::string signal, double initial) { return signals::ExternalInput{std::move(signal), initial}; }),
             py::arg("signal"), py::arg("initial") = 0.0)
        .def_readwrite("signal", &signals::ExternalInput::signal)
        .def_readwrite("initial", &signals::ExternalInput::initial);
    py::class_<signals::RailSpec>(m, "RailSpec")
        .def(py::init([](std::string name, std::string supply, std::vector<std::string> gates,
                         std::vector<std::string> consumers) {
                 return signals::RailSpec{std::move(name), std::move(supply), std::move(gates), std::move(consumers)};
             }),
             py::arg("name"), py::arg("supply"), py::arg("gates") = std::vector<std::string>{},
             py::arg("consumers") = std::vector<std::string>{})
        .def_readwrite("name", &signals::RailSpec::name)
        .def_readwrite("supply", &signals::RailSpec::supply)
        .def_readwrite("gates", &signals::RailSpec::gates)
        .def_readwrite("consumers", &signals::RailSpec::consumers);
    py::class_<signals::ActuatorOutput>(m, "ActuatorOutput")
        .def(py::init([](std::string signal, std::size_t dof, std::string unit, bool applied) {
                 return signals::ActuatorOutput{std::move(signal), dof, std::move(unit), applied};
             }),
             py::arg("signal"), py::arg("dof"), py::arg("unit") = "N*m", py::arg("applied") = true)
        .def_readwrite("applied", &signals::ActuatorOutput::applied)
        .def_readwrite("signal", &signals::ActuatorOutput::signal)
        .def_readwrite("dof", &signals::ActuatorOutput::dof)
        .def_readwrite("unit", &signals::ActuatorOutput::unit);
    py::class_<signals::SignalLayerConfig>(m, "SignalLayerConfig", "Lists are copied in and out: assign them whole.")
        .def(py::init<>())
        .def_readwrite("name", &signals::SignalLayerConfig::name)
        .def_readwrite("physics_outputs", &signals::SignalLayerConfig::physics_outputs)
        .def_readwrite("pin_outputs", &signals::SignalLayerConfig::pin_outputs)
        .def_readwrite("rate", &signals::SignalLayerConfig::rate)
        .def_readwrite("controller_rate", &signals::SignalLayerConfig::controller_rate)
        .def_readwrite("seed", &signals::SignalLayerConfig::seed)
        .def_readwrite("bindings", &signals::SignalLayerConfig::bindings)
        .def_readwrite("slots", &signals::SignalLayerConfig::slots)
        .def_readwrite("connections", &signals::SignalLayerConfig::connections)
        .def_readwrite("externals", &signals::SignalLayerConfig::externals)
        .def_readwrite("rails", &signals::SignalLayerConfig::rails)
        .def_readwrite("controller_rail", &signals::SignalLayerConfig::controller_rail)
        .def_readwrite("actuators", &signals::SignalLayerConfig::actuators)
        .def_readwrite("velocity_dofs", &signals::SignalLayerConfig::velocity_dofs);
    py::class_<signals::SignalLayer, kernel::Party, py::smart_holder>(
        m, "SignalLayer", "The signal layer party (SIG-024); add components before registering it with a kernel.")
        .def(py::init<signals::SignalLayerConfig>(), py::arg("config"))
        .def("add_component", &signals::SignalLayer::add_component, py::arg("component"), py::arg("stage"),
             "Takes ownership; the Python handle is disowned.")
        .def("add_operator_link", &signals::SignalLayer::add_operator_link, py::arg("link"))
        .def("signal_value", &signals::SignalLayer::signal_value, py::arg("signal"))
        .def("rails", [](const signals::SignalLayer& layer) {
            return std::vector<pins::PowerRail>(layer.rails().begin(), layer.rails().end());
        })
        .def("pin_inputs", &signals::SignalLayer::pin_inputs)
        .def("physics_inputs", &signals::SignalLayer::physics_inputs);

    py::enum_<controller::PinSlot>(m, "PinSlot")
        .value("digital_in", controller::PinSlot::digital_in)
        .value("counter_delta", controller::PinSlot::counter_delta)
        .value("adc_sample", controller::PinSlot::adc_sample)
        .value("uart_rx", controller::PinSlot::uart_rx)
        .value("step_count", controller::PinSlot::step_count)
        .value("digital_out", controller::PinSlot::digital_out)
        .value("uart_tx", controller::PinSlot::uart_tx);
    py::class_<controller::ChannelAssignment>(m, "ChannelAssignment")
        .def(py::init([](std::string signal, controller::PinSlot slot, std::size_t channel) {
                 return controller::ChannelAssignment{std::move(signal), slot, channel};
             }),
             py::arg("signal"), py::arg("slot"), py::arg("channel"))
        .def_readwrite("signal", &controller::ChannelAssignment::signal)
        .def_readwrite("slot", &controller::ChannelAssignment::slot)
        .def_readwrite("channel", &controller::ChannelAssignment::channel);
    py::class_<controller::AxisChannels>(m, "AxisChannels")
        .def(py::init([](std::size_t dir, std::size_t enable) { return controller::AxisChannels{dir, enable}; }),
             py::arg("dir_channel"), py::arg("enable_channel"))
        .def_readwrite("dir_channel", &controller::AxisChannels::dir_channel)
        .def_readwrite("enable_channel", &controller::AxisChannels::enable_channel);
    py::class_<controller::ChannelMap>(m, "ChannelMap", "The builder's channel table (CTL-006).")
        .def(py::init<>())
        .def("add", &controller::ChannelMap::add, py::arg("assignment"))
        .def("find", &controller::ChannelMap::find, py::arg("signal"))
        .def("entries", [](const controller::ChannelMap& map) {
            return std::vector<controller::ChannelAssignment>(map.entries().begin(), map.entries().end());
        })
        .def("check_bindings", &controller::ChannelMap::check_bindings, py::arg("bindings"))
        .def("assign_axis", &controller::ChannelMap::assign_axis, py::arg("axis"), py::arg("channels"))
        .def("axis", &controller::ChannelMap::axis, py::arg("axis"));
    m.def("slot_capacity", &controller::slot_capacity, py::arg("slot"));
    m.def("slot_accepts", &controller::slot_accepts, py::arg("slot"), py::arg("mode"));

    m.def("board_interface_version", &controller::board_interface_version);
    m.attr("placeholder_brownout_threshold_v") = controller::placeholder_brownout_threshold_v;
    py::class_<controller::HostBuildRequest>(m, "HostBuildRequest")
        .def(py::init<>())
        .def_readwrite("firmware_tree", &controller::HostBuildRequest::firmware_tree)
        .def_readwrite("work_dir", &controller::HostBuildRequest::work_dir)
        .def_readwrite("tick_hz", &controller::HostBuildRequest::tick_hz)
        .def_readwrite("app_target", &controller::HostBuildRequest::app_target)
        .def_readwrite("cmake_command", &controller::HostBuildRequest::cmake_command);
    py::class_<controller::HostBuildResult>(m, "HostBuildResult")
        .def_readonly("library", &controller::HostBuildResult::library)
        .def_readonly("log", &controller::HostBuildResult::log);
    m.def("build_host_firmware", &controller::build_host_firmware, py::arg("request"),
          py::call_guard<py::gil_scoped_release>());
    py::class_<controller::NativeRuntimeConfig>(m, "NativeRuntimeConfig")
        .def(py::init<>())
        .def_readwrite("control_rate", &controller::NativeRuntimeConfig::control_rate)
        .def_readwrite("work_dir", &controller::NativeRuntimeConfig::work_dir)
        .def_readwrite("brownout_threshold_v", &controller::NativeRuntimeConfig::brownout_threshold_v)
        .def_readwrite("channels", &controller::NativeRuntimeConfig::channels)
        .def_readwrite("app_target", &controller::NativeRuntimeConfig::app_target)
        .def_readwrite("cmake_command", &controller::NativeRuntimeConfig::cmake_command);
    py::class_<controller::NativeControllerRuntime, controller::ControllerRuntime, py::smart_holder>(
        m, "NativeControllerRuntime", "Host-built firmware through the board interface (CTL-002 to CTL-007).")
        .def(py::init<controller::NativeRuntimeConfig>(), py::arg("config"))
        .def("channel_map", &controller::NativeControllerRuntime::channel_map)
        .def("library_path", &controller::NativeControllerRuntime::library_path)
        .def("build_log", &controller::NativeControllerRuntime::build_log)
        .def("ticks_run", &controller::NativeControllerRuntime::ticks_run)
        .def("inits", &controller::NativeControllerRuntime::inits)
        .def("frozen", &controller::NativeControllerRuntime::frozen)
        .def("held_in_reset", &controller::NativeControllerRuntime::held_in_reset)
        .def("supply_voltage", &controller::NativeControllerRuntime::supply_voltage);
}

}  // namespace mechsim::bindings
