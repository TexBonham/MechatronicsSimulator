#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "bindings.hpp"
#include "factories.hpp"
#include "mechsim/kernel/kernel.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/recorder_sink.hpp"
#include "mechsim/kernel/run_config.hpp"
#include "mechsim/kernel/time.hpp"

namespace py = pybind11;

namespace mechsim::bindings {

namespace {

using kernel::Duration;
using kernel::Party;
using kernel::Rate;
using kernel::Seconds;
using kernel::SimTime;

// The slow path (ADR 0005, KER-019): a Python subclass of Party is a Party to the kernel. Views the kernel holds
// (name, coupling lists) are copied out of Python into members so they outlive the call.
class PyParty : public Party, public py::trampoline_self_life_support {
public:
    [[nodiscard]] std::string_view name() const override {
        name_ = call_override<std::string>(this, "name");
        return name_;
    }

    [[nodiscard]] Rate nominal_rate() const override { return call_override<Rate>(this, "nominal_rate"); }

    [[nodiscard]] std::span<const std::string> published() const override {
        published_ = call_override<std::vector<std::string>>(this, "published");
        return published_;
    }

    [[nodiscard]] std::span<const std::string> consumed() const override {
        consumed_ = call_override<std::vector<std::string>>(this, "consumed");
        return consumed_;
    }

    void start(const Rate& base_rate) override { call_override<void>(this, "start", Rate(base_rate)); }

    void step(SimTime t, Duration dt) override { call_override<void>(this, "step", t, dt); }

    // A Python error here cannot propagate (the kernel is already ending the run), so it is reported as unraisable.
    // Anything else (allocation failure) has no recovery and ends the process, as noexcept would.
    void on_fault(const kernel::PartyFault& fault) noexcept override {
        try {
            const py::gil_scoped_acquire gil;
            try {
                const py::function override = py::get_override(static_cast<const Party*>(this), "on_fault");
                if (!static_cast<bool>(override)) {
                    py::set_error(PyExc_RuntimeError, "Python subclass does not implement on_fault()");
                    throw py::error_already_set();
                }
                override(party_fault_object(fault));
            } catch (py::error_already_set& error) {
                error.discard_as_unraisable("mechsim._core_ext.Party.on_fault");
            }
        } catch (...) {
            std::terminate();
        }
    }

private:
    mutable std::string name_;
    mutable std::vector<std::string> published_;
    mutable std::vector<std::string> consumed_;
};

std::string rate_repr(const Rate& rate) {
    if (rate.denominator() == 1) {
        return "Rate(" + std::to_string(rate.numerator()) + ")";
    }
    return "Rate(" + std::to_string(rate.numerator()) + ", " + std::to_string(rate.denominator()) + ")";
}

void bind_party(py::module_& m) {
    py::class_<Party, PyParty, py::smart_holder>(m, "Party",
                                                 "One interface for every party (KER-018); subclass it in Python for "
                                                 "the slow path (KER-019). Call super().__init__().")
        .def(py::init<>())
        .def("name", [](const Party& party) { return std::string(party.name()); })
        .def("nominal_rate", &Party::nominal_rate)
        .def("published",
             [](const Party& party) {
                 const auto view = party.published();
                 return std::vector<std::string>(view.begin(), view.end());
             })
        .def("consumed",
             [](const Party& party) {
                 const auto view = party.consumed();
                 return std::vector<std::string>(view.begin(), view.end());
             })
        .def("start", &Party::start, py::arg("base_rate"))
        .def("step", &Party::step, py::arg("t"), py::arg("dt"))
        .def(
            "on_fault",
            [](Party&, const py::object&) { throw std::logic_error("Python subclass does not implement on_fault()"); },
            py::arg("fault"));
}

void bind_run_config(py::module_& m) {
    py::enum_<kernel::ControllerMode>(m, "ControllerMode")
        .value("native", kernel::ControllerMode::native)
        .value("emulated", kernel::ControllerMode::emulated);

    py::class_<kernel::SolverConfig>(m, "SolverConfig")
        .def(py::init<>())
        .def_readwrite("iteration_cap", &kernel::SolverConfig::iteration_cap)
        .def_readwrite("velocity_tolerance", &kernel::SolverConfig::velocity_tolerance)
        .def_readwrite("projection_iterations", &kernel::SolverConfig::projection_iterations);

    py::class_<kernel::DriftThresholds>(m, "DriftThresholds")
        .def(py::init<>())
        .def_readwrite("translational_m", &kernel::DriftThresholds::translational_m)
        .def_readwrite("rotational_rad", &kernel::DriftThresholds::rotational_rad);

    py::class_<kernel::RunConfig>(m, "RunConfig", "Run configuration (KER-004, KER-005, KER-012); rates are data.")
        .def(py::init<>())
        .def_readwrite("physics_rate", &kernel::RunConfig::physics_rate)
        .def_readwrite("control_rate", &kernel::RunConfig::control_rate)
        .def_readwrite("party_rates", &kernel::RunConfig::party_rates)
        .def_readwrite("base_rate", &kernel::RunConfig::base_rate)
        .def_readwrite("seed", &kernel::RunConfig::seed)
        .def_readwrite("controller_mode", &kernel::RunConfig::controller_mode)
        .def_readwrite("solver", &kernel::RunConfig::solver)
        .def_readwrite("drift", &kernel::RunConfig::drift);

    py::class_<kernel::EndCondition>(m, "EndCondition",
                                     "End at end_time, or earlier when stop(t) returns True at a sync point (KER-017).")
        .def(py::init([](const Seconds& end_time, std::function<bool(SimTime)> stop) {
                 return kernel::EndCondition{end_time, std::move(stop)};
             }),
             py::arg("end_time"), py::arg("stop") = py::none())
        .def_readwrite("end_time", &kernel::EndCondition::end_time)
        .def_readwrite("stop", &kernel::EndCondition::stop);

    py::enum_<kernel::RunOutcome>(m, "RunOutcome")
        .value("end_condition_met", kernel::RunOutcome::end_condition_met)
        .value("run_error", kernel::RunOutcome::run_error);

    py::class_<kernel::RunReport>(m, "RunReport", "How the run ended (KER-015, KER-017).")
        .def_readonly("outcome", &kernel::RunReport::outcome)
        .def_readonly("base_rate", &kernel::RunReport::base_rate)
        .def_readonly("end_time", &kernel::RunReport::end_time)
        .def_readonly("ticks", &kernel::RunReport::ticks)
        .def_property_readonly("fault", [](const kernel::RunReport& report) -> py::object {
            if (!report.fault.has_value()) {
                return py::none();
            }
            return party_fault_object(report.fault.value());
        });
}

void bind_kernel_class(py::module_& m) {
    py::class_<kernel::Kernel, py::smart_holder>(m, "Kernel", "Owns simulated time and steps parties (KER-001).")
        .def("register_party", &kernel::Kernel::register_party, py::arg("party"),
             "Takes ownership of the party; a Python party is kept alive by the kernel.")
        .def("run", &kernel::Kernel::run, py::arg("end"), py::arg("sink"), py::call_guard<py::gil_scoped_release>(),
             "Runs to the end condition with the GIL released; Python parties and callbacks take it per call.")
        .def("now", &kernel::Kernel::now)
        .def_property_readonly("config", [](const kernel::Kernel& k) { return k.config(); });

    py::class_<kernel::NullKernel, kernel::Kernel, py::smart_holder>(m, "NullKernel")
        .def(py::init<kernel::RunConfig>(), py::arg("config") = kernel::RunConfig{});

    m.def("make_kernel", &make_kernel, py::arg("config") = kernel::RunConfig{},
          "The current kernel implementation for a run configuration.");
}

}  // namespace

void bind_time(py::module_& m) {
    py::class_<Rate>(m, "Rate", "Exact rational hertz, numerator / denominator, stored reduced.")
        .def(py::init<>())
        .def(py::init<std::int64_t, std::int64_t>(), py::arg("numerator"), py::arg("denominator") = 1)
        .def_property_readonly("numerator", &Rate::numerator)
        .def_property_readonly("denominator", &Rate::denominator)
        .def(py::self == py::self)
        .def("__hash__", [](const Rate& rate) { return py::hash(py::make_tuple(rate.numerator(), rate.denominator())); })
        .def("__repr__", &rate_repr);

    py::class_<Seconds>(m, "Seconds", "Exact non-negative rational seconds.")
        .def(py::init<>())
        .def(py::init<std::int64_t, std::int64_t>(), py::arg("numerator"), py::arg("denominator") = 1)
        .def_property_readonly("numerator", &Seconds::numerator)
        .def_property_readonly("denominator", &Seconds::denominator)
        .def(py::self == py::self)
        .def("__repr__", [](const Seconds& s) {
            return "Seconds(" + std::to_string(s.numerator()) + ", " + std::to_string(s.denominator()) + ")";
        });

    py::class_<Duration>(m, "Duration", "An interval in base ticks.")
        .def(py::init<>())
        .def(py::init<std::int64_t>(), py::arg("ticks"))
        .def("count", &Duration::count)
        .def(py::self == py::self)
        .def(py::self < py::self)
        .def(py::self <= py::self)
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def("__hash__", [](const Duration& d) { return py::hash(py::int_(d.count())); })
        .def("__repr__", [](const Duration& d) { return "Duration(" + std::to_string(d.count()) + ")"; });

    py::class_<SimTime>(m, "SimTime", "Simulated time in base ticks of 1 / base rate.")
        .def(py::init<>())
        .def(py::init<std::int64_t>(), py::arg("ticks"))
        .def("count", &SimTime::count)
        .def(py::self == py::self)
        .def(py::self < py::self)
        .def(py::self <= py::self)
        .def(py::self + Duration())
        .def(py::self - py::self)
        .def("__hash__", [](const SimTime& t) { return py::hash(py::int_(t.count())); })
        .def("__repr__", [](const SimTime& t) { return "SimTime(" + std::to_string(t.count()) + ")"; });

    m.def("lcm", &kernel::lcm, py::arg("a"), py::arg("b"));
    m.def("period_in_ticks", &kernel::period_in_ticks, py::arg("rate"), py::arg("base"));
    m.def("ticks_at", &kernel::ticks_at, py::arg("time"), py::arg("base"));
    m.def("to_seconds", py::overload_cast<Duration, const Rate&>(&kernel::to_seconds), py::arg("d"), py::arg("base"));
    m.def("to_seconds", py::overload_cast<SimTime, const Rate&>(&kernel::to_seconds), py::arg("t"), py::arg("base"));
}

void bind_kernel(py::module_& m) {
    bind_party(m);
    bind_run_config(m);
    bind_kernel_class(m);
}

}  // namespace mechsim::bindings
