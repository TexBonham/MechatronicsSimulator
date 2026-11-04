#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "bindings.hpp"
#include "factories.hpp"
#include "mechsim/kernel/physics_adapter.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"
#include "mechsim/physics/mujoco/mujoco_adapter.hpp"

namespace py = pybind11;

namespace mechsim::bindings {

namespace {

using kernel::PhysicsAdapter;

// A Python adapter for development and tests; the stepping adapters are C++ (ADR 0014) and register by name.
class PyPhysicsAdapter : public PhysicsAdapter, public py::trampoline_self_life_support {
public:
    [[nodiscard]] std::span<const std::string> supported_elements() const override {
        supported_ = call_override<std::vector<std::string>>(this, "supported_elements");
        return supported_;
    }

    void load(const kernel::PreparedModel& model, const kernel::Rate& physics_rate) override {
        call_override<void>(this, "load", kernel::PreparedModel(model), kernel::Rate(physics_rate));
    }

    void reset() override { call_override<void>(this, "reset"); }
    void step() override { call_override<void>(this, "step"); }

    [[nodiscard]] const kernel::PhysicsOutputs& outputs() const override {
        outputs_ = call_override<kernel::PhysicsOutputs>(this, "outputs");
        return outputs_;
    }

    [[nodiscard]] kernel::PhysicsHealth health() const override {
        return call_override<kernel::PhysicsHealth>(this, "health");
    }

    void set_inputs(const kernel::PhysicsInputs& inputs) override {
        call_override<void>(this, "set_inputs", kernel::PhysicsInputs(inputs));
    }

    void set_spring_offset(std::size_t spring, double offset) override {
        call_override<void>(this, "set_spring_offset", spring, offset);
    }
    void set_spring_stiffness(std::size_t spring, double stiffness) override {
        call_override<void>(this, "set_spring_stiffness", spring, stiffness);
    }
    void disable_constraint(std::size_t constraint) override {
        call_override<void>(this, "disable_constraint", constraint);
    }
    void release_load(std::size_t load) override { call_override<void>(this, "release_load", load); }
    void set_payload_mass(std::size_t payload, double mass) override {
        call_override<void>(this, "set_payload_mass", payload, mass);
    }
    void set_payload_attachment(std::size_t payload, std::size_t body) override {
        call_override<void>(this, "set_payload_attachment", payload, body);
    }

private:
    mutable std::vector<std::string> supported_;
    mutable kernel::PhysicsOutputs outputs_;
};

void bind_types(py::module_& m) {
    py::class_<kernel::Wrench>(m, "Wrench")
        .def(py::init<>())
        .def_readwrite("moment", &kernel::Wrench::moment)
        .def_readwrite("force", &kernel::Wrench::force);
    py::class_<kernel::BodyState>(m, "BodyState")
        .def(py::init<>())
        .def_readwrite("position", &kernel::BodyState::position)
        .def_readwrite("orientation", &kernel::BodyState::orientation)
        .def_readwrite("angular_velocity", &kernel::BodyState::angular_velocity)
        .def_readwrite("linear_velocity", &kernel::BodyState::linear_velocity);
    py::class_<kernel::JointForce>(m, "JointForce")
        .def(py::init<>())
        .def_readwrite("generalized_force", &kernel::JointForce::generalized_force)
        .def_readwrite("reaction", &kernel::JointForce::reaction);
    py::class_<kernel::ConstraintForce>(m, "ConstraintForce")
        .def(py::init<>())
        .def_readwrite("force", &kernel::ConstraintForce::force)
        .def_readwrite("translational_error", &kernel::ConstraintForce::translational_error)
        .def_readwrite("rotational_error", &kernel::ConstraintForce::rotational_error);
    py::class_<kernel::ContactForce>(m, "ContactForce")
        .def(py::init<>())
        .def_readwrite("body_a", &kernel::ContactForce::body_a)
        .def_readwrite("body_b", &kernel::ContactForce::body_b)
        .def_readwrite("point", &kernel::ContactForce::point)
        .def_readwrite("force", &kernel::ContactForce::force);
    py::class_<kernel::MemberEndLoads>(m, "MemberEndLoads")
        .def(py::init<>())
        .def_readwrite("start", &kernel::MemberEndLoads::start)
        .def_readwrite("end", &kernel::MemberEndLoads::end);
    py::class_<kernel::RopeTension>(m, "RopeTension")
        .def(py::init<>())
        .def_readwrite("tension", &kernel::RopeTension::tension)
        .def_readwrite("slack", &kernel::RopeTension::slack);
    py::enum_<kernel::PowerFlow>(m, "PowerFlow")
        .value("Static", kernel::PowerFlow::Static)
        .value("Forward", kernel::PowerFlow::Forward)
        .value("Backward", kernel::PowerFlow::Backward)
        .value("Locked", kernel::PowerFlow::Locked);
    py::class_<kernel::TransmissionState>(m, "TransmissionState")
        .def(py::init<>())
        .def_readwrite("slip", &kernel::TransmissionState::slip)
        .def_readwrite("slipping", &kernel::TransmissionState::slipping)
        .def_readwrite("shaft_load_torque", &kernel::TransmissionState::shaft_load_torque)
        .def_readwrite("force", &kernel::TransmissionState::force)
        .def_readwrite("stretch", &kernel::TransmissionState::stretch)
        .def_readwrite("power_flow", &kernel::TransmissionState::power_flow);
    py::class_<kernel::PhysicsHealth>(m, "PhysicsHealth")
        .def(py::init<>())
        .def_readwrite("max_constraint_drift", &kernel::PhysicsHealth::max_constraint_drift)
        .def_readwrite("converged", &kernel::PhysicsHealth::converged)
        .def_readwrite("iterations", &kernel::PhysicsHealth::iterations)
        .def_readwrite("steps", &kernel::PhysicsHealth::steps)
        .def_readwrite("physics_rate", &kernel::PhysicsHealth::physics_rate);
    py::class_<kernel::PhysicsOutputs>(m, "PhysicsOutputs", "Published every tick (PHY-003, PHY-007); SI, world frame.")
        .def(py::init<>())
        .def_readwrite("steps", &kernel::PhysicsOutputs::steps)
        .def_readwrite("bodies", &kernel::PhysicsOutputs::bodies)
        .def_readwrite("joints", &kernel::PhysicsOutputs::joints)
        .def_readwrite("constraints", &kernel::PhysicsOutputs::constraints)
        .def_readwrite("contacts", &kernel::PhysicsOutputs::contacts)
        .def_readwrite("members", &kernel::PhysicsOutputs::members)
        .def_readwrite("ropes", &kernel::PhysicsOutputs::ropes)
        .def_readwrite("transmissions", &kernel::PhysicsOutputs::transmissions);
    py::class_<kernel::BodyWrench>(m, "BodyWrench")
        .def(py::init<>())
        .def_readwrite("body", &kernel::BodyWrench::body)
        .def_readwrite("wrench", &kernel::BodyWrench::wrench);
    py::class_<kernel::PhysicsInputs>(m, "PhysicsInputs", "Applied for the next step only (ENG-028, KER-010).")
        .def(py::init<>())
        .def_readwrite("generalized_forces", &kernel::PhysicsInputs::generalized_forces)
        .def_readwrite("body_wrenches", &kernel::PhysicsInputs::body_wrenches);
    py::class_<kernel::PreparedModel>(m, "PreparedModel")
        .def(py::init([](std::string json, std::string artifact, std::string version) {
                 return kernel::PreparedModel{std::move(json), std::move(artifact), std::move(version)};
             }),
             py::arg("neutral_model_json") = "", py::arg("engine_artifact") = "", py::arg("design_version") = "")
        .def_readwrite("neutral_model_json", &kernel::PreparedModel::neutral_model_json)
        .def_readwrite("engine_artifact", &kernel::PreparedModel::engine_artifact)
        .def_readwrite("design_version", &kernel::PreparedModel::design_version);
}

void bind_adapter(py::module_& m) {
    py::class_<PhysicsAdapter, PyPhysicsAdapter, py::smart_holder>(
        m, "PhysicsAdapter", "One stepping interface for every engine (PHY-001, ADR 0014).")
        .def(py::init<>())
        .def("supported_elements",
             [](const PhysicsAdapter& adapter) {
                 const auto view = adapter.supported_elements();
                 return std::vector<std::string>(view.begin(), view.end());
             })
        .def("load", &PhysicsAdapter::load, py::arg("model"), py::arg("physics_rate"))
        .def("reset", &PhysicsAdapter::reset)
        .def("step", &PhysicsAdapter::step)
        .def("outputs", &PhysicsAdapter::outputs, py::return_value_policy::copy)
        .def("health", &PhysicsAdapter::health)
        .def("set_inputs", &PhysicsAdapter::set_inputs, py::arg("inputs"))
        .def("set_spring_offset", &PhysicsAdapter::set_spring_offset, py::arg("spring"), py::arg("offset"))
        .def("set_spring_stiffness", &PhysicsAdapter::set_spring_stiffness, py::arg("spring"), py::arg("stiffness"))
        .def("disable_constraint", &PhysicsAdapter::disable_constraint, py::arg("constraint"))
        .def("release_load", &PhysicsAdapter::release_load, py::arg("load"))
        .def("set_payload_mass", &PhysicsAdapter::set_payload_mass, py::arg("payload"), py::arg("mass"))
        .def("set_payload_attachment", &PhysicsAdapter::set_payload_attachment, py::arg("payload"),
             py::arg("body"));

    py::class_<physics::mujoco::MujocoAdapter, PhysicsAdapter, py::smart_holder>(
        m, "MujocoAdapter", "MuJoCo behind the PhysicsAdapter interface (PHY-014, ADR 0014); load the translation's MJCF.")
        .def(py::init<>())
        .def("integrator", &physics::mujoco::MujocoAdapter::integrator)
        .def("timestep", &physics::mujoco::MujocoAdapter::timestep);

    py::class_<kernel::NullPhysicsAdapter, PhysicsAdapter, py::smart_holder>(m, "NullPhysicsAdapter")
        .def(py::init<>())
        .def_property_readonly("loaded", &kernel::NullPhysicsAdapter::loaded)
        .def_property_readonly("design_version", &kernel::NullPhysicsAdapter::design_version);

    m.def(
        "physics_adapter_step",
        [](PhysicsAdapter& adapter) {
            adapter.step();
            return adapter.outputs();
        },
        py::arg("adapter"),
        "Steps an adapter once through the C++ interface and returns a copy of its outputs, as a kernel party would.");

    m.def("physics_adapter_names", &physics_adapter_names, "Registered adapter names, in name order.");
    m.def("make_physics_adapter", &make_physics_adapter, py::arg("name"), "A registered adapter by name.");
}

}  // namespace

void bind_physics(py::module_& m) {
    bind_types(m);
    bind_adapter(m);
}

}  // namespace mechsim::bindings
