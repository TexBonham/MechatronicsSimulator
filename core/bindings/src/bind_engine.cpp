#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <utility>

#include "bindings.hpp"
#include "mechsim/engine/integrator.hpp"
#include "mechsim/engine/lumped_beam.hpp"
#include "mechsim/engine/member_loads.hpp"
#include "mechsim/engine/multibody_model.hpp"
#include "mechsim/engine/multibody_provider.hpp"
#include "mechsim/engine/rope.hpp"
#include "mechsim/engine/transmission.hpp"
#include "mechsim/engine/state.hpp"
#include "mechsim/engine/step_config.hpp"
#include "mechsim/engine/step_health.hpp"

namespace py = pybind11;

namespace mechsim::bindings {

namespace {

using Wxyz = std::array<double, 4>;

// pybind11 has no Eigen quaternion caster; quaternions cross as [w, x, y, z], the neutral model's order.
Wxyz to_wxyz(const Eigen::Quaterniond& q) {
    return {q.w(), q.x(), q.y(), q.z()};
}

Eigen::Quaterniond from_wxyz(const Wxyz& q) {
    return {q[0], q[1], q[2], q[3]};
}

void bind_model(py::module_& m) {
    using engine::Body;
    using engine::Joint;
    using engine::LoopConstraint;
    using engine::MultibodyConfig;
    using engine::MultibodyModel;

    py::enum_<engine::JointType>(m, "JointType")
        .value("Revolute", engine::JointType::Revolute)
        .value("Prismatic", engine::JointType::Prismatic)
        .value("Fixed", engine::JointType::Fixed)
        .value("Planar", engine::JointType::Planar)
        .value("Spherical", engine::JointType::Spherical)
        .value("Free", engine::JointType::Free)
        .value("Translation", engine::JointType::Translation);

    py::class_<Joint>(m, "Joint",
                      "Joint frame in the parent frame; springs (ENG-051), limits (ENG-013), friction (ENG-014).")
        .def(py::init<>())
        .def_readwrite("type", &Joint::type)
        .def_readwrite("position", &Joint::position)
        .def_property(
            "orientation", [](const Joint& j) { return to_wxyz(j.orientation); },
            [](Joint& j, const Wxyz& q) { j.orientation = from_wxyz(q); }, "[w, x, y, z], unit")
        .def_readwrite("axis", &Joint::axis)
        .def_readwrite("stiffness", &Joint::stiffness)
        .def_readwrite("damping", &Joint::damping)
        .def_readwrite("rest", &Joint::rest)
        .def_readwrite("lower", &Joint::lower)
        .def_readwrite("upper", &Joint::upper)
        .def_readwrite("friction", &Joint::friction);

    py::class_<Body>(m, "Body")
        .def(py::init<>())
        .def_readwrite("name", &Body::name)
        .def_readwrite("parent", &Body::parent)
        .def_readwrite("joint", &Body::joint)
        .def_readwrite("mass", &Body::mass)
        .def_readwrite("com", &Body::com)
        .def_readwrite("inertia", &Body::inertia);

    py::enum_<engine::ConstraintType>(m, "ConstraintType")
        .value("Point", engine::ConstraintType::Point)
        .value("Hinge", engine::ConstraintType::Hinge);

    py::class_<LoopConstraint>(m, "LoopConstraint")
        .def(py::init<>())
        .def_readwrite("type", &LoopConstraint::type)
        .def_readwrite("body_a", &LoopConstraint::body_a)
        .def_readwrite("body_b", &LoopConstraint::body_b)
        .def_readwrite("point_a", &LoopConstraint::point_a)
        .def_readwrite("point_b", &LoopConstraint::point_b)
        .def_readwrite("axis_a", &LoopConstraint::axis_a)
        .def_readwrite("axis_b", &LoopConstraint::axis_b);

    py::enum_<engine::TransmissionType>(m, "TransmissionType")
        .value("BeltPulley", engine::TransmissionType::BeltPulley)
        .value("LeadScrew", engine::TransmissionType::LeadScrew)
        .value("GearPair", engine::TransmissionType::GearPair);
    py::enum_<engine::PowerFlow>(m, "PowerFlow")
        .value("Static", engine::PowerFlow::Static)
        .value("Forward", engine::PowerFlow::Forward)
        .value("Backward", engine::PowerFlow::Backward)
        .value("Locked", engine::PowerFlow::Locked);
    py::class_<engine::Transmission>(m, "Transmission", "Belt, lead screw or gear pair (ENG-043 to ENG-046).")
        .def(py::init<>())
        .def_readwrite("type", &engine::Transmission::type)
        .def_readwrite("input_body", &engine::Transmission::input_body)
        .def_readwrite("output_body", &engine::Transmission::output_body)
        .def_readwrite("ratio", &engine::Transmission::ratio)
        .def_readwrite("reverses", &engine::Transmission::reverses)
        .def_readwrite("efficiency", &engine::Transmission::efficiency)
        .def_readwrite("back_efficiency", &engine::Transmission::back_efficiency)
        .def_readwrite("self_locking", &engine::Transmission::self_locking)
        .def_readwrite("stiffness", &engine::Transmission::stiffness)
        .def_readwrite("damping", &engine::Transmission::damping)
        .def_readwrite("backlash", &engine::Transmission::backlash)
        .def_readwrite("slip", &engine::Transmission::slip);
    py::class_<engine::TransmissionOutput>(m, "TransmissionOutput",
                                           "shaft_load_torque: torque the shaft must supply (PHY-033, ENG-048).")
        .def_readonly("force", &engine::TransmissionOutput::force)
        .def_readonly("shaft_load_torque", &engine::TransmissionOutput::shaft_load_torque)
        .def_readonly("stretch", &engine::TransmissionOutput::stretch)
        .def_readonly("backlash_position", &engine::TransmissionOutput::backlash_position)
        .def_readonly("in_contact", &engine::TransmissionOutput::in_contact)
        .def_readonly("slip", &engine::TransmissionOutput::slip)
        .def_readonly("slip_active", &engine::TransmissionOutput::slip_active)
        .def_readonly("slip_offset", &engine::TransmissionOutput::slip_offset)
        .def_readonly("power_flow", &engine::TransmissionOutput::power_flow)
        .def_readonly("scheme_decay", &engine::TransmissionOutput::scheme_decay, "pi omega h (ruling 21)");

    py::class_<engine::Rope>(m, "Rope", "Lumped rope chain (ENG-047, ADR 0016); add_rope fills nodes.")
        .def(py::init<>())
        .def_readwrite("from_body", &engine::Rope::from_body)
        .def_readwrite("from_point", &engine::Rope::from_point)
        .def_readwrite("to_body", &engine::Rope::to_body)
        .def_readwrite("to_point", &engine::Rope::to_point)
        .def_readwrite("segments", &engine::Rope::segments)
        .def_readwrite("axial_rigidity", &engine::Rope::axial_rigidity)
        .def_readwrite("damping", &engine::Rope::damping)
        .def_readwrite("mass_per_length", &engine::Rope::mass_per_length)
        .def_readwrite("rest_length", &engine::Rope::rest_length)
        .def_readwrite("breaking_tension", &engine::Rope::breaking_tension)
        .def_readwrite("attachment_damping", &engine::Rope::attachment_damping)
        .def_readwrite("attachment_friction", &engine::Rope::attachment_friction)
        .def_readonly("nodes", &engine::Rope::nodes);

    py::class_<MultibodyModel>(m, "MultibodyModel",
                               "Bodies in topological order (ENG-005); assign lists whole, they are copied.")
        .def(py::init<>())
        .def_readwrite("bodies", &MultibodyModel::bodies)
        .def_readwrite("constraints", &MultibodyModel::constraints)
        .def_readwrite("transmissions", &MultibodyModel::transmissions)
        .def_readwrite("members", &MultibodyModel::members)
        .def_readwrite("ropes", &MultibodyModel::ropes)
        .def_readwrite("gravity", &MultibodyModel::gravity);

    py::class_<MultibodyConfig>(m, "MultibodyConfig")
        .def(py::init<>())
        .def_readwrite("iteration_cap", &MultibodyConfig::iteration_cap)
        .def_readwrite("velocity_tolerance", &MultibodyConfig::velocity_tolerance)
        .def_readwrite("projection_iterations", &MultibodyConfig::projection_iterations)
        .def_readwrite("drift_threshold_m", &MultibodyConfig::drift_threshold_m)
        .def_readwrite("drift_threshold_rad", &MultibodyConfig::drift_threshold_rad)
        .def_readwrite("unit_tolerance", &MultibodyConfig::unit_tolerance)
        .def_readwrite("friction_slip_threshold", &MultibodyConfig::friction_slip_threshold);

    m.def(
        "add_rope",
        [](MultibodyModel& model, const engine::Rope& rope, const Eigen::Vector3d& from_world,
           const Eigen::Vector3d& to_world) { return engine::add_rope(model, rope, from_world, to_world); },
        py::arg("model"), py::arg("rope"), py::arg("from_world"), py::arg("to_world"),
        "Appends the rope's nodes and the rope to model in place; returns the rope index.");
}

void bind_outputs(py::module_& m) {
    py::class_<engine::BodyOutput>(m, "BodyOutput")
        .def_readonly("position", &engine::BodyOutput::position)
        .def_property_readonly("orientation", [](const engine::BodyOutput& b) { return to_wxyz(b.orientation); })
        .def_readonly("spatial_velocity", &engine::BodyOutput::spatial_velocity)
        .def_readonly("origin_velocity", &engine::BodyOutput::origin_velocity);
    py::class_<engine::JointOutput>(m, "JointOutput")
        .def_readonly("generalized_force", &engine::JointOutput::generalized_force)
        .def_readonly("reaction_wrench", &engine::JointOutput::reaction_wrench)
        .def_readonly("limit_force", &engine::JointOutput::limit_force)
        .def_readonly("limit_overshoot", &engine::JointOutput::limit_overshoot)
        .def_readonly("friction_creep", &engine::JointOutput::friction_creep)
        .def_readonly("scheme_decay", &engine::JointOutput::scheme_decay);
    py::class_<engine::ConstraintOutput>(m, "ConstraintOutput")
        .def_readonly("impulse", &engine::ConstraintOutput::impulse)
        .def_readonly("force", &engine::ConstraintOutput::force)
        .def_readonly("torque", &engine::ConstraintOutput::torque)
        .def_readonly("translational_error", &engine::ConstraintOutput::translational_error)
        .def_readonly("rotational_error", &engine::ConstraintOutput::rotational_error);
    py::class_<engine::ContactOutput>(m, "ContactOutput")
        .def_readonly("body_a", &engine::ContactOutput::body_a)
        .def_readonly("body_b", &engine::ContactOutput::body_b)
        .def_readonly("point", &engine::ContactOutput::point)
        .def_readonly("force", &engine::ContactOutput::force);
    py::class_<engine::RopeOutput>(m, "RopeOutput")
        .def_readonly("tension", &engine::RopeOutput::tension)
        .def_readonly("max_tension", &engine::RopeOutput::max_tension)
        .def_readonly("load_end_tension", &engine::RopeOutput::load_end_tension)
        .def_readonly("attached", &engine::RopeOutput::attached)
        .def_readonly("rest_length", &engine::RopeOutput::rest_length)
        .def_readonly("snatch_omega_h", &engine::RopeOutput::snatch_omega_h)
        .def_readonly("snatch_underestimated", &engine::RopeOutput::snatch_underestimated)
        .def_readonly("scheme_decay", &engine::RopeOutput::scheme_decay);
    py::class_<engine::MemberOutput>(m, "MemberOutput", "Support-on-member end wrench, root frame axes (ENG-030).")
        .def_readonly("start_moment", &engine::MemberOutput::start_moment)
        .def_readonly("start_force", &engine::MemberOutput::start_force)
        .def_readonly("end_moment", &engine::MemberOutput::end_moment)
        .def_readonly("end_force", &engine::MemberOutput::end_force);
    py::enum_<engine::ModelChange>(m, "ModelChange")
        .value("SpringOffset", engine::ModelChange::SpringOffset)
        .value("SpringStiffness", engine::ModelChange::SpringStiffness)
        .value("ConstraintDisabled", engine::ModelChange::ConstraintDisabled)
        .value("LoadReleased", engine::ModelChange::LoadReleased)
        .value("RopeSnapped", engine::ModelChange::RopeSnapped)
        .value("PayloadAttached", engine::ModelChange::PayloadAttached)
        .value("PayloadMass", engine::ModelChange::PayloadMass);
    py::enum_<engine::LoadSource>(m, "LoadSource")
        .value("Constraint", engine::LoadSource::Constraint)
        .value("Rope", engine::LoadSource::Rope)
        .value("Payload", engine::LoadSource::Payload);
    py::class_<engine::ModelChangeEvent>(m, "ModelChangeEvent", "Mid-run change or snap (ENG-031 to ENG-035).")
        .def_readonly("kind", &engine::ModelChangeEvent::kind)
        .def_readonly("target", &engine::ModelChangeEvent::target)
        .def_readonly("index", &engine::ModelChangeEvent::index)
        .def_readonly("value", &engine::ModelChangeEvent::value)
        .def_readonly("limit", &engine::ModelChangeEvent::limit)
        .def_readonly("time", &engine::ModelChangeEvent::time)
        .def_readonly("source", &engine::ModelChangeEvent::source);
    py::class_<engine::MultibodyOutputs>(m, "MultibodyOutputs")
        .def_readonly("bodies", &engine::MultibodyOutputs::bodies)
        .def_readonly("joints", &engine::MultibodyOutputs::joints)
        .def_readonly("constraints", &engine::MultibodyOutputs::constraints)
        .def_readonly("contacts", &engine::MultibodyOutputs::contacts)
        .def_readonly("transmissions", &engine::MultibodyOutputs::transmissions)
        .def_readonly("members", &engine::MultibodyOutputs::members)
        .def_readonly("ropes", &engine::MultibodyOutputs::ropes)
        .def_readonly("events", &engine::MultibodyOutputs::events)
        .def_readonly("time", &engine::MultibodyOutputs::time);
    py::class_<engine::ConstraintError>(m, "ConstraintError")
        .def_readonly("residual", &engine::ConstraintError::residual)
        .def_readonly("translational", &engine::ConstraintError::translational)
        .def_readonly("rotational", &engine::ConstraintError::rotational);
}

void bind_stepping(py::module_& m) {
    using engine::MultibodyProvider;

    py::class_<engine::StepConfig>(m, "StepConfig", "Step length h, s (ENG-002); symmetry tolerance (ENG-049).")
        .def(py::init<>())
        .def(py::init([](double h, double symmetry_tolerance) { return engine::StepConfig{h, symmetry_tolerance}; }),
             py::arg("h"), py::arg("symmetry_tolerance") = engine::StepConfig::default_symmetry_tolerance)
        .def_readwrite("h", &engine::StepConfig::h)
        .def_readwrite("symmetry_tolerance", &engine::StepConfig::symmetry_tolerance);

    py::class_<engine::State>(m, "State", "Generalized positions q, velocities v, time t (s).")
        .def(py::init<>())
        .def(py::init([](Eigen::VectorXd q, Eigen::VectorXd v, double t) {
                 return engine::State{std::move(q), std::move(v), t};
             }),
             py::arg("q"), py::arg("v"), py::arg("t") = 0.0)
        .def_readwrite("q", &engine::State::q)
        .def_readwrite("v", &engine::State::v)
        .def_readwrite("t", &engine::State::t);

    py::class_<engine::StepHealth>(m, "StepHealth", "Per-step health (ENG-026).")
        .def_readonly("max_constraint_drift", &engine::StepHealth::max_constraint_drift)
        .def_readonly("converged", &engine::StepHealth::converged)
        .def_readonly("iterations", &engine::StepHealth::iterations)
        .def_readonly("time", &engine::StepHealth::time)
        .def_readonly("step", &engine::StepHealth::step);

    py::class_<engine::Integrator>(m, "Integrator")
        .def_property_readonly("config", &engine::Integrator::config, py::return_value_policy::copy);
    py::class_<engine::SemiImplicitEuler, engine::Integrator>(m, "SemiImplicitEuler", "ADR 0013 option A (ENG-003).")
        .def(py::init<engine::StepConfig>(), py::arg("config") = engine::StepConfig{});

    py::class_<MultibodyProvider>(m, "MultibodyProvider", "Reduced-coordinate rigid multibody (ADR 0013).")
        .def(py::init<engine::MultibodyModel, engine::MultibodyConfig>(), py::arg("model"),
             py::arg("config") = engine::MultibodyConfig{})
        .def("position_size", &MultibodyProvider::position_size)
        .def("velocity_size", &MultibodyProvider::velocity_size)
        .def_property_readonly("model", &MultibodyProvider::model, py::return_value_policy::copy)
        .def_property_readonly("config", &MultibodyProvider::config, py::return_value_policy::copy)
        .def("position_offset", &MultibodyProvider::position_offset, py::arg("body"))
        .def("velocity_offset", &MultibodyProvider::velocity_offset, py::arg("body"))
        .def("neutral_positions", &MultibodyProvider::neutral_positions)
        .def("set_joint_drive", &MultibodyProvider::set_joint_drive, py::arg("joint"), py::arg("dof"),
             py::arg("rest"), py::arg("stiffness"), py::arg("damping"))
        .def("set_shaft_torque", &MultibodyProvider::set_shaft_torque, py::arg("transmission"), py::arg("torque"))
        .def("set_slip", &MultibodyProvider::set_slip, py::arg("transmission"), py::arg("slip"))
        .def("slip", &MultibodyProvider::slip, py::arg("transmission"))
        .def("transmission_states", &MultibodyProvider::transmission_states, py::arg("q"), py::arg("v"))
        .def("constraint_errors", &MultibodyProvider::constraint_errors, py::arg("q"))
        .def("body_states", &MultibodyProvider::body_states, py::arg("q"), py::arg("v"))
        .def("set_generalized_forces", &MultibodyProvider::set_generalized_forces, py::arg("tau"))
        .def("apply_body_wrench", &MultibodyProvider::apply_body_wrench, py::arg("body"), py::arg("torque"),
             py::arg("force"))
        .def("clear_inputs", &MultibodyProvider::clear_inputs)
        .def("set_spring_offset", &MultibodyProvider::set_spring_offset, py::arg("member"), py::arg("node"),
             py::arg("offset"))
        .def("set_spring_stiffness", &MultibodyProvider::set_spring_stiffness, py::arg("member"), py::arg("node"),
             py::arg("stiffness"))
        .def("disable_constraint", &MultibodyProvider::disable_constraint, py::arg("constraint"))
        .def("release_load", py::overload_cast<Eigen::Index>(&MultibodyProvider::release_load),
             py::arg("constraint"))
        .def("release_rope", &MultibodyProvider::release_rope, py::arg("rope"))
        .def("constraint_enabled", &MultibodyProvider::constraint_enabled, py::arg("constraint"))
        .def("rope_attached", &MultibodyProvider::rope_attached, py::arg("rope"))
        .def("set_rope_rest_length", &MultibodyProvider::set_rope_rest_length, py::arg("rope"), py::arg("length"))
        .def("rope_states", &MultibodyProvider::rope_states, py::arg("q"), py::arg("v"))
        .def("step", &MultibodyProvider::step, py::arg("integrator"), py::arg("state"),
             "Advances state in place by one step; state is unchanged when it raises.")
        .def_property_readonly("outputs", &MultibodyProvider::outputs, py::return_value_policy::copy);
}

void bind_beam(py::module_& m) {
    using engine::BeamSpec;
    using engine::LumpedBeam;

    py::enum_<engine::BeamEnds>(m, "BeamEnds")
        .value("Cantilever", engine::BeamEnds::Cantilever)
        .value("SimplySupported", engine::BeamEnds::SimplySupported)
        .value("Clamped", engine::BeamEnds::Clamped);

    py::class_<BeamSpec>(m, "BeamSpec", "Lumped flexible member (ENG-022): N rigid segments, springs E I / l.")
        .def(py::init<>())
        .def_readwrite("youngs_modulus", &BeamSpec::youngs_modulus)
        .def_readwrite("area_moment", &BeamSpec::area_moment)
        .def_readwrite("density", &BeamSpec::density)
        .def_readwrite("area", &BeamSpec::area)
        .def_readwrite("length", &BeamSpec::length)
        .def_readwrite("segments", &BeamSpec::segments)
        .def_readwrite("node_damping", &BeamSpec::node_damping)
        .def_readwrite("ends", &BeamSpec::ends)
        .def_readwrite("roller_mass_fraction", &BeamSpec::roller_mass_fraction);

    py::class_<engine::BeamPoint>(m, "BeamPoint")
        .def_readonly("body", &engine::BeamPoint::body)
        .def_readonly("point", &engine::BeamPoint::point);

    py::class_<LumpedBeam>(m, "LumpedBeam")
        .def_readonly("parent", &LumpedBeam::parent)
        .def_readonly("position", &LumpedBeam::position)
        .def_property_readonly("orientation", [](const LumpedBeam& b) { return to_wxyz(b.orientation); })
        .def_readonly("length", &LumpedBeam::length)
        .def_readonly("node_stiffness", &LumpedBeam::node_stiffness)
        .def_readonly("bodies", &LumpedBeam::bodies)
        .def_readonly("starts", &LumpedBeam::starts)
        .def_readonly("roller", &LumpedBeam::roller)
        .def_readonly("roller_constraint", &LumpedBeam::roller_constraint)
        .def_readonly("pinned_root", &LumpedBeam::pinned_root);
    m.def("member_nodes", &engine::member_nodes, py::arg("beam"), "Spring nodes root to tip (ENG-031, ENG-032).");

    m.def(
        "add_lumped_beam",
        [](engine::MultibodyModel& model, const BeamSpec& spec, Eigen::Index parent, const Eigen::Vector3d& position,
           const Wxyz& orientation) {
            return engine::add_lumped_beam(model, spec, parent, position, from_wxyz(orientation));
        },
        py::arg("model"), py::arg("spec"), py::arg("parent") = -1, py::arg("position") = Eigen::Vector3d::Zero(),
        py::arg("orientation") = Wxyz{1.0, 0.0, 0.0, 0.0}, "Appends the beam to model in place and describes it.");
    m.def("beam_point", &engine::beam_point, py::arg("beam"), py::arg("x"));
}

}  // namespace

void bind_engine(py::module_& m) {
    bind_model(m);
    bind_outputs(m);
    bind_stepping(m);
    bind_contact(m);
    bind_beam(m);
}

}  // namespace mechsim::bindings
