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
#include "mechsim/engine/multibody_model.hpp"
#include "mechsim/engine/multibody_provider.hpp"
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
        .value("Free", engine::JointType::Free);

    py::class_<Joint>(m, "Joint", "Joint frame in the parent frame; springs and dampers per DOF (ENG-051).")
        .def(py::init<>())
        .def_readwrite("type", &Joint::type)
        .def_readwrite("position", &Joint::position)
        .def_property(
            "orientation", [](const Joint& j) { return to_wxyz(j.orientation); },
            [](Joint& j, const Wxyz& q) { j.orientation = from_wxyz(q); }, "[w, x, y, z], unit")
        .def_readwrite("axis", &Joint::axis)
        .def_readwrite("stiffness", &Joint::stiffness)
        .def_readwrite("damping", &Joint::damping)
        .def_readwrite("rest", &Joint::rest);

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

    py::class_<MultibodyModel>(m, "MultibodyModel",
                               "Bodies in topological order (ENG-005); assign lists whole, they are copied.")
        .def(py::init<>())
        .def_readwrite("bodies", &MultibodyModel::bodies)
        .def_readwrite("constraints", &MultibodyModel::constraints)
        .def_readwrite("gravity", &MultibodyModel::gravity);

    py::class_<MultibodyConfig>(m, "MultibodyConfig")
        .def(py::init<>())
        .def_readwrite("iteration_cap", &MultibodyConfig::iteration_cap)
        .def_readwrite("velocity_tolerance", &MultibodyConfig::velocity_tolerance)
        .def_readwrite("projection_iterations", &MultibodyConfig::projection_iterations)
        .def_readwrite("drift_threshold_m", &MultibodyConfig::drift_threshold_m)
        .def_readwrite("drift_threshold_rad", &MultibodyConfig::drift_threshold_rad)
        .def_readwrite("unit_tolerance", &MultibodyConfig::unit_tolerance);
}

void bind_outputs(py::module_& m) {
    py::class_<engine::BodyOutput>(m, "BodyOutput")
        .def_readonly("position", &engine::BodyOutput::position)
        .def_property_readonly("orientation", [](const engine::BodyOutput& b) { return to_wxyz(b.orientation); })
        .def_readonly("spatial_velocity", &engine::BodyOutput::spatial_velocity)
        .def_readonly("origin_velocity", &engine::BodyOutput::origin_velocity);
    py::class_<engine::JointOutput>(m, "JointOutput")
        .def_readonly("generalized_force", &engine::JointOutput::generalized_force)
        .def_readonly("reaction_wrench", &engine::JointOutput::reaction_wrench);
    py::class_<engine::ConstraintOutput>(m, "ConstraintOutput")
        .def_readonly("impulse", &engine::ConstraintOutput::impulse)
        .def_readonly("force", &engine::ConstraintOutput::force)
        .def_readonly("translational_error", &engine::ConstraintOutput::translational_error)
        .def_readonly("rotational_error", &engine::ConstraintOutput::rotational_error);
    py::class_<engine::ContactOutput>(m, "ContactOutput")
        .def_readonly("body_a", &engine::ContactOutput::body_a)
        .def_readonly("body_b", &engine::ContactOutput::body_b)
        .def_readonly("point", &engine::ContactOutput::point)
        .def_readonly("force", &engine::ContactOutput::force);
    py::class_<engine::MultibodyOutputs>(m, "MultibodyOutputs")
        .def_readonly("bodies", &engine::MultibodyOutputs::bodies)
        .def_readonly("joints", &engine::MultibodyOutputs::joints)
        .def_readonly("constraints", &engine::MultibodyOutputs::constraints)
        .def_readonly("contacts", &engine::MultibodyOutputs::contacts)
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
        .def("constraint_errors", &MultibodyProvider::constraint_errors, py::arg("q"))
        .def("body_states", &MultibodyProvider::body_states, py::arg("q"), py::arg("v"))
        .def("set_generalized_forces", &MultibodyProvider::set_generalized_forces, py::arg("tau"))
        .def("apply_body_wrench", &MultibodyProvider::apply_body_wrench, py::arg("body"), py::arg("torque"),
             py::arg("force"))
        .def("clear_inputs", &MultibodyProvider::clear_inputs)
        .def("step", &MultibodyProvider::step, py::arg("integrator"), py::arg("state"),
             "Advances state in place by one step; state is unchanged when it raises.")
        .def_property_readonly("outputs", &MultibodyProvider::outputs, py::return_value_policy::copy);
}

void bind_beam(py::module_& m) {
    using engine::BeamSpec;
    using engine::LumpedBeam;

    py::enum_<engine::BeamEnds>(m, "BeamEnds")
        .value("Cantilever", engine::BeamEnds::Cantilever)
        .value("SimplySupported", engine::BeamEnds::SimplySupported);

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
        .def_readonly("roller_constraint", &LumpedBeam::roller_constraint);

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
    bind_beam(m);
}

}  // namespace mechsim::bindings
