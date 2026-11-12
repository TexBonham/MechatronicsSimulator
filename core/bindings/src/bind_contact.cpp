#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <vector>

#include "bindings.hpp"
#include "mechsim/engine/collision.hpp"
#include "mechsim/engine/contact.hpp"
#include "mechsim/engine/multibody_provider.hpp"

namespace py = pybind11;

namespace mechsim::bindings {

namespace {

using Wxyz = std::array<double, 4>;

// Adds a read-only property to a class bound elsewhere in this module.
template <typename Getter>
void add_property(const py::object& cls, const char* name, Getter getter) {
    const py::object property = py::module_::import("builtins").attr("property");
    py::setattr(cls, name, property(py::cpp_function(getter)));
}

template <typename Function, typename... Extra>
void add_method(const py::object& cls, const char* name, Function function, const Extra&... extra) {
    py::setattr(cls, name,
                py::cpp_function(function, py::name(name), py::is_method(cls), py::sibling(py::getattr(cls, name, py::none())),
                                 extra...));
}

engine::Pose world_pose(const engine::Geometry& g) {
    engine::Pose pose;
    pose.rotation = g.orientation.toRotationMatrix();
    pose.position = g.position;
    return pose;
}

}  // namespace

void bind_contact(py::module_& m) {
    using engine::Geometry;
    using engine::MultibodyProvider;

    py::class_<engine::ContactParameters>(m, "ContactParameters", "Soft contact K_n, C_n and mu (ENG-020).")
        .def(py::init<>())
        .def_readwrite("stiffness", &engine::ContactParameters::stiffness)
        .def_readwrite("damping", &engine::ContactParameters::damping)
        .def_readwrite("friction", &engine::ContactParameters::friction);

    py::class_<engine::MaterialPair>(m, "MaterialPair")
        .def(py::init<>())
        .def_readwrite("material_a", &engine::MaterialPair::material_a)
        .def_readwrite("material_b", &engine::MaterialPair::material_b)
        .def_readwrite("parameters", &engine::MaterialPair::parameters);

    py::class_<Geometry>(m, "Geometry", "Contact geometry in the neutral model's terms; body -1 is the world.")
        .def(py::init<>())
        .def_readwrite("kind", &Geometry::kind)
        .def_readwrite("body", &Geometry::body)
        .def_readwrite("position", &Geometry::position)
        .def_property(
            "orientation",
            [](const Geometry& g) {
                return Wxyz{g.orientation.w(), g.orientation.x(), g.orientation.y(), g.orientation.z()};
            },
            [](Geometry& g, const Wxyz& q) { g.orientation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]); },
            "[w, x, y, z], unit")
        .def_readwrite("radius", &Geometry::radius)
        .def_readwrite("length", &Geometry::length)
        .def_readwrite("size", &Geometry::size)
        .def_readwrite("vertices", &Geometry::vertices)
        .def_readwrite("material", &Geometry::material);

    py::class_<engine::ContactModel>(m, "ContactModel")
        .def(py::init<>())
        .def_readwrite("geometries", &engine::ContactModel::geometries)
        .def_readwrite("default_parameters", &engine::ContactModel::default_parameters)
        .def_readwrite("pairs", &engine::ContactModel::pairs)
        .def_readwrite("exclusions", &engine::ContactModel::exclusions)
        .def_readwrite("slip_velocity", &engine::ContactModel::slip_velocity);

    py::class_<engine::CollisionLimits>(m, "CollisionLimits", "GJK and EPA caps and tolerance (ENG-018).")
        .def(py::init<>())
        .def_readwrite("gjk_iterations", &engine::CollisionLimits::gjk_iterations)
        .def_readwrite("epa_iterations", &engine::CollisionLimits::epa_iterations)
        .def_readwrite("tolerance", &engine::CollisionLimits::tolerance);

    py::class_<engine::ContactPoint>(m, "ContactPoint")
        .def_readonly("point", &engine::ContactPoint::point)
        .def_readonly("normal", &engine::ContactPoint::normal)
        .def_readonly("depth", &engine::ContactPoint::depth);

    py::class_<engine::PayloadOutput>(m, "PayloadOutput")
        .def_readonly("body", &engine::PayloadOutput::body)
        .def_readonly("carrier", &engine::PayloadOutput::carrier)
        .def_readonly("attached", &engine::PayloadOutput::attached)
        .def_readonly("force", &engine::PayloadOutput::force);

    m.def(
        "collide",
        [](const Geometry& a, const Geometry& b, const engine::CollisionLimits& limits) {
            return engine::collide(engine::to_shape(a, 0), world_pose(a), engine::to_shape(b, 1), world_pose(b),
                                   limits);
        },
        py::arg("a"), py::arg("b"), py::arg("limits") = engine::CollisionLimits{}, "Narrow phase on two geometries whose poses are world poses (ENG-016).");

    const py::object contact_output = m.attr("ContactOutput");
    add_property(contact_output, "normal", [](const engine::ContactOutput& c) { return c.normal; });
    add_property(contact_output, "depth", [](const engine::ContactOutput& c) { return c.depth; });
    add_property(m.attr("MultibodyOutputs"), "payloads",
                 [](const engine::MultibodyOutputs& o) { return o.payloads; });

    const py::object provider = m.attr("MultibodyProvider");
    add_method(provider, "set_contact_model", &MultibodyProvider::set_contact_model, py::arg("contacts"));
    add_property(provider, "contact_model", [](const MultibodyProvider& p) { return p.contact_model(); });
    add_method(provider, "contacts", &MultibodyProvider::contacts, py::arg("q"), py::arg("v"));
    add_method(provider, "set_body_mass", &MultibodyProvider::set_body_mass, py::arg("body"), py::arg("mass"));
    add_method(provider, "set_attachment", &MultibodyProvider::set_attachment, py::arg("body"), py::arg("carrier"),
               py::arg("attached"), py::arg("q"), py::arg("frequency") = engine::WeldAttachment::default_frequency);
}

}  // namespace mechsim::bindings
