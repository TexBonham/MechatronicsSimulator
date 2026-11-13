#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <string>
#include <utility>
#include <vector>

#include "bindings.hpp"
#include "mechsim/integrity/beam.hpp"
#include "mechsim/integrity/integrity_party.hpp"
#include "mechsim/integrity/limits.hpp"
#include "mechsim/kernel/party.hpp"
#include "mechsim/kernel/physics_party.hpp"
#include "mechsim/kernel/physics_types.hpp"
#include "mechsim/kernel/time.hpp"

namespace py = pybind11;

namespace mechsim::bindings {

namespace {

integrity::Limits parsed(const std::string& json) {
    try {
        return integrity::parse_limits(json);
    } catch (const integrity::LimitsError& error) {
        throw py::value_error(error.what());
    }
}

}  // namespace

void bind_integrity(py::module_& m) {
    py::class_<kernel::ModelChangeRequest>(m, "ModelChangeRequest", "A mid-run model change request (KER-028).")
        .def(py::init([](kernel::ModelChangeKind kind, std::size_t index, double value) {
                 return kernel::ModelChangeRequest{kind, index, value};
             }),
             py::arg("kind"), py::arg("index"), py::arg("value") = 0.0)
        .def_readonly("kind", &kernel::ModelChangeRequest::kind)
        .def_readonly("index", &kernel::ModelChangeRequest::index)
        .def_readonly("value", &kernel::ModelChangeRequest::value);
    // Read-only value copy: Python reads it from a row's coupling column (the recorder digests every coupling value,
    // REC-022) and never writes it; only the integrity party publishes it, to physics (invariant 1: no controller sees
    // it).
    py::class_<kernel::ModelChangeRequests>(m, "ModelChangeRequests",
                                            "One published list of model change requests (KER-028), a copy.")
        .def(py::init([](std::int64_t sequence, std::vector<kernel::ModelChangeRequest> requests) {
                 return kernel::ModelChangeRequests{sequence, std::move(requests)};
             }),
             py::arg("sequence"), py::arg("requests"))
        .def_readonly("sequence", &kernel::ModelChangeRequests::sequence)
        .def_readonly("requests", &kernel::ModelChangeRequests::requests);
    py::class_<integrity::Crossing>(m, "IntegrityCrossingRecord", "One tier 1 limit crossing (INT-024).")
        .def_readonly("time", &integrity::Crossing::time)
        .def_readonly("element", &integrity::Crossing::element)
        .def_readonly("quantity", &integrity::Crossing::quantity)
        .def_readonly("value", &integrity::Crossing::value)
        .def_readonly("limit", &integrity::Crossing::limit);
    py::class_<integrity::IntegrityTrace, py::smart_holder>(
        m, "IntegrityTrace", "What the tier 1 party saw, readable after the kernel owns the party (INT-023).")
        .def(py::init<>())
        .def_readonly("peak_stress", &integrity::IntegrityTrace::peak_stress)
        .def_readonly("peak_deflection", &integrity::IntegrityTrace::peak_deflection)
        .def_readonly("min_buckling_margin", &integrity::IntegrityTrace::min_buckling_margin)
        .def_readonly("peak_fastener_load", &integrity::IntegrityTrace::peak_fastener_load)
        .def_readonly("crossings", &integrity::IntegrityTrace::crossings)
        .def_readonly("before_evaluation", &integrity::IntegrityTrace::before_evaluation)
        .def_readonly("requests", &integrity::IntegrityTrace::requests);
    py::class_<integrity::IntegrityPartyConfig>(m, "IntegrityPartyConfig")
        .def(py::init<>())
        .def_readwrite("name", &integrity::IntegrityPartyConfig::name)
        .def_readwrite("rate", &integrity::IntegrityPartyConfig::rate)
        .def_readwrite("physics_outputs", &integrity::IntegrityPartyConfig::physics_outputs)
        .def_readwrite("halt_on_crossing", &integrity::IntegrityPartyConfig::halt_on_crossing)
        .def_readwrite("evaluate_from", &integrity::IntegrityPartyConfig::evaluate_from);
    py::class_<integrity::IntegrityParty, kernel::Party, py::smart_holder>(
        m, "IntegrityParty", "The tier 1 integrity party over a limits table (JSON, mechsim.integrity.tier1).")
        .def(py::init([](const std::string& limits, integrity::IntegrityPartyConfig config,
                         std::shared_ptr<integrity::IntegrityTrace> trace) {
                 try {
                     return std::make_unique<integrity::IntegrityParty>(parsed(limits), std::move(config),
                                                                        std::move(trace));
                 } catch (const integrity::LimitsError& error) {
                     throw py::value_error(error.what());
                 }
             }),
             py::arg("limits"), py::arg("config") = integrity::IntegrityPartyConfig{},
             py::arg("trace") = std::shared_ptr<integrity::IntegrityTrace>{});
    m.def(
        "integrity_member_stress",
        [](const std::string& limits, std::size_t member, const kernel::MemberEndLoads& loads,
           const std::vector<kernel::BodyState>& bodies) {
            const integrity::Limits table = parsed(limits);
            if (member >= table.members.size()) {
                throw py::index_error("member " + std::to_string(member) + " of " +
                                      std::to_string(table.members.size()));
            }
            try {
                return std::make_pair(integrity::peak_normal_stress(loads, bodies, table.members[member]),
                                      integrity::node_moments(bodies, table.members[member]));
            } catch (const integrity::LimitsError& error) {
                throw py::value_error(error.what());
            }
        },
        py::arg("limits"), py::arg("member"), py::arg("loads"), py::arg("bodies"),
        "Tier 1 peak normal stress (Pa) and node spring moments (N m) of one member of a limits table on one tick's "
        "end-loads and body poses (INT-005, INT-023).");
    m.def("check_integrity_limits", [](const std::string& limits) { static_cast<void>(parsed(limits)); },
          py::arg("limits"), "Parse a limits table; ValueError naming the field when malformed.");
}

}  // namespace mechsim::bindings
