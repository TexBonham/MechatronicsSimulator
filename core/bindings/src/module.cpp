#include <pybind11/pybind11.h>

#include "bindings.hpp"

PYBIND11_MODULE(_core_ext, m) {
    m.doc() = "mechsim core runtime: kernel, physics adapters, signal layer, controller runtimes, engine (ADR 0005)";
    mechsim::bindings::bind_errors(m);
    mechsim::bindings::bind_time(m);
    mechsim::bindings::bind_recorder(m);
    mechsim::bindings::bind_kernel(m);
    mechsim::bindings::bind_physics(m);
    mechsim::bindings::bind_signals(m);
    mechsim::bindings::bind_controller(m);
    mechsim::bindings::bind_cosim(m);
    mechsim::bindings::bind_integrity(m);
    // Engine types live in their own submodule so the import-boundary check can keep them behind
    // mechsim/physics/native/ (invariant 5); registering it in sys.modules makes it importable by its dotted name.
    pybind11::module_ engine =
        m.def_submodule("engine", "The own engine (ADR 0001); import only from mechsim/physics/native/.");
    mechsim::bindings::bind_engine(engine);
    pybind11::module_::import("sys").attr("modules")[engine.attr("__name__")] = engine;
}
