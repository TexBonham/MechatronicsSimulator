# pybind11 binding exposing the core runtime to Python as mechsim._core_ext (ADR 0005). scikit-build-core builds
# only this target (pyproject.toml); the presets build it with everything else.
option(MECHSIM_BUILD_BINDINGS "Build the Python binding mechsim._core_ext" ON)
if(NOT MECHSIM_BUILD_BINDINGS)
    return()
endif()

find_package(Python 3.12 REQUIRED COMPONENTS Interpreter Development.Module)

include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v3.0.1
    GIT_SHALLOW TRUE
    SYSTEM
)
FetchContent_MakeAvailable(pybind11)

# The core libraries are static; linking them into a shared extension module needs position-independent code.
set_target_properties(mechsim_engine mechsim_kernel mechsim_signals mechsim_controller
    PROPERTIES POSITION_INDEPENDENT_CODE ON)

pybind11_add_module(_core_ext MODULE
    src/bind_controller.cpp
    src/bind_engine.cpp
    src/bind_kernel.cpp
    src/bind_physics.cpp
    src/bind_recorder.cpp
    src/bind_signals.cpp
    src/errors.cpp
    src/factories.cpp
    src/module.cpp
)
target_include_directories(_core_ext PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(_core_ext PRIVATE mechsim_engine mechsim_controller mechsim_warnings)

install(TARGETS _core_ext LIBRARY DESTINATION mechsim COMPONENT python)
