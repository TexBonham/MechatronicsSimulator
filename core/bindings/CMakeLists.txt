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
set_target_properties(mechsim_engine mechsim_kernel mechsim_signals mechsim_controller mechsim_physics_mujoco mechsim_physics_native
    mechsim_integrity PROPERTIES POSITION_INDEPENDENT_CODE ON)

pybind11_add_module(_core_ext MODULE
    src/bind_contact.cpp
    src/bind_cosim.cpp
    src/bind_controller.cpp
    src/bind_engine.cpp
    src/bind_integrity.cpp
    src/bind_kernel.cpp
    src/bind_physics.cpp
    src/bind_recorder.cpp
    src/bind_signals.cpp
    src/errors.cpp
    src/factories.cpp
    src/module.cpp
)
target_include_directories(_core_ext PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(_core_ext PRIVATE mechsim_engine mechsim_controller mechsim_physics_mujoco mechsim_physics_native
    mechsim_integrity
    mechsim_warnings)
# libmujoco ships next to the extension in the mechsim package; the installed extension finds it through $ORIGIN, the
# build tree keeps its own RPATH to the fetched release.
set_target_properties(_core_ext PROPERTIES INSTALL_RPATH "$ORIGIN" BUILD_WITH_INSTALL_RPATH OFF)

install(TARGETS _core_ext LIBRARY DESTINATION mechsim COMPONENT python)
install(FILES $<TARGET_FILE:mujoco::mujoco> DESTINATION mechsim COMPONENT python)
