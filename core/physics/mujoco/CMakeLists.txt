# MuJoCo stepping adapter (ADR 0014, PHY-014). The only target that links MuJoCo; its public header names no MuJoCo type.
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/MuJoCo.cmake)

add_library(mechsim_physics_mujoco
    src/mujoco_adapter.cpp
)
add_library(mechsim::physics_mujoco ALIAS mechsim_physics_mujoco)
target_include_directories(mechsim_physics_mujoco PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mechsim_physics_mujoco
    PUBLIC mechsim_kernel
    PRIVATE mujoco::mujoco mechsim_warnings
)

add_executable(mechsim_physics_mujoco_tests
    tests/test_mujoco_adapter.cpp
)
target_compile_definitions(mechsim_physics_mujoco_tests PRIVATE
    MECHSIM_MUJOCO_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
target_link_libraries(mechsim_physics_mujoco_tests PRIVATE
    mechsim_physics_mujoco mechsim_signals mechsim_warnings Catch2::Catch2WithMain)
catch_discover_tests(mechsim_physics_mujoco_tests)
catch_discover_tests(mechsim_physics_mujoco_tests TEST_SPEC "[reference]" TEST_PREFIX "reference." PROPERTIES LABELS reference)
