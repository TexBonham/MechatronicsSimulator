# Own-engine adapter (PHY-020, ADR 0014): kernel::PhysicsAdapter over core/engine/. Its public header names no
# engine type.
add_library(mechsim_physics_native
    src/json.cpp
    src/native_adapter.cpp
)
add_library(mechsim::physics_native ALIAS mechsim_physics_native)
target_include_directories(mechsim_physics_native PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mechsim_physics_native
    PUBLIC mechsim_kernel
    PRIVATE mechsim_engine mechsim_warnings
)

add_executable(mechsim_physics_native_tests
    tests/test_native_adapter.cpp
)
target_include_directories(mechsim_physics_native_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mechsim_physics_native_tests PRIVATE
    mechsim_physics_native mechsim_warnings Catch2::Catch2WithMain)
catch_discover_tests(mechsim_physics_native_tests TEST_PREFIX "native.")
catch_discover_tests(mechsim_physics_native_tests TEST_SPEC "[reference]" TEST_PREFIX "reference.native."
    PROPERTIES LABELS reference)
