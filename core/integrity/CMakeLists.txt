# Tier 1 structural integrity party (INT-023 to INT-028, ADR 0010): a kernel party over the physics outputs.
add_library(mechsim_integrity
    src/beam.cpp
    src/integrity_party.cpp
    src/limits.cpp
)
add_library(mechsim::integrity ALIAS mechsim_integrity)
target_include_directories(mechsim_integrity PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mechsim_integrity PUBLIC mechsim_kernel PRIVATE mechsim_warnings)

add_executable(mechsim_integrity_tests
    tests/test_integrity_party.cpp
    tests/test_integrity_native.cpp
)
target_link_libraries(mechsim_integrity_tests PRIVATE
    mechsim_integrity mechsim_physics_native mechsim_warnings Catch2::Catch2WithMain)
catch_discover_tests(mechsim_integrity_tests TEST_PREFIX "integrity.")
