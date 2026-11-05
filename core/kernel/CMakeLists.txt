# Co-simulation kernel: time, rates, party interface, physics adapter interface, recorder sink.
add_library(mechsim_pins INTERFACE)
add_library(mechsim::pins ALIAS mechsim_pins)
target_include_directories(mechsim_pins INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/pins/include)

add_library(mechsim_kernel
    src/coupling.cpp
    src/cosim_kernel.cpp
    src/kernel.cpp
    src/memory_recorder_sink.cpp
    src/null_kernel.cpp
    src/null_party.cpp
    src/null_physics_adapter.cpp
    src/null_recorder_sink.cpp
    src/party.cpp
    src/physics_party.cpp
    src/time.cpp
)
add_library(mechsim::kernel ALIAS mechsim_kernel)
target_include_directories(mechsim_kernel PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mechsim_kernel PRIVATE mechsim_warnings)

add_executable(mechsim_kernel_tests
    tests/test_cosim_kernel.cpp
    tests/test_coupling.cpp
    tests/test_memory_recorder_sink.cpp
    tests/test_null_kernel.cpp
    tests/test_physics_adapter.cpp
    tests/test_physics_party.cpp
    tests/test_recorder_sink.cpp
    tests/test_time.cpp
)
target_link_libraries(mechsim_kernel_tests PRIVATE mechsim_kernel mechsim_warnings Catch2::Catch2WithMain)
catch_discover_tests(mechsim_kernel_tests)
