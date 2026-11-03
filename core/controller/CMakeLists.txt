# Controller runtimes: runtime interface; native shim and Renode bridge from milestones 1 and 4.
add_library(mechsim_controller
    src/null_controller_runtime.cpp
)
add_library(mechsim::controller ALIAS mechsim_controller)
target_include_directories(mechsim_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mechsim_controller PUBLIC mechsim_signals PRIVATE mechsim_warnings)

add_executable(mechsim_controller_tests
    tests/test_controller_runtime.cpp
)
target_link_libraries(mechsim_controller_tests PRIVATE mechsim_controller mechsim_warnings Catch2::Catch2WithMain)
catch_discover_tests(mechsim_controller_tests)
