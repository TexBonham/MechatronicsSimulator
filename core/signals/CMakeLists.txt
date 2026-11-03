# Signal layer: component plugin interface, pin bindings, power rails.
add_library(mechsim_signals
    src/pin_binding.cpp
    src/signal_component.cpp
    src/signal_name.cpp
)
add_library(mechsim::signals ALIAS mechsim_signals)
target_include_directories(mechsim_signals PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mechsim_signals PUBLIC mechsim_kernel PRIVATE mechsim_warnings)

add_executable(mechsim_signals_tests
    tests/test_pin_binding.cpp
    tests/test_signal_component.cpp
)
target_link_libraries(mechsim_signals_tests PRIVATE mechsim_signals mechsim_warnings Catch2::Catch2WithMain)
catch_discover_tests(mechsim_signals_tests)
