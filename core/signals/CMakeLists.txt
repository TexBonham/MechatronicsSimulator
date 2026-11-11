# Signal layer: component plugin interface, built-in components, pin bindings, power rails, the signal-layer party.
add_library(mechsim_signals
    src/component_base.cpp
    src/front_end.cpp
    src/operator_link.cpp
    src/pin_binding.cpp
    src/safety.cpp
    src/sensors.cpp
    src/signal_component.cpp
    src/signal_layer.cpp
    src/signal_name.cpp
    src/stepper.cpp
)
add_library(mechsim::signals ALIAS mechsim_signals)
# The layer presents PinInputs and collects PinOutputs, plain structs in the controller's header (no controller
# symbol is linked); mechsim_controller links this target, so the dependency runs header-only the other way.
target_include_directories(mechsim_signals PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mechsim_signals PUBLIC mechsim_kernel mechsim_pins PRIVATE mechsim_warnings)

add_executable(mechsim_signals_tests
    tests/test_pin_binding.cpp
    tests/test_signal_component.cpp
    tests/test_sensors.cpp
    tests/test_stepper.cpp
    tests/test_safety.cpp
    tests/test_operator_link.cpp
    tests/test_signal_layer.cpp
    tests/test_m2_components.cpp
)
target_link_libraries(mechsim_signals_tests PRIVATE mechsim_signals mechsim_warnings Catch2::Catch2WithMain)
catch_discover_tests(mechsim_signals_tests)

# Analytical reference tier (T1): `ctest --preset debug-sanitize -L reference` runs exactly these.
add_executable(mechsim_signals_reference
    tests/test_reference_encoder.cpp
    tests/test_reference_m2_sensors.cpp
)
target_link_libraries(mechsim_signals_reference PRIVATE mechsim_signals mechsim_warnings Catch2::Catch2WithMain)
catch_discover_tests(mechsim_signals_reference PROPERTIES LABELS reference)
