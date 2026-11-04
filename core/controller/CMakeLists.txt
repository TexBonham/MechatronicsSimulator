# Controller runtimes: runtime interface, native shim (milestone 1); Renode bridge from milestone 4.
set(MECHSIM_BOARD_INTERFACE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../board_interface)

add_library(mechsim_controller
    src/channel_map.cpp
    src/host_build.cpp
    src/native_controller_runtime.cpp
    src/null_controller_runtime.cpp
)
add_library(mechsim::controller ALIAS mechsim_controller)
target_include_directories(mechsim_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(mechsim_controller SYSTEM PRIVATE ${MECHSIM_BOARD_INTERFACE_DIR})
target_link_libraries(mechsim_controller PUBLIC mechsim_signals PRIVATE mechsim_warnings ${CMAKE_DL_LIBS})

add_executable(mechsim_controller_tests
    tests/test_channel_map.cpp
    tests/test_controller_runtime.cpp
)
target_link_libraries(mechsim_controller_tests PRIVATE mechsim_controller mechsim_warnings Catch2::Catch2WithMain)
catch_discover_tests(mechsim_controller_tests)

# Test applications for the shim, one source with variants: matching, wrong version, no version symbol, 250 Hz.
function(mechsim_controller_mirror_app name)
    add_library(${name} SHARED tests/fixtures/mirror_app.cpp)
    target_include_directories(${name} SYSTEM PRIVATE ${MECHSIM_BOARD_INTERFACE_DIR})
    target_link_libraries(${name} PRIVATE mechsim_warnings)
    target_compile_definitions(${name} PRIVATE ${ARGN})
endfunction()
mechsim_controller_mirror_app(mechsim_controller_mirror_app)
mechsim_controller_mirror_app(mechsim_controller_mirror_bad_version MIRROR_VERSION_OFFSET=1U)
mechsim_controller_mirror_app(mechsim_controller_mirror_no_version MIRROR_NO_VERSION)
mechsim_controller_mirror_app(mechsim_controller_mirror_250hz MIRROR_TICK_HZ=250U)

set(MECHSIM_CONTROLLER_NATIVE_WORK ${CMAKE_CURRENT_BINARY_DIR}/native_work)
add_executable(mechsim_controller_native_tests
    tests/native_test_support.cpp
    tests/test_native_build.cpp
    tests/test_native_shim.cpp
)
target_include_directories(mechsim_controller_native_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(mechsim_controller_native_tests
    PRIVATE mechsim_controller mechsim_warnings Catch2::Catch2WithMain)
add_dependencies(mechsim_controller_native_tests
    mechsim_controller_mirror_app mechsim_controller_mirror_bad_version mechsim_controller_mirror_no_version
    mechsim_controller_mirror_250hz)
target_compile_definitions(mechsim_controller_native_tests PRIVATE
    MECHSIM_TEST_FIRMWARE_TREE="${CMAKE_CURRENT_SOURCE_DIR}/../../firmware"
    MECHSIM_TEST_WORK_DIR="${MECHSIM_CONTROLLER_NATIVE_WORK}"
    MECHSIM_TEST_CMAKE="${CMAKE_COMMAND}"
    MECHSIM_TEST_MIRROR_APP="$<TARGET_FILE:mechsim_controller_mirror_app>"
    MECHSIM_TEST_MIRROR_BAD_VERSION="$<TARGET_FILE:mechsim_controller_mirror_bad_version>"
    MECHSIM_TEST_MIRROR_NO_VERSION="$<TARGET_FILE:mechsim_controller_mirror_no_version>"
    MECHSIM_TEST_MIRROR_250HZ="$<TARGET_FILE:mechsim_controller_mirror_250hz>")
# One real host build of the reference stub per test binary: a ctest fixture runs the hidden build test first and
# every other test loads the library it left in the work directory.
add_test(NAME mechsim_controller_native_reference_build
    COMMAND mechsim_controller_native_tests "[native-build]")
set_tests_properties(mechsim_controller_native_reference_build PROPERTIES FIXTURES_SETUP controller_native_reference)
catch_discover_tests(mechsim_controller_native_tests
    PROPERTIES FIXTURES_REQUIRED controller_native_reference)
