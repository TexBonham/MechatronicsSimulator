cmake_minimum_required(VERSION 3.28)
project(mechsim_engine LANGUAGES CXX)

if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/MechsimWarnings.cmake)
    include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/Catch2.cmake)
    enable_testing()
endif()
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/Eigen.cmake)

add_library(mechsim_engine
    src/engine_error.cpp
    src/multibody_provider.cpp
    src/semi_implicit_euler.cpp
)
add_library(mechsim::engine ALIAS mechsim_engine)
target_include_directories(mechsim_engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mechsim_engine PUBLIC Eigen3::Eigen PRIVATE mechsim_warnings)

add_executable(mechsim_engine_tests
    tests/test_constraints.cpp
    tests/test_errors.cpp
    tests/test_joints.cpp
    tests/test_multibody_errors.cpp
    tests/test_integrator.cpp
    tests/test_step_config.cpp
)
target_link_libraries(mechsim_engine_tests PRIVATE mechsim_engine mechsim_warnings Catch2::Catch2WithMain)
catch_discover_tests(mechsim_engine_tests)
