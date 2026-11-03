cmake_minimum_required(VERSION 3.28)
project(mechsim_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MechsimWarnings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Catch2.cmake)

enable_testing()

add_subdirectory(engine)
add_subdirectory(kernel)
add_subdirectory(signals)
add_subdirectory(controller)
add_subdirectory(bindings)
add_subdirectory(viewer_debug)
