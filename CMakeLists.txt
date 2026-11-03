# CMake presets cannot point at a source directory other than their own (no sourceDir field up to preset version 6
# and CMake 3.28), so this file only forwards to the core tree. core/ stays configurable on its own.
cmake_minimum_required(VERSION 3.28)
project(mechsim LANGUAGES CXX)
enable_testing()
add_subdirectory(core)
