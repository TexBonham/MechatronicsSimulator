# Firmware tree, configured on its own: cmake --preset host | arm-none-eabi (C2, FW-007).
cmake_minimum_required(VERSION 3.28)
project(mechsim_firmware LANGUAGES C)

option(MECHSIM_FW_BUILD_TESTS "Build the host test executable (Catch2, C++)" OFF)

include(cmake/FirmwareWarnings.cmake)

set(MECHSIM_FW_APP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/app)
set(MECHSIM_FW_BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/board)
set(MECHSIM_FW_INCLUDE_CHECK ${CMAKE_CURRENT_SOURCE_DIR}/cmake/include_check.cmake)
set(MECHSIM_BOARD_INTERFACE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../board_interface)

set(MECHSIM_BOARD_HEADER ${MECHSIM_BOARD_INTERFACE_DIR}/mechsim_board.h)
set(MECHSIM_APP_TICK_HZ "" CACHE STRING "Control tick rate in Hz from the design (C4); empty keeps the header default")

set(include_check_command
    ${CMAKE_COMMAND} -DAPP_ROOT=${MECHSIM_FW_APP_ROOT} -DBOARD_ROOT=${MECHSIM_FW_BOARD_ROOT}
    -DBOARD_HEADER=${MECHSIM_BOARD_HEADER} -P ${MECHSIM_FW_INCLUDE_CHECK})

execute_process(COMMAND ${include_check_command} RESULT_VARIABLE include_check_result)
if(NOT include_check_result EQUAL 0)
    message(FATAL_ERROR "firmware_include_check rejected firmware/app/ at configure time")
endif()

file(GLOB_RECURSE MECHSIM_FW_APP_FILES CONFIGURE_DEPENDS
    ${MECHSIM_FW_APP_ROOT}/*.c ${MECHSIM_FW_APP_ROOT}/*.h)
add_custom_target(firmware_include_check ALL
    COMMAND ${include_check_command}
    DEPENDS ${MECHSIM_FW_APP_FILES} ${MECHSIM_FW_INCLUDE_CHECK}
    COMMENT "Checking firmware/app/ includes (invariant 8)"
    VERBATIM)

# The reference firmware; the gantry controller replaces it in milestone 1 (firmware/CLAUDE.md).
function(mechsim_fw_add_app name)
    add_library(${name} STATIC ${MECHSIM_FW_APP_ROOT}/main_tick.c)
    set_target_properties(${name} PROPERTIES
        C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
    target_include_directories(${name} PUBLIC ${MECHSIM_BOARD_INTERFACE_DIR})
    target_compile_options(${name} PRIVATE ${MECHSIM_FW_WARNINGS})
    if(NOT MECHSIM_APP_TICK_HZ STREQUAL "")
        target_compile_definitions(${name} PUBLIC MECHSIM_APP_TICK_HZ=${MECHSIM_APP_TICK_HZ})
    endif()
    add_dependencies(${name} firmware_include_check)
endfunction()

mechsim_fw_add_app(mechsim_fw_app)

if(MECHSIM_FW_BUILD_TESTS)
    enable_language(CXX)
    enable_testing()
    add_subdirectory(tests)
endif()
