include(${CMAKE_CURRENT_SOURCE_DIR}/../../core/cmake/Catch2.cmake)

# Same sources as mechsim_fw_app, built with non-default sizes and timing to exercise the override macros.
mechsim_fw_add_app(mechsim_fw_app_variant)
target_compile_definitions(mechsim_fw_app_variant
    PRIVATE MECHSIM_APP_READY_TICKS=1u
    PUBLIC MECHSIM_APP_TICK_HZ=250u)

function(mechsim_fw_add_test name app)
    add_executable(${name} ${name}.cpp)
    set_target_properties(${name} PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
    target_compile_options(${name} PRIVATE ${MECHSIM_FW_WARNINGS})
    target_link_libraries(${name} PRIVATE ${app} Catch2::Catch2WithMain)
    catch_discover_tests(${name} TEST_PREFIX "${name}: ")
endfunction()

mechsim_fw_add_test(test_fw_reference mechsim_fw_app)
mechsim_fw_add_test(test_fw_variant mechsim_fw_app_variant)

# Gantry controller as shipped, and a variant with the queue policy and a 500 Hz tick to exercise the macros.
mechsim_fw_add_app(mechsim_fw_gantry_variant ${MECHSIM_FW_GANTRY_SOURCES})
target_compile_definitions(mechsim_fw_gantry_variant
    PRIVATE GANTRY_NEW_TARGET_POLICY=1 GANTRY_TX_QUEUE_BYTES=64u
    PUBLIC MECHSIM_APP_TICK_HZ=500u)
# A third with the lift at the 8 mm screw's own step scale (unequal per-axis scales).
mechsim_fw_add_app(mechsim_fw_gantry_scale ${MECHSIM_FW_GANTRY_SOURCES})
target_compile_definitions(mechsim_fw_gantry_scale
    PUBLIC GANTRY_STEPS_PER_M_LIFT=400000 GANTRY_COUNTS_PER_M_LIFT=300000)
mechsim_fw_add_test(test_gantry mechsim_fw_gantry)
mechsim_fw_add_test(test_gantry_variant mechsim_fw_gantry_variant)
mechsim_fw_add_test(test_gantry_scale mechsim_fw_gantry_scale)
target_include_directories(test_gantry PRIVATE ${MECHSIM_FW_APP_ROOT}/gantry)
target_include_directories(test_gantry_variant PRIVATE ${MECHSIM_FW_APP_ROOT}/gantry)
target_include_directories(test_gantry_scale PRIVATE ${MECHSIM_FW_APP_ROOT}/gantry)

set(fixtures ${CMAKE_CURRENT_SOURCE_DIR}/fixtures)
function(mechsim_fw_add_include_check name file)
    add_test(NAME "fw_include_check: ${name}"
        COMMAND ${CMAKE_COMMAND} -DAPP_ROOT=${fixtures} -DBOARD_HEADER=${MECHSIM_BOARD_HEADER}
        -DFILES=${fixtures}/${file} -P ${MECHSIM_FW_INCLUDE_CHECK})
    set_tests_properties("fw_include_check: ${name}" PROPERTIES LABELS "FW-005;FW-006" ${ARGN})
endfunction()

mechsim_fw_add_include_check("accepts board, freestanding and local headers" good_include.c
    PASS_REGULAR_EXPRESSION "firmware_include_check: 1 files clean")
mechsim_fw_add_include_check("rejects stdio.h" bad_include.c
    PASS_REGULAR_EXPRESSION "bad_include.c:1: rejected include <stdio\\.h>")
mechsim_fw_add_include_check("rejects a path escaping the app tree" bad_escape.c
    PASS_REGULAR_EXPRESSION "bad_escape.c:1: rejected include \"\\.\\./\\.\\./app/app_debug\\.h\"")
mechsim_fw_add_include_check("rejects a simulator header" bad_simulator.c
    PASS_REGULAR_EXPRESSION "bad_simulator.c:2: rejected include \"core/signals/signal\\.h\"")
mechsim_fw_add_include_check("rejects a macro include" bad_macro.c
    PASS_REGULAR_EXPRESSION "bad_macro.c:2: rejected include form")
mechsim_fw_add_include_check("rejects a missing local header" bad_missing.c
    PASS_REGULAR_EXPRESSION "bad_missing.c:1: rejected include \"absent\\.h\"")
add_test(NAME "fw_include_check: rejects loose sources in firmware/board/"
    COMMAND ${CMAKE_COMMAND} -DAPP_ROOT=${fixtures} -DFILES=${fixtures}/good_include.c
    -DBOARD_HEADER=${MECHSIM_BOARD_HEADER} -DBOARD_ROOT=${fixtures}/board_loose -P ${MECHSIM_FW_INCLUDE_CHECK})
set_tests_properties("fw_include_check: rejects loose sources in firmware/board/" PROPERTIES
    LABELS "FW-012" PASS_REGULAR_EXPRESSION "loose_vendor.c: source outside firmware/board/<chip>/")
add_test(NAME "fw_include_check: accepts the shipped firmware/app/"
    COMMAND ${CMAKE_COMMAND} -DAPP_ROOT=${MECHSIM_FW_APP_ROOT} -DBOARD_ROOT=${MECHSIM_FW_BOARD_ROOT}
    -DBOARD_HEADER=${MECHSIM_BOARD_HEADER} -P ${MECHSIM_FW_INCLUDE_CHECK})
set_tests_properties("fw_include_check: accepts the shipped firmware/app/" PROPERTIES LABELS "FW-005;FW-006;FW-012")
mechsim_fw_add_include_check("rejects an absolute path" bad_absolute.c
    PASS_REGULAR_EXPRESSION "bad_absolute.c:1: rejected include \"/usr/include/stdio\\.h\"")
mechsim_fw_add_include_check("rejects a mismatched angle delimiter" bad_mismatch_angle.c
    PASS_REGULAR_EXPRESSION "bad_mismatch_angle.c:1: rejected include form")
mechsim_fw_add_include_check("rejects a mismatched quote delimiter" bad_mismatch_quote.c
    PASS_REGULAR_EXPRESSION "bad_mismatch_quote.c:1: rejected include form")
mechsim_fw_add_include_check("rejects trailing text after an include" bad_trailing.c
    PASS_REGULAR_EXPRESSION "bad_trailing.c:1: rejected include form")
mechsim_fw_add_include_check("rejects a board header shadowed in the app tree" shadow/uses_board.c
    PASS_REGULAR_EXPRESSION "uses_board.c:1: mechsim_board.h resolves to .*shadow/mechsim_board\\.h, not .*board_interface/mechsim_board\\.h")
mechsim_fw_add_include_check("rejects a file named mechsim_board.h in the app tree" shadow/mechsim_board.h
    PASS_REGULAR_EXPRESSION "shadow/mechsim_board\\.h: shadows board_interface/mechsim_board\\.h")
add_test(NAME "fw_include_check: requires the board_interface header"
    COMMAND ${CMAKE_COMMAND} -DAPP_ROOT=${fixtures} -DBOARD_HEADER=${fixtures}/absent/mechsim_board.h
    -DFILES=${fixtures}/good_include.c -P ${MECHSIM_FW_INCLUDE_CHECK})
set_tests_properties("fw_include_check: requires the board_interface header" PROPERTIES
    LABELS "FW-005;FW-006" PASS_REGULAR_EXPRESSION "BOARD_HEADER does not name the board_interface")
