# Included by core/ and by core/engine/ when configured standalone, so the guard keeps one definition.
if(NOT TARGET mechsim_warnings)
    add_library(mechsim_warnings INTERFACE)
    target_compile_options(mechsim_warnings INTERFACE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror)
endif()
