# Included by core/ and by core/engine/ when configured standalone, so the guard keeps one definition.
if(NOT TARGET mechsim_warnings)
    add_library(mechsim_warnings INTERFACE)
    if(MSVC)
        # /permissive- for standard conformance; /utf-8 so source and execution character sets match on every locale.
        target_compile_options(mechsim_warnings INTERFACE /W4 /WX /permissive- /utf-8)
    else()
        target_compile_options(mechsim_warnings INTERFACE
            -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror)
    endif()
endif()
