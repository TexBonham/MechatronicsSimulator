# Included by core/ and by core/engine/ when configured standalone, so the guard keeps one definition.
if(NOT TARGET mechsim_warnings)
    add_library(mechsim_warnings INTERFACE)
    if(MSVC)
        # /permissive- for standard conformance; /utf-8 so source and execution character sets match on every locale.
        target_compile_options(mechsim_warnings INTERFACE /W4 /WX /permissive- /utf-8)
        # /bigobj: Eigen-heavy translation units (multibody_provider.cpp) exceed the default COFF section count.
        target_compile_options(mechsim_warnings INTERFACE /bigobj)
        # The portable C library calls (std::strerror, std::getenv) stay; MSVC's _s replacements are not standard C++.
        target_compile_definitions(mechsim_warnings INTERFACE _CRT_SECURE_NO_WARNINGS)
    else()
        target_compile_options(mechsim_warnings INTERFACE
            -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror)
    endif()
endif()
