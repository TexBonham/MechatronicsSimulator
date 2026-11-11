# One list for both presets (FW-008); never set per preset. Open ruling 3 reads FW-008 as the GCC host build versus
# arm-none-eabi: under MSVC (Windows host preset) the closest equivalent is /W4 /WX, and C11 comes from the targets'
# C_STANDARD 11, which CMake passes to cl as /std:c11.
if(MSVC)
    # /wd4127: configuration macros (gantry_config.h) are compile-time constants by design; GCC does not warn on them.
    set(MECHSIM_FW_WARNINGS /W4 /WX /wd4127)
else()
    set(MECHSIM_FW_WARNINGS -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror)
endif()
