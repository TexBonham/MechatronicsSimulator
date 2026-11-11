# One list for both presets (FW-008); never set per preset. Open ruling 3 reads FW-008 as the GCC host build versus
# arm-none-eabi: under MSVC (Windows host preset) the closest equivalent is /W4 /WX, and C11 comes from the targets'
# C_STANDARD 11, which CMake passes to cl as /std:c11.
if(MSVC)
    set(MECHSIM_FW_WARNINGS /W4 /WX)
else()
    set(MECHSIM_FW_WARNINGS -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror)
endif()
