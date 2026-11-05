# MuJoCo's C library at a pinned release (ADR 0014; owner ruling: latest 3.9 patch, the prebuilt release archive,
# never a system package). Only core/physics/mujoco/ links it. The archive carries no CMake package for its library,
# so the imported target is declared here.
#
# Release assets at https://github.com/google-deepmind/mujoco/releases/tag/3.9.0:
#   Linux x86_64    mujoco-3.9.0-linux-x86_64.tar.gz   (pinned below)
#   Linux aarch64   mujoco-3.9.0-linux-aarch64.tar.gz  (hash not pinned)
#   macOS universal mujoco-3.9.0-macos-universal2.dmg  (a disk image; CMake cannot extract it, hash not pinned)
#   Windows x86_64  mujoco-3.9.0-windows-x86_64.zip    (hash not pinned)
if(NOT TARGET mujoco::mujoco)
    set(MECHSIM_MUJOCO_VERSION 3.9.0)
    if(NOT (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$"))
        message(FATAL_ERROR
            "MuJoCo ${MECHSIM_MUJOCO_VERSION} is pinned only for Linux x86_64; "
            "got ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}. See core/cmake/MuJoCo.cmake for the archive names.")
    endif()
    include(FetchContent)
    FetchContent_Declare(
        mujoco
        URL https://github.com/google-deepmind/mujoco/releases/download/${MECHSIM_MUJOCO_VERSION}/mujoco-${MECHSIM_MUJOCO_VERSION}-linux-x86_64.tar.gz
        URL_HASH SHA256=d11f281540d0d1844e2923bf43b6fff5ad186ec55927a8dae0eb26b9e579eed2
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(mujoco)
    add_library(mujoco::mujoco SHARED IMPORTED GLOBAL)
    set_target_properties(mujoco::mujoco PROPERTIES
        IMPORTED_LOCATION ${mujoco_SOURCE_DIR}/lib/libmujoco.so.${MECHSIM_MUJOCO_VERSION}
        IMPORTED_SONAME libmujoco.so.${MECHSIM_MUJOCO_VERSION}
        INTERFACE_INCLUDE_DIRECTORIES ${mujoco_SOURCE_DIR}/include
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES ${mujoco_SOURCE_DIR}/include
    )
endif()
