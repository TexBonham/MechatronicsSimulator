# Included by core/engine/ in both the full and the standalone configure, so the guard keeps one fetch.
if(NOT TARGET Eigen3::Eigen)
    include(FetchContent)
    FetchContent_Declare(
        Eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE
        SYSTEM
    )
    # Eigen's own CMake predates CMP0077, so without the policy default its option() calls would ignore these.
    block()
        set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
        set(EIGEN_BUILD_DOC OFF)
        set(BUILD_TESTING OFF)
        set(EIGEN_BUILD_PKGCONFIG OFF)
        FetchContent_MakeAvailable(Eigen)
    endblock()
endif()
