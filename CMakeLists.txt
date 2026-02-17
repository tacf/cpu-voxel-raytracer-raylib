cmake_minimum_required(VERSION 3.16...4.2)
project(voxel_dda_raylib C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Needed for strict C modes on Linux so raylib can see timespec/clock_gettime/
# nanosleep/readlink declarations while compiling rcore.c.
if(UNIX AND NOT APPLE)
    add_compile_definitions(_POSIX_C_SOURCE=200809L _DEFAULT_SOURCE)
endif()

# CMake 4 rejects very old policy compatibility levels in dependencies.
# raylib tags may still call `cmake_minimum_required()` with old versions,
# so provide the minimum policy floor suggested by CMake diagnostics.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
    if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
        set(CMAKE_POLICY_VERSION_MINIMUM "3.5" CACHE STRING
            "Policy compatibility floor used when configuring dependencies")
    endif()
endif()

add_executable(voxel_dda_raylib main.c)
include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

# Fetch raylib
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY "https://github.com/raysan5/raylib.git"
    GIT_TAG "master"
    GIT_PROGRESS TRUE
    GIT_SHALLOW 1
)
FetchContent_MakeAvailable(raylib)


target_link_libraries(voxel_dda_raylib PRIVATE raylib)

if(UNIX AND NOT APPLE)
    target_link_libraries(voxel_dda_raylib PRIVATE m)
endif()
