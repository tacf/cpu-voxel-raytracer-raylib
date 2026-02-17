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

set(_raylib_target "")

# Prefer an already-installed raylib package when available.
find_package(raylib QUIET CONFIG)
if(TARGET raylib)
    set(_raylib_target raylib)
elseif(TARGET raylib::raylib)
    set(_raylib_target raylib::raylib)
endif()

option(RAYLIB_FETCHCONTENT "Fetch raylib via CMake FetchContent when not found" ON)
set(RAYLIB_FETCH_TAG "5.0" CACHE STRING "raylib tag to fetch via FetchContent")

# Fallback: fetch raylib from upstream.
if(NOT _raylib_target AND RAYLIB_FETCHCONTENT)
    include(FetchContent)

    # Keep fetched raylib build minimal.
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
        raylib
        URL https://github.com/raysan5/raylib/archive/refs/tags/${RAYLIB_FETCH_TAG}.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP ON
    )
    FetchContent_MakeAvailable(raylib)

    if(TARGET raylib)
        set(_raylib_target raylib)
    elseif(TARGET raylib::raylib)
        set(_raylib_target raylib::raylib)
    endif()
endif()

if(NOT _raylib_target)
    message(FATAL_ERROR
        "raylib was not found. Install raylib dev files, set raylib_DIR, "
        "or configure with -DRAYLIB_FETCHCONTENT=ON.")
endif()

target_link_libraries(voxel_dda_raylib PRIVATE ${_raylib_target})

if(UNIX AND NOT APPLE)
    target_link_libraries(voxel_dda_raylib PRIVATE m)
endif()
