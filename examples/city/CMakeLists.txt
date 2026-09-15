# Keep the sample dependency isolated from normal Frontier builds. bgfx.cmake
# pins bgfx, bx, and bimg as submodules and supplies bgfx's portable entry and
# debug-draw helpers, so the sample needs no platform-specific window code.
set(BGFX_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BGFX_BUILD_EXAMPLE_COMMON ON CACHE BOOL "" FORCE)
set(BGFX_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BGFX_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BGFX_INSTALL OFF CACHE BOOL "" FORCE)
set(BGFX_CUSTOM_TARGETS OFF CACHE BOOL "" FORCE)

# The pinned debug-draw shaders use integer a_indices to select model matrices
# for cones/cylinders. bgfx's default GL 2.1 build binds those as floats, which
# corrupts nonzero indices and stretches triangles across unrelated geometry.
# GL 3.1 uses integer vertex attributes and the GLSL 1.40 shader path. Upgrade
# the empty cache value left by existing builds, while preserving higher minima.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" OR WIN32)
    if(NOT DEFINED BGFX_OPENGL_VERSION OR BGFX_OPENGL_VERSION STREQUAL "")
        set(BGFX_OPENGL_VERSION 31 CACHE STRING "Specify minimum opengl version" FORCE)
    elseif(BGFX_OPENGL_VERSION LESS 31)
        message(FATAL_ERROR
            "The city sample's debug-draw shaders require OpenGL 3.1 or newer. "
            "Reconfigure with -DBGFX_OPENGL_VERSION=31 (or a higher version).")
    endif()
endif()

# Diagnose system dependencies before downloading/configuring bgfx. Desktop
# images can have the runtime libraries without their development packages.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    option(FRONTIER_CITY_USE_SDL
           "Use SDL2 for native Wayland and X11 city windows" ON)
    set(BGFX_WITH_SDL ${FRONTIER_CITY_USE_SDL})
    find_package(X11 QUIET)
    find_package(OpenGL QUIET)
    set(FRONTIER_CITY_MISSING_DEPENDENCIES "")
    if(FRONTIER_CITY_USE_SDL)
        find_package(SDL2 2.0.22 QUIET CONFIG)
        if(NOT SDL2_FOUND)
            list(APPEND FRONTIER_CITY_MISSING_DEPENDENCIES "SDL2 >= 2.0.22 (libsdl2-dev)")
        endif()
    endif()
    if(NOT X11_FOUND)
        list(APPEND FRONTIER_CITY_MISSING_DEPENDENCIES "X11 (libx11-dev)")
    endif()
    if(NOT OPENGL_FOUND)
        list(APPEND FRONTIER_CITY_MISSING_DEPENDENCIES "OpenGL (libgl1-mesa-dev)")
    endif()

    # The pinned bgfx.cmake defaults BGFX_WITH_WAYLAND to ON on Linux and
    # links wayland-egl independently of the selected window entry layer.
    if(NOT DEFINED BGFX_WITH_WAYLAND OR BGFX_WITH_WAYLAND)
        find_library(FRONTIER_CITY_WAYLAND_EGL_LIBRARY NAMES wayland-egl)
        mark_as_advanced(FRONTIER_CITY_WAYLAND_EGL_LIBRARY)
        if(NOT FRONTIER_CITY_WAYLAND_EGL_LIBRARY)
            list(APPEND FRONTIER_CITY_MISSING_DEPENDENCIES
                "Wayland EGL (libwayland-dev)")
        endif()
    endif()

    if(FRONTIER_CITY_MISSING_DEPENDENCIES)
        list(JOIN FRONTIER_CITY_MISSING_DEPENDENCIES ", " FRONTIER_CITY_MISSING_TEXT)
        message(FATAL_ERROR
            "The Frontier city sample is missing Linux development dependencies:\n"
            "  ${FRONTIER_CITY_MISSING_TEXT}\n"
            "On Armbian, Debian, or Ubuntu, install them with:\n"
            "  sudo apt-get update\n"
            "  sudo apt-get install libx11-dev libgl1-mesa-dev libwayland-dev libsdl2-dev libdecor-0-0 libdecor-0-plugin-1-gtk\n"
            "Then rerun ./run_city_sample.sh (or your CMake configure command).\n"
            "See examples/city/README.md for Linux setup instructions.")
    endif()
endif()

FetchContent_Declare(
    bgfx_cmake
    GIT_REPOSITORY https://github.com/bkaradzic/bgfx.cmake.git
    GIT_TAG        12428216cfa33eb34dc91187ad3a3c92196c9849
    GIT_SHALLOW    FALSE
    GIT_SUBMODULES_RECURSE TRUE
)
FetchContent_MakeAvailable(bgfx_cmake)
include(cmake/bgfx_diagnostics.cmake)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND FRONTIER_CITY_USE_SDL)
    # Propagate the imported target's include paths as well as its library.
    target_link_libraries(example-common PUBLIC SDL2::SDL2)
    message(STATUS "Frontier city windows: SDL2 (native Wayland / X11)")
endif()

if(ANDROID)
    add_library(frontier_city SHARED city.cpp hardware_info.cpp)
else()
    add_executable(frontier_city WIN32 city.cpp hardware_info.cpp)
endif()

target_link_libraries(frontier_city PRIVATE frontier example-common)
if(WIN32)
    target_link_libraries(frontier_city PRIVATE advapi32)
endif()
target_compile_features(frontier_city PRIVATE cxx_std_20)
target_compile_definitions(frontier_city PRIVATE ENTRY_CONFIG_IMPLEMENT_MAIN=1)
set_target_properties(frontier_city PROPERTIES
    CXX_EXTENSIONS OFF
    FOLDER "Frontier/examples"
)

if(MSVC)
    target_compile_options(frontier_city PRIVATE /W4 /permissive-)
    # bgfx's Windows entry layer supplies main(); this matches upstream's
    # example target setup for a WIN32 executable.
    set_target_properties(frontier_city PROPERTIES
        LINK_FLAGS "/ENTRY:\"mainCRTStartup\""
    )
else()
    target_compile_options(frontier_city PRIVATE -Wall -Wextra)
endif()

if(IOS)
    set_target_properties(frontier_city PROPERTIES
        MACOSX_BUNDLE ON
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.frontier.dynamic-city"
    )
endif()

if(FRONTIER_BUILD_TESTS AND NOT ANDROID AND NOT IOS)
    add_test(
        NAME frontier_city_streaming_make_before_break
        COMMAND frontier_city --streaming-orbit-self-test
    )
    set_tests_properties(frontier_city_streaming_make_before_break PROPERTIES
        LABELS "city;streaming"
        TIMEOUT 120
    )
endif()
