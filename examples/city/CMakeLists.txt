# Keep the sample dependency isolated from normal Frontier builds. bgfx.cmake
# pins bgfx, bx, and bimg as submodules and supplies bgfx's portable entry and
# debug-draw helpers, so the sample needs no platform-specific window code.
set(BGFX_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BGFX_BUILD_EXAMPLE_COMMON ON CACHE BOOL "" FORCE)
set(BGFX_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BGFX_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BGFX_INSTALL OFF CACHE BOOL "" FORCE)
set(BGFX_CUSTOM_TARGETS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    bgfx_cmake
    GIT_REPOSITORY https://github.com/bkaradzic/bgfx.cmake.git
    GIT_TAG        12428216cfa33eb34dc91187ad3a3c92196c9849
    GIT_SHALLOW    FALSE
    GIT_SUBMODULES_RECURSE TRUE
)
FetchContent_MakeAvailable(bgfx_cmake)

if(ANDROID)
    add_library(frontier_city SHARED city.cpp)
else()
    add_executable(frontier_city WIN32 city.cpp)
endif()

target_link_libraries(frontier_city PRIVATE frontier example-common)
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
