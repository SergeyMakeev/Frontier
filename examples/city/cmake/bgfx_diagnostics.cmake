# Adapt only the pinned source, preserving bgfx's normal device selection and
# C/C++ ABI agreement. A normal patch keeps the dependency changes reviewable.
find_package(Git REQUIRED)
set(frontier_bgfx_directory "${bgfx_cmake_SOURCE_DIR}/bgfx")
foreach(frontier_bgfx_patch_name IN ITEMS bgfx-diagnostics.patch bgfx-sdl-wayland.patch)
    set(frontier_bgfx_patch "${CMAKE_CURRENT_LIST_DIR}/${frontier_bgfx_patch_name}")
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" apply --reverse --check "${frontier_bgfx_patch}"
        WORKING_DIRECTORY "${frontier_bgfx_directory}"
        RESULT_VARIABLE frontier_bgfx_already_patched
        OUTPUT_QUIET ERROR_QUIET
    )
    if(NOT frontier_bgfx_already_patched EQUAL 0)
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" apply --check "${frontier_bgfx_patch}"
            WORKING_DIRECTORY "${frontier_bgfx_directory}"
            RESULT_VARIABLE frontier_bgfx_patch_check
            OUTPUT_QUIET ERROR_VARIABLE frontier_bgfx_patch_error
        )
        if(NOT frontier_bgfx_patch_check EQUAL 0)
            message(FATAL_ERROR
                "The city patch ${frontier_bgfx_patch_name} does not match the pinned bgfx source. "
                "Review cmake/${frontier_bgfx_patch_name} when updating the dependency.\n"
                "${frontier_bgfx_patch_error}")
        endif()
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" apply "${frontier_bgfx_patch}"
            WORKING_DIRECTORY "${frontier_bgfx_directory}"
            COMMAND_ERROR_IS_FATAL ANY
        )
    endif()
endforeach()
