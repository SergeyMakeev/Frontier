cmake_minimum_required(VERSION 3.24)
project(hlod_tree CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(HLOD_BUILD_TESTS "Build unit tests" ON)
option(HLOD_BUILD_BENCH "Build performance tests" ON)
option(HLOD_AVX2 "Enable AVX2 code generation" ON)

# ---- core library -----------------------------------------------------------
add_library(hlod STATIC
    src/builder.cpp
    src/world.cpp
)
target_include_directories(hlod PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(MSVC)
    target_compile_options(hlod PRIVATE /W4 /permissive-)
    if(HLOD_AVX2)
        target_compile_options(hlod PUBLIC /arch:AVX2)
    endif()
else()
    target_compile_options(hlod PRIVATE -Wall -Wextra)
    if(HLOD_AVX2)
        target_compile_options(hlod PUBLIC -mavx2 -mfma)
    endif()
endif()

# ---- external deps via FetchContent ------------------------------------------
include(FetchContent)

if(HLOD_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.17.0
        GIT_SHALLOW    TRUE
    )
    # Required on Windows: match the parent project's CRT.
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    add_subdirectory(tests)
endif()

if(HLOD_BUILD_BENCH)
    FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG        v1.9.4
        GIT_SHALLOW    TRUE
    )
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_INSTALL_DOCS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(benchmark)

    add_subdirectory(bench)
endif()
