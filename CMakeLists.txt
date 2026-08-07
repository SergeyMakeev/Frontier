cmake_minimum_required(VERSION 3.24)
project(hlod_tree CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(HLOD_BUILD_TESTS "Build unit tests" ON)
option(HLOD_BUILD_BENCH "Build performance tests" ON)
option(HLOD_AVX2 "Enable AVX2 code generation (x86-64 only)" ON)
option(HLOD_FORCE_SCALAR "Disable SIMD intrinsics; use the portable scalar paths" OFF)
option(HLOD_PROFILE_SYMBOLS "Keep optimized source line tables for CPU profiling" OFF)

if(HLOD_BUILD_TESTS OR HLOD_BUILD_BENCH)
    find_package(Threads REQUIRED)
endif()

# AVX2 flags only make sense on x86-64; other architectures pick their SIMD
# path (NEON on arm64, SSE2 baseline on x86) automatically in math.h.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64|x64)$")
    set(HLOD_ARCH_X64 TRUE)
else()
    set(HLOD_ARCH_X64 FALSE)
endif()

# ---- core library -----------------------------------------------------------
add_library(hlod STATIC
    src/builder.cpp
    src/config.cpp
    src/page.cpp
    src/world.cpp
)
target_include_directories(hlod PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(MSVC)
    target_compile_options(hlod PRIVATE /W4 /permissive-)
    if(HLOD_AVX2 AND HLOD_ARCH_X64 AND NOT HLOD_FORCE_SCALAR)
        target_compile_options(hlod PUBLIC /arch:AVX2)
    endif()
else()
    target_compile_options(hlod PRIVATE -Wall -Wextra)
    if(HLOD_AVX2 AND HLOD_ARCH_X64 AND NOT HLOD_FORCE_SCALAR)
        target_compile_options(hlod PUBLIC -mavx2 -mfma)
    endif()
endif()

if(HLOD_FORCE_SCALAR)
    target_compile_definitions(hlod PUBLIC HLOD_FORCE_SCALAR=1)
endif()

if(HLOD_PROFILE_SYMBOLS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(hlod PRIVATE -gline-tables-only)
endif()

# ---- external deps via FetchContent ------------------------------------------
include(FetchContent)

if(HLOD_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        # v1.17.0 + fa8438ae: fixes a -Wcharacter-conversion error in
        # gtest-printers.h that breaks the build with clang >= 21 / clang-cl.
        GIT_TAG        fa8438ae6b70c57010177de47a9f13d7041a6328
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
