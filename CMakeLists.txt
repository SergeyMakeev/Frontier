cmake_minimum_required(VERSION 3.24)
project(Frontier VERSION 0.7.0 LANGUAGES CXX)

# A standalone checkout is developer-friendly by default. When Frontier is
# embedded with add_subdirectory(), do not fetch or build its test-only
# dependencies unless the parent explicitly opts in.
option(FRONTIER_BUILD_TESTS "Build unit tests" ${PROJECT_IS_TOP_LEVEL})
option(FRONTIER_BUILD_CITY_SAMPLE
       "Build the cross-platform bgfx dynamic-city sample" OFF)
option(FRONTIER_TEST_PAYLOAD32
       "Build additional BVH4/BVH8 unit tests with four-byte payload words" OFF)
option(FRONTIER_BUILD_BENCH "Build performance tests" OFF)
option(FRONTIER_AVX2 "Enable AVX2 code generation for BVH8 (x86-64 only)" ON)
option(FRONTIER_FORCE_SCALAR "Disable SIMD intrinsics; use the portable scalar paths" OFF)
set(FRONTIER_BVH_WIDTH "AUTO" CACHE STRING
    "BVH branching factor and wide traversal width (AUTO, 4, or 8)")
set_property(CACHE FRONTIER_BVH_WIDTH PROPERTY STRINGS AUTO 4 8)
option(FRONTIER_PROFILE_SYMBOLS "Keep optimized source line tables for CPU profiling" OFF)
option(FRONTIER_IPO "Enable interprocedural optimization for Frontier targets" OFF)
set(FRONTIER_PGO_MODE "OFF" CACHE STRING
    "GCC profile-guided optimization phase (OFF, GENERATE, or USE)")
set_property(CACHE FRONTIER_PGO_MODE PROPERTY STRINGS OFF GENERATE USE)
set(FRONTIER_PGO_DIR "" CACHE PATH
    "Directory containing GCC profile data for FRONTIER_PGO_MODE")
option(FRONTIER_STATS "Enable per-query traversal statistics" OFF)
option(FRONTIER_DEBUG_TOOLS
       "Enable opt-in read-only runtime debug introspection" OFF)
option(FRONTIER_CONTRACT_CHECKS "Enable public caller-contract checks" ON)
option(FRONTIER_VALIDATE_SUBTREES
       "Validate complete serialized subtree structure during registration" ON)

# Standalone correctness builds are Debug unless the caller explicitly chose a
# configuration. Benchmarks use their own Release build directory and never
# share this default.
if(PROJECT_IS_TOP_LEVEL AND FRONTIER_BUILD_TESTS AND
   NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

if(FRONTIER_BUILD_TESTS OR FRONTIER_BUILD_BENCH)
    find_package(Threads REQUIRED)
endif()

# AVX2 flags only make sense on x86-64; other architectures pick their SIMD
# path (NEON on arm64, SSE2 baseline on x86) automatically in math.h.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64|x64)$")
    set(FRONTIER_ARCH_X64 TRUE)
else()
    set(FRONTIER_ARCH_X64 FALSE)
endif()

if(FRONTIER_BVH_WIDTH STREQUAL "AUTO")
    # AVX2 is the only eight-lane backend. SSE2 and NEON are four lanes, so
    # their natural default is BVH4. Scalar builds also prefer the compact
    # BVH4 layout; callers can explicitly request BVH8 for comparisons.
    if(FRONTIER_AVX2 AND FRONTIER_ARCH_X64 AND NOT FRONTIER_FORCE_SCALAR)
        set(FRONTIER_EFFECTIVE_BVH_WIDTH 8)
    else()
        set(FRONTIER_EFFECTIVE_BVH_WIDTH 4)
    endif()
elseif(FRONTIER_BVH_WIDTH STREQUAL "4" OR
       FRONTIER_BVH_WIDTH STREQUAL "8")
    set(FRONTIER_EFFECTIVE_BVH_WIDTH ${FRONTIER_BVH_WIDTH})
else()
    message(FATAL_ERROR "FRONTIER_BVH_WIDTH must be AUTO, 4, or 8")
endif()
message(STATUS
    "Frontier BVH width: ${FRONTIER_EFFECTIVE_BVH_WIDTH} "
    "(requested ${FRONTIER_BVH_WIDTH})")

if(FRONTIER_IPO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT FRONTIER_IPO_SUPPORTED OUTPUT FRONTIER_IPO_ERROR)
    if(NOT FRONTIER_IPO_SUPPORTED)
        message(FATAL_ERROR
            "FRONTIER_IPO requested but unsupported: ${FRONTIER_IPO_ERROR}")
    endif()
endif()

# PGO is intentionally a build-wide setting. With IPO enabled, GCC consumes
# profiles during both compilation and the final LTO link; the benchmark
# executable must also link the profiling runtime during the generate phase.
# Applying the flags globally keeps all three stages consistent and profiles
# call edges that cross the library/executable boundary.
string(TOUPPER "${FRONTIER_PGO_MODE}" FRONTIER_PGO_MODE)
if(NOT FRONTIER_PGO_MODE STREQUAL "OFF")
    if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        message(FATAL_ERROR
            "FRONTIER_PGO_MODE currently supports GCC only")
    endif()
    if(NOT FRONTIER_PGO_MODE STREQUAL "GENERATE" AND
       NOT FRONTIER_PGO_MODE STREQUAL "USE")
        message(FATAL_ERROR
            "FRONTIER_PGO_MODE must be OFF, GENERATE, or USE")
    endif()
    if(FRONTIER_PGO_DIR STREQUAL "")
        message(FATAL_ERROR
            "FRONTIER_PGO_DIR is required when FRONTIER_PGO_MODE is enabled")
    endif()
    get_filename_component(FRONTIER_PGO_DIR_ABSOLUTE
        "${FRONTIER_PGO_DIR}" ABSOLUTE BASE_DIR "${CMAKE_BINARY_DIR}")
    if(FRONTIER_PGO_MODE STREQUAL "GENERATE")
        set(FRONTIER_PGO_FLAGS
            "-fprofile-generate=${FRONTIER_PGO_DIR_ABSOLUTE}")
    else()
        set(FRONTIER_PGO_FLAGS
            "-fprofile-use=${FRONTIER_PGO_DIR_ABSOLUTE}"
            -fprofile-correction
            -Wno-missing-profile)
    endif()
    add_compile_options(${FRONTIER_PGO_FLAGS})
    add_link_options(${FRONTIER_PGO_FLAGS})
    message(STATUS
        "Frontier GCC PGO: ${FRONTIER_PGO_MODE} (${FRONTIER_PGO_DIR_ABSOLUTE})")
endif()

# ---- core library -----------------------------------------------------------
set(FRONTIER_SOURCES
    src/builder.cpp
    src/config.cpp
    src/subtree.cpp
    src/spatial_database.cpp
    src/rigid_motion.cpp
    src/terminal_render.cpp
)

function(frontier_configure_library target width)
    target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_features(${target} PUBLIC cxx_std_20)
    target_compile_definitions(${target} PUBLIC FRONTIER_BVH_WIDTH=${width})
    set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
    if(FRONTIER_IPO)
        set_property(TARGET ${target} PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
    endif()

    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
        if(FRONTIER_AVX2 AND FRONTIER_ARCH_X64 AND NOT FRONTIER_FORCE_SCALAR AND
           width EQUAL 8)
            target_compile_options(${target} PUBLIC /arch:AVX2)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra)
        if(FRONTIER_AVX2 AND FRONTIER_ARCH_X64 AND NOT FRONTIER_FORCE_SCALAR AND
           width EQUAL 8)
            target_compile_options(${target} PUBLIC -mavx2 -mfma)
        endif()
    endif()

    if(FRONTIER_FORCE_SCALAR)
        target_compile_definitions(${target} PUBLIC FRONTIER_FORCE_SCALAR=1)
    endif()

    if(FRONTIER_STATS)
        target_compile_definitions(${target} PUBLIC FRONTIER_STATS=1)
    endif()

    if(FRONTIER_DEBUG_TOOLS)
        target_compile_definitions(${target} PUBLIC FRONTIER_DEBUG_TOOLS=1)
    endif()

    if(FRONTIER_CONTRACT_CHECKS)
        target_compile_definitions(${target} PUBLIC FRONTIER_CONTRACT_CHECKS=1)
    else()
        target_compile_definitions(${target} PUBLIC FRONTIER_CONTRACT_CHECKS=0)
    endif()

    if(FRONTIER_VALIDATE_SUBTREES)
        target_compile_definitions(${target} PUBLIC FRONTIER_VALIDATE_SUBTREES=1)
    else()
        target_compile_definitions(${target} PUBLIC FRONTIER_VALIDATE_SUBTREES=0)
    endif()

    if(FRONTIER_PROFILE_SYMBOLS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(${target} PRIVATE -gline-tables-only)
    endif()
endfunction()

add_library(frontier STATIC ${FRONTIER_SOURCES})
frontier_configure_library(frontier ${FRONTIER_EFFECTIVE_BVH_WIDTH})

# End-to-end performance tests compile the implementation once per supported
# payload word width. These private variants keep the public library's payload
# customization macro-based while guaranteeing an otherwise matched 32/64-bit
# benchmark comparison in one build directory.
if(FRONTIER_BUILD_BENCH)
    add_library(frontier_bench_payload64_lib STATIC ${FRONTIER_SOURCES})
    frontier_configure_library(frontier_bench_payload64_lib
                               ${FRONTIER_EFFECTIVE_BVH_WIDTH})
    target_compile_definitions(frontier_bench_payload64_lib PUBLIC
        FRONTIER_USER_PAYLOAD=::std::uint64_t
        FRONTIER_INVALID_PAYLOAD=UINT64_MAX)

    add_library(frontier_bench_payload32_lib STATIC ${FRONTIER_SOURCES})
    frontier_configure_library(frontier_bench_payload32_lib
                               ${FRONTIER_EFFECTIVE_BVH_WIDTH})
    target_compile_definitions(frontier_bench_payload32_lib PUBLIC
        FRONTIER_USER_PAYLOAD=::std::uint32_t
        FRONTIER_INVALID_PAYLOAD=UINT32_MAX)
endif()

# A normal unit-test build always compiles and executes the other branch width
# as well. This prevents the configured/default width from silently leaving
# the other serialized layout and traversal implementation untested.
if(FRONTIER_BUILD_TESTS)
    if(FRONTIER_EFFECTIVE_BVH_WIDTH EQUAL 8)
        set(FRONTIER_ALTERNATE_BVH_WIDTH 4)
    else()
        set(FRONTIER_ALTERNATE_BVH_WIDTH 8)
    endif()
    add_library(frontier_test_alternate STATIC ${FRONTIER_SOURCES})
    frontier_configure_library(frontier_test_alternate
                               ${FRONTIER_ALTERNATE_BVH_WIDTH})

    if(FRONTIER_TEST_PAYLOAD32)
        add_library(frontier_test_payload32 STATIC ${FRONTIER_SOURCES})
        frontier_configure_library(frontier_test_payload32
                                   ${FRONTIER_EFFECTIVE_BVH_WIDTH})
        target_compile_definitions(frontier_test_payload32 PUBLIC
            FRONTIER_USER_PAYLOAD=::std::uint32_t
            FRONTIER_INVALID_PAYLOAD=UINT32_MAX)

        add_library(frontier_test_payload32_alternate STATIC ${FRONTIER_SOURCES})
        frontier_configure_library(frontier_test_payload32_alternate
                                   ${FRONTIER_ALTERNATE_BVH_WIDTH})
        target_compile_definitions(frontier_test_payload32_alternate PUBLIC
            FRONTIER_USER_PAYLOAD=::std::uint32_t
            FRONTIER_INVALID_PAYLOAD=UINT32_MAX)
    endif()
endif()

# ---- external deps via FetchContent ------------------------------------------
include(FetchContent)

if(FRONTIER_BUILD_TESTS)
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

if(FRONTIER_BUILD_BENCH)
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

if(FRONTIER_BUILD_CITY_SAMPLE)
    add_subdirectory(examples/city)
endif()
