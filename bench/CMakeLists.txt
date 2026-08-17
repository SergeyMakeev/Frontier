add_executable(frontier_bench bench_frontier.cpp)
target_link_libraries(frontier_bench PRIVATE frontier_bench_payload64_lib benchmark::benchmark Threads::Threads)
target_include_directories(frontier_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../tests)
target_compile_definitions(frontier_bench PRIVATE FRONTIER_OMIT_SUBMISSION_BENCH=1)
if(FRONTIER_IPO)
    set_property(TARGET frontier_bench PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
endif()

add_executable(frontier_bench_payload32 bench_frontier.cpp)
target_link_libraries(frontier_bench_payload32 PRIVATE frontier_bench_payload32_lib benchmark::benchmark Threads::Threads)
target_include_directories(frontier_bench_payload32 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../tests)
target_compile_definitions(frontier_bench_payload32 PRIVATE FRONTIER_OMIT_SUBMISSION_BENCH=1)
if(FRONTIER_IPO)
    set_property(TARGET frontier_bench_payload32 PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
endif()

# Keep the realistic downstream-consumption case in separate executables. A
# large benchmark function in the primary executable can otherwise perturb the
# LTO/link layout of the production traversal code it is trying to measure.
add_executable(frontier_submission_bench bench_frontier.cpp)
target_link_libraries(frontier_submission_bench PRIVATE
    frontier_bench_payload64_lib benchmark::benchmark Threads::Threads)
target_include_directories(frontier_submission_bench PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../tests)
if(FRONTIER_IPO)
    set_property(TARGET frontier_submission_bench PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
endif()

add_executable(frontier_submission_bench_payload32 bench_frontier.cpp)
target_link_libraries(frontier_submission_bench_payload32 PRIVATE
    frontier_bench_payload32_lib benchmark::benchmark Threads::Threads)
target_include_directories(frontier_submission_bench_payload32 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../tests)
if(FRONTIER_IPO)
    set_property(TARGET frontier_submission_bench_payload32 PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
endif()

# Kept separate from frontier_bench deliberately: these machine-characterization
# probes must not change the code layout of the end-to-end library benchmarks.
add_executable(frontier_machine_bench bench_machine.cpp)
target_link_libraries(frontier_machine_bench PRIVATE frontier benchmark::benchmark Threads::Threads)
target_include_directories(frontier_machine_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../tests)
if(FRONTIER_IPO)
    set_property(TARGET frontier_machine_bench PROPERTY INTERPROCEDURAL_OPTIMIZATION ON)
endif()
if(MSVC)
    target_compile_options(frontier_machine_bench PRIVATE /W4 /permissive- /fp:precise)
else()
    target_compile_options(frontier_machine_bench PRIVATE
        -Wall -Wextra -ffp-contract=off)
endif()
