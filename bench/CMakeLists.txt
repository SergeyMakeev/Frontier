add_executable(frontier_bench bench_frontier.cpp)
target_link_libraries(frontier_bench PRIVATE frontier benchmark::benchmark_main Threads::Threads)
target_include_directories(frontier_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../tests)

# Kept separate from frontier_bench deliberately: these machine-characterization
# probes must not change the code layout of the end-to-end library benchmarks.
add_executable(frontier_machine_bench bench_machine.cpp)
target_link_libraries(frontier_machine_bench PRIVATE frontier benchmark::benchmark Threads::Threads)
target_include_directories(frontier_machine_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../tests)
if(MSVC)
    target_compile_options(frontier_machine_bench PRIVATE /W4 /permissive- /fp:precise)
else()
    target_compile_options(frontier_machine_bench PRIVATE
        -Wall -Wextra -ffp-contract=off)
endif()
