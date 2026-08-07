add_executable(hlod_bench bench_hlod.cpp)
target_link_libraries(hlod_bench PRIVATE hlod benchmark::benchmark_main Threads::Threads)
target_include_directories(hlod_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../tests)

# Kept separate from hlod_bench deliberately: these machine-characterization
# probes must not change the code layout of the end-to-end library benchmarks.
add_executable(hlod_machine_bench bench_machine.cpp)
target_link_libraries(hlod_machine_bench PRIVATE hlod benchmark::benchmark Threads::Threads)
if(MSVC)
    target_compile_options(hlod_machine_bench PRIVATE /W4 /permissive- /fp:precise)
else()
    target_compile_options(hlod_machine_bench PRIVATE
        -Wall -Wextra -ffp-contract=off)
endif()
