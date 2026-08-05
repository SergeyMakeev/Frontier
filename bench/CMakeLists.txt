add_executable(hlod_bench bench_hlod.cpp)
target_link_libraries(hlod_bench PRIVATE hlod benchmark::benchmark_main Threads::Threads)
target_include_directories(hlod_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../tests)
