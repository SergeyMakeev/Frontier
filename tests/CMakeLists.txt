add_executable(frontier_tests
    test_math.cpp
    test_builder.cpp
    test_frontier.cpp
    test_streaming.cpp
    test_motion.cpp
    test_tlas.cpp
    test_cache.cpp
    test_contracts.cpp
)
target_link_libraries(frontier_tests PRIVATE frontier GTest::gtest_main Threads::Threads)

include(GoogleTest)
gtest_discover_tests(frontier_tests DISCOVERY_TIMEOUT 60)
