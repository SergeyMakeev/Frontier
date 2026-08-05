add_executable(hlod_tests
    test_math.cpp
    test_builder.cpp
    test_cut.cpp
    test_streaming.cpp
    test_motion.cpp
    test_tlas.cpp
    test_cache.cpp
    test_contracts.cpp
)
target_link_libraries(hlod_tests PRIVATE hlod GTest::gtest_main Threads::Threads)

include(GoogleTest)
gtest_discover_tests(hlod_tests DISCOVERY_TIMEOUT 60)
