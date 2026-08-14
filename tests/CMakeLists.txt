set(FRONTIER_TEST_SOURCES
    test_build_mode.cpp
    test_math.cpp
    test_builder.cpp
    test_frontier.cpp
    test_streaming.cpp
    test_motion.cpp
    test_tlas.cpp
    test_cache.cpp
    test_contracts.cpp
    test_torture.cpp
)

add_executable(frontier_tests ${FRONTIER_TEST_SOURCES})
target_link_libraries(frontier_tests PRIVATE frontier GTest::gtest_main Threads::Threads)

add_executable(frontier_tests_alternate ${FRONTIER_TEST_SOURCES})
target_link_libraries(frontier_tests_alternate PRIVATE
    frontier_test_alternate GTest::gtest_main Threads::Threads)

set(FRONTIER_UNIT_TEST_TARGETS frontier_tests frontier_tests_alternate)

if(FRONTIER_TEST_PAYLOAD32)
    add_executable(frontier_tests_payload32 ${FRONTIER_TEST_SOURCES})
    target_link_libraries(frontier_tests_payload32 PRIVATE
        frontier_test_payload32 GTest::gtest_main Threads::Threads)

    add_executable(frontier_tests_payload32_alternate ${FRONTIER_TEST_SOURCES})
    target_link_libraries(frontier_tests_payload32_alternate PRIVATE
        frontier_test_payload32_alternate GTest::gtest_main Threads::Threads)

    list(APPEND FRONTIER_UNIT_TEST_TARGETS
        frontier_tests_payload32 frontier_tests_payload32_alternate)
endif()

include(GoogleTest)
gtest_discover_tests(frontier_tests
    TEST_PREFIX "BVH${FRONTIER_EFFECTIVE_BVH_WIDTH}."
    DISCOVERY_TIMEOUT 60)
gtest_discover_tests(frontier_tests_alternate
    TEST_PREFIX "BVH${FRONTIER_ALTERNATE_BVH_WIDTH}."
    DISCOVERY_TIMEOUT 60)

if(FRONTIER_TEST_PAYLOAD32)
    gtest_discover_tests(frontier_tests_payload32
        TEST_PREFIX "Payload32.BVH${FRONTIER_EFFECTIVE_BVH_WIDTH}."
        DISCOVERY_TIMEOUT 60)
    gtest_discover_tests(frontier_tests_payload32_alternate
        TEST_PREFIX "Payload32.BVH${FRONTIER_ALTERNATE_BVH_WIDTH}."
        DISCOVERY_TIMEOUT 60)
endif()

add_custom_target(frontier_unit_tests
    DEPENDS ${FRONTIER_UNIT_TEST_TARGETS})
