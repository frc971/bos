enable_testing()

add_executable(multi_tag_test  multi_tag_test.cc)
target_link_libraries(multi_tag_test PRIVATE localization utils GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(multi_tag_test)
