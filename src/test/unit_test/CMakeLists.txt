add_executable(joint_solve_test joint_solve_test.cc)
target_link_libraries(joint_solve_test localization absl::flags absl::flags_parse utils wpilibc apriltag GTest::gtest GTest::gtest_main)
