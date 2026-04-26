enable_testing()

add_library(unit_test_utils unit_test_utils.cc)
target_link_libraries(unit_test_utils wpilibc absl::check nlohmann_json::nlohmann_json)

add_executable(joint_solve_test joint_solve_test.cc)
target_link_libraries(joint_solve_test PRIVATE localization utils GTest::gtest GTest::gtest_main unit_test_utils)

add_executable(multi_tag_test  multi_tag_test.cc)
target_link_libraries(multi_tag_test PRIVATE localization utils GTest::gtest_main)

add_executable(square_solve_test  square_solve_test.cc)
target_link_libraries(square_solve_test PRIVATE localization utils GTest::gtest_main)

add_executable(general_solver_test  general_solver_test.cc)
target_link_libraries(general_solver_test PRIVATE localization utils GTest::gtest_main)

add_executable(matrix  matrix.cc)
target_link_libraries(matrix PRIVATE localization utils GTest::gtest_main)

add_executable(pathing_test pathing_test.cc)
target_link_libraries(pathing_test PRIVATE pathing utils nlohmann_json::nlohmann_json GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(multi_tag_test)
gtest_discover_tests(square_solve_test)
