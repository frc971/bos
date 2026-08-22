include(GoogleTest)
enable_testing()

add_executable(joint_solve_test joint_solve_test.cc)
target_link_libraries(joint_solve_test PRIVATE localization utils GTest::gtest GTest::gtest_main)

add_executable(multi_tag_test  multi_tag_test.cc)
target_link_libraries(multi_tag_test PRIVATE localization utils GTest::gtest_main)

add_executable(square_solve_test  square_solve_test.cc)
target_link_libraries(square_solve_test PRIVATE localization utils GTest::gtest_main)

add_executable(general_solver_test  general_solver_test.cc)
target_link_libraries(general_solver_test PRIVATE localization utils GTest::gtest_main)

add_executable(matrix  matrix.cc)
target_link_libraries(matrix PRIVATE localization utils GTest::gtest_main)

add_executable(ellipse_test ellipse_test.cc)
target_link_libraries(ellipse_test PRIVATE gamepiece GTest::gtest_main)

add_executable(hsv_cluster_visualization_test hsv_cluster_visualizer.cc)
target_compile_definitions(hsv_cluster_visualization_test PRIVATE
  BOS_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(hsv_cluster_visualization_test PRIVATE gamepiece GTest::gtest_main)

add_executable(pathing_test pathing_test.cc)
target_link_libraries(pathing_test PRIVATE pathing utils nlohmann_json::nlohmann_json GTest::gtest_main)

add_executable(simulated_uvc_camera_test simulated_uvc_camera_test.cc)
target_compile_definitions(simulated_uvc_camera_test PRIVATE
  BOS_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(simulated_uvc_camera_test PRIVATE
  SimulatedUVCCamera
  GTest::gtest_main
)

gtest_discover_tests(joint_solve_test)
gtest_discover_tests(multi_tag_test)
gtest_discover_tests(square_solve_test)
gtest_discover_tests(general_solver_test)
gtest_discover_tests(ellipse_test)
gtest_discover_tests(hsv_cluster_visualization_test)
gtest_discover_tests(simulated_uvc_camera_test)
# gtest_discover_tests(pathing_test)
