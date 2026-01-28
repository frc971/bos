add_executable(apriltag_detect_test apriltag_detect_test.cc)
target_link_libraries(apriltag_detect_test PRIVATE ${OpenCV_LIBS} 971apriltag apriltag cameraserver ntcore cscore wpiutil camera nlohmann_json::nlohmann_json Eigen3::Eigen localization absl::flags absl::flags_parse)

add_executable(intrinsics_test intrinsics_test.cc)
target_link_libraries(intrinsics_test PRIVATE ${OpenCV_LIBS} camera nlohmann_json::nlohmann_json absl::flags absl::flags_parse)

add_executable(yolo_test yolo_test.cc)
target_link_libraries(yolo_test PRIVATE camera yolo utils)

add_executable(camera_source_test camera_source_test.cc)
target_link_libraries(camera_source_test PRIVATE ${OpenCV_LIBS} camera)

add_executable(timer_test timer_test.cc)
target_link_libraries(timer_test PRIVATE utils)

add_executable(stress_test stress_test.cc)
target_link_libraries(stress_test PRIVATE ${OpenCV_LIBS} apriltag ntcore camera nlohmann_json::nlohmann_json Eigen3::Eigen localization utils yolo absl::flags absl::flags_parse)

add_executable(path_plan_test path_plan_test.cc)
target_link_libraries(path_plan_test PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json ntcore wpilibc wpimath utils)

add_executable(gamepiece_test gamepiece_test.cc)
target_link_libraries(gamepiece_test PRIVATE gamepiece yolo camera utils localization)

add_executable(gamepiece_test_no_img gamepiece_test_no_img.cc)
target_link_libraries(gamepiece_test_no_img PRIVATE gamepiece yolo camera utils localization)

add_executable(absl_flag_test absl_flag_test.cc)
target_link_libraries(absl_flag_test absl::flags absl::flags_parse)

