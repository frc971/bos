add_executable(apriltag_detect_test apriltag_detect_test.cc)
target_link_libraries(apriltag_detect_test PRIVATE ${OpenCV_LIBS} 971apriltag apriltag cameraserver ntcore cscore wpiutil camera nlohmann_json::nlohmann_json Eigen3::Eigen localization utils)

add_executable(tag_estimator_test tag_estimator_test.cc)
target_link_libraries(tag_estimator_test PRIVATE ${OpenCV_LIBS} 971apriltag apriltag cameraserver ntcore cscore wpiutil camera nlohmann_json::nlohmann_json Eigen3::Eigen localization)

add_executable(intrinsics_test intrinsics_test.cc)
target_link_libraries(intrinsics_test PRIVATE ${OpenCV_LIBS} camera nlohmann_json::nlohmann_json)

add_executable(yolo_test yolo_test.cc)
target_link_libraries(yolo_test PRIVATE camera yolo utils)

add_executable(camera_source_test camera_source_test.cc)
target_link_libraries(camera_source_test PRIVATE ${OpenCV_LIBS} camera)

add_executable(timer_test timer_test.cc)
target_link_libraries(timer_test PRIVATE utils)

add_executable(stress_test stress_test.cc)
target_link_libraries(stress_test PRIVATE ${OpenCV_LIBS} apriltag ntcore camera nlohmann_json::nlohmann_json Eigen3::Eigen localization utils yolo)

add_executable(path_plan_test path_plan_test.cc)
target_link_libraries(path_plan_test PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

add_executable(gamepiece_test gamepiece_test.cc)
target_link_libraries(gamepiece_test PRIVATE gamepiece yolo camera utils localization)

add_executable(nvidia_apriltag_test nvidia_apriltag_test.cc)
target_link_libraries(nvidia_apriltag_test PRIVATE ${OpenCV_LIBS} apriltag ntcore camera nlohmann_json::nlohmann_json Eigen3::Eigen localization utils yolo vpi utils)

