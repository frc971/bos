add_executable(apriltag_detect_test apriltag_detect_test.cc)
target_link_libraries(apriltag_detect_test PRIVATE ${OpenCV_LIBS} 971apriltag apriltag cameraserver ntcore cscore wpiutil camera nlohmann_json::nlohmann_json Eigen3::Eigen localization)

add_executable(intrinsics_test intrinsics_test.cc)
target_link_libraries(intrinsics_test PRIVATE ${OpenCV_LIBS} camera nlohmann_json::nlohmann_json)

add_executable(realsense_yolo_test realsense_yolo_test.cc)
target_link_libraries(realsense_yolo_test PRIVATE camera yolo)

add_executable(distance_measurer distance_measurer.cc)
target_link_libraries(distance_measurer PRIVATE camera)
