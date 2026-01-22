cmake_minimum_required(VERSION 3.10)

find_package(OpenCV REQUIRED)

add_executable(intrinsics_calibrate intrinsics_calibrate.cc intrinsics_calibrate_lib.cc)
target_link_libraries(intrinsics_calibrate PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json camera absl::flags_parse utils)

add_executable(frame_shower frame_shower.cc)
target_link_libraries(frame_shower PRIVATE ${OpenCV_LIBS} camera utils absl::flags absl::flags_parse utils)

add_executable(frame_logger frame_logger.cc)
target_link_libraries(frame_logger PRIVATE ${OpenCV_LIBS} camera nlohmann_json::nlohmann_json utils absl::flags absl::flags_parse utils)

add_executable(focus_calibrate focus_calibrate.cc)
target_link_libraries(focus_calibrate PRIVATE ${OpenCV_LIBS} camera nlohmann_json::nlohmann_json utils)

