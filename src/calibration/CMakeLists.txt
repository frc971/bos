cmake_minimum_required(VERSION 3.10)

find_package(OpenCV REQUIRED)


add_library(intrinsics_calibrate_lib intrinsics_calibrate_lib.cc)
target_link_libraries(intrinsics_calibrate_lib PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

add_executable(intrinsics_calibrate intrinsics_calibrate.cc)
target_link_libraries(intrinsics_calibrate PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json camera intrinsics_calibrate_lib)

add_executable(extrinsics_calibrate extrinsics_calibrate.cc)
target_link_libraries(extrinsics_calibrate PRIVATE ${OpenCV_LIBS} camera nlohmann_json::nlohmann_json localization)

add_executable(frame_logger frame_logger.cc)
target_link_libraries(frame_logger PRIVATE ${OpenCV_LIBS} camera)

add_executable(test_intrinsics test_intrinsics.cc)
target_link_libraries(test_intrinsics PRIVATE ${OpenCV_LIBS} camera nlohmann_json::nlohmann_json)

add_executable(focus_calibrate focus_calibrate.cc)
target_link_libraries(focus_calibrate PRIVATE ${OpenCV_LIBS} camera nlohmann_json::nlohmann_json)

target_precompile_headers(intrinsics_calibrate_lib PRIVATE ${CMAKE_SOURCE_DIR}/include/pch.h)
target_precompile_headers(intrinsics_calibrate PRIVATE ${CMAKE_SOURCE_DIR}/include/pch.h)
target_precompile_headers(extrinsics_calibrate PRIVATE ${CMAKE_SOURCE_DIR}/include/pch.h)
target_precompile_headers(frame_logger PRIVATE ${CMAKE_SOURCE_DIR}/include/pch.h)
target_precompile_headers(test_intrinsics PRIVATE ${CMAKE_SOURCE_DIR}/include/pch.h)
target_precompile_headers(focus_calibrate PRIVATE ${CMAKE_SOURCE_DIR}/include/pch.h)
