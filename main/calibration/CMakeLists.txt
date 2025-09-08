cmake_minimum_required(VERSION 3.10)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

add_executable(calibrate calibrate.cc)
target_link_libraries(calibrate PRIVATE ${OpenCV_LIBS} nlohmann_json::nlohmann_json)

add_executable(frame_logger frame_logger.cc)
target_include_directories(frame_logger PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(frame_logger PRIVATE ${OpenCV_LIBS} camera)

add_executable(test_intrinsics test_intrinsics.cc)
target_include_directories(test_intrinsics PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(test_intrinsics PRIVATE ${OpenCV_LIBS} camera nlohmann_json::nlohmann_json)
