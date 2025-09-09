cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(/usr/local/cuda/include)

add_subdirectory(camera)
add_subdirectory(localization)
add_subdirectory(calibration)


add_executable(main main.cc)
target_link_libraries(main PRIVATE ${OpenCV_LIBS} apriltag ntcore camera nlohmann_json::nlohmann_json Eigen3::Eigen localization)

add_executable(test test.cc)
target_link_libraries(test PRIVATE ${OpenCV_LIBS} 971apriltag apriltag cameraserver ntcore cscore wpiutil camera nlohmann_json::nlohmann_json Eigen3::Eigen localization)
