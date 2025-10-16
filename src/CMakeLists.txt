cmake_minimum_required(VERSION 3.10)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(/usr/local/cuda/include)

add_subdirectory(camera)
add_subdirectory(localization)
add_subdirectory(calibration)
add_subdirectory(yolo)

add_executable(localization_main localization_main.cc)
target_link_libraries(localization_main PRIVATE ${OpenCV_LIBS} apriltag ntcore camera nlohmann_json::nlohmann_json Eigen3::Eigen localization)

add_executable(game_peice_main game_peice_main.cc)
target_link_libraries(game_peice_main PRIVATE ${OpenCV_LIBS} apriltag ntcore camera nlohmann_json::nlohmann_json Eigen3::Eigen yolo)

add_executable(test test.cc)
target_link_libraries(test PRIVATE ${OpenCV_LIBS} 971apriltag apriltag cameraserver ntcore cscore wpiutil camera nlohmann_json::nlohmann_json Eigen3::Eigen localization)
