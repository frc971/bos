cmake_minimum_required(VERSION 3.10)

include_directories(${CMAKE_SOURCE_DIR})

add_subdirectory(utils)
add_subdirectory(camera)
add_subdirectory(localization)
add_subdirectory(calibration)
add_subdirectory(yolo)
add_subdirectory(test)

add_executable(fiddler_main fiddler_main.cc)
target_link_libraries(fiddler_main PRIVATE ${OpenCV_LIBS} apriltag ntcore camera nlohmann_json::nlohmann_json Eigen3::Eigen localization utils)

add_executable(software_bot_main software_bot_main.cc)
target_link_libraries(software_bot_main PRIVATE ${OpenCV_LIBS} apriltag ntcore camera nlohmann_json::nlohmann_json Eigen3::Eigen localization utils yolo)
