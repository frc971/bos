cmake_minimum_required(VERSION 3.10)

include_directories(${CMAKE_SOURCE_DIR})

# set(CMAKE_CXX_CLANG_TIDY
#     clang-tidy;
#     -checks=modernize-*,performance-*;
# )

add_compile_options(-Wno-c99-designator)

add_subdirectory(utils)
add_subdirectory(camera)
add_subdirectory(gamepiece)
add_subdirectory(localization)
add_subdirectory(calibration)
add_subdirectory(yolo)
add_subdirectory(test)

add_executable(fiddler_main fiddler_main.cc)
target_link_libraries(fiddler_main PRIVATE ${OpenCV_LIBS} apriltag ntcore camera nlohmann_json::nlohmann_json Eigen3::Eigen localization utils)

add_executable(turret_bot_main turret_bot_main.cc)
target_link_libraries(turret_bot_main PRIVATE ${OpenCV_LIBS} apriltag ntcore camera nlohmann_json::nlohmann_json Eigen3::Eigen localization utils yolo gamepiece)

add_executable(vision_bot_main vision_bot_main.cc)
target_link_libraries(vision_bot_main PRIVATE ${OpenCV_LIBS} apriltag ntcore camera nlohmann_json::nlohmann_json Eigen3::Eigen localization utils)

# set(CMAKE_CXX_CLANG_TIDY "")
