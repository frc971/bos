cmake_minimum_required(VERSION 3.10)

find_package(OpenCV REQUIRED)

add_library(localization pose_estimator.cc position_sender.cc)
target_include_directories(localization PRIVATE /usr/local/cuda/include)
target_link_libraries(localization PRIVATE ${OpenCV_LIBS} 971apriltag ntcore apriltag nlohmann_json::nlohmann_json)
