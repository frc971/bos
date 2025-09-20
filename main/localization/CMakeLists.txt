cmake_minimum_required(VERSION 3.10)

add_library(localization tag_estimator.cc position_sender.cc)
target_include_directories(localization PUBLIC /usr/local/cuda/include)
target_link_libraries(localization PUBLIC ${OpenCV_LIBS} 971apriltag ntcore apriltag nlohmann_json::nlohmann_json)
