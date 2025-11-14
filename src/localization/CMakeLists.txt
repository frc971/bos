cmake_minimum_required(VERSION 3.10)

add_library(localization simple_kalman.cc tag_estimator.cc position_sender.cc pose_estimator.cc)
target_include_directories(localization PUBLIC /usr/local/cuda/include)
target_link_libraries(localization PUBLIC ${OpenCV_LIBS} Eigen3::Eigen 971apriltag ntcore apriltag nlohmann_json::nlohmann_json wpilibc kalman-cpp)
