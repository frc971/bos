cmake_minimum_required(VERSION 3.10)

add_library(localization position_sender.cc  gpu_apriltag_detector.cc run_localization.cc nvidia_apriltag_detector.cc square_solver.cc get_field_relitive_position.cc)
target_link_libraries(localization PUBLIC ${OpenCV_LIBS} 971apriltag ntcore apriltag nlohmann_json::nlohmann_json wpilibc utils camera)
