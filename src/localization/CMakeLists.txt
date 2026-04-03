cmake_minimum_required(VERSION 3.10)

add_library(localization position_sender.cc position_solver.cc gpu_apriltag_detector.cc opencv_apriltag_detector.cc run_localization.cc nvidia_apriltag_detector.cc square_solver.cc joint_solver.cc multi_tag_solver.cc position_receiver.cc)
target_link_libraries(localization PRIVATE 971apriltag utils camera wpilibc wpiutil)
