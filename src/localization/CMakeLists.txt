add_library(localization)
target_sources(localization PRIVATE
  gpu_apriltag_detector.cc 
  opencv_apriltag_detector.cc 
  run_localization.cc
  nvidia_apriltag_detector.cc
  square_solver.cc 
  joint_solver.cc 
  multi_tag_solver.cc 
  position_receiver.cc 
  unambiguous_estimator.cc 
  simulation_sender.cc 
  networktable_sender.cc
  multi_camera_detector.cc
)
target_link_libraries(localization PRIVATE 
  971apriltag 
  utils 
  camera 
)

add_library(GPUApriltagDetector)
target_sources(GPUApriltagDetector
  PRIVATE
    gpu_apriltag_detector.cc

  PUBLIC
    FILE_SET HEADERS
    BASE_DIRS
    FILES
      gpu_apriltag_detector.h
)
target_link_libraries(GPUApriltagDetector PRIVATE
  971apriltag
  utils
)
