add_library(GPUApriltagDetector)
target_sources(GPUApriltagDetector
  PRIVATE
    gpu_apriltag_detector.cc
)
target_link_libraries(GPUApriltagDetector PRIVATE
  971apriltag
  utils
)

add_library(OpencvApriltagDetector)
target_sources(OpencvApriltagDetector
  PRIVATE
    opencv_apriltag_detector.cc
)
target_link_libraries(OpencvApriltagDetector PRIVATE
  utils
)

add_library(NvidiaApriltagDetector)
target_sources(NvidiaApriltagDetector
  PRIVATE
    nvidia_apriltag_detector.cc
)
target_link_libraries(NvidiaApriltagDetector PRIVATE
  utils
)

add_library(SquareSolver)
target_sources(SquareSolver
  PRIVATE
    square_solver.cc
)
target_link_libraries(SquareSolver PRIVATE
  utils
)

add_library(JointSolver)
target_sources(JointSolver
  PRIVATE
   joint_solver.cc
)
target_link_libraries(JointSolver PRIVATE
  utils
)

add_library(MultitagSolver)
target_sources(MultitagSolver
  PRIVATE
    multi_tag_solver.cc
)
target_link_libraries(MultitagSolver PRIVATE
  utils
  SquareSolver
)

add_library(PositionReceiver)
target_sources(PositionReceiver
  PRIVATE
    position_receiver.cc
)
target_link_libraries(PositionReceiver PRIVATE
  utils
)

add_library(UnambiguousEstimator)
target_sources(UnambiguousEstimator
  PRIVATE
    unambiguous_estimator.cc
)
target_link_libraries(UnambiguousEstimator PRIVATE
  SquareSolver
  MultitagSolver
  utils
)

add_library(SimulationSender)
target_sources(SimulationSender
  PRIVATE
    simulation_sender.cc
)
target_link_libraries(SimulationSender PRIVATE
  utils
)

add_library(NetworktableSender)
target_sources(NetworktableSender
  PRIVATE
    networktable_sender.cc
)
target_link_libraries(NetworktableSender PRIVATE
  utils
)

add_library(localization)
target_sources(localization 
  PRIVATE
    run_localization.cc
    multi_camera_detector.cc
)
target_link_libraries(localization PUBLIC 
  GPUApriltagDetector
  OpencvApriltagDetector
  NvidiaApriltagDetector
  SquareSolver
  JointSolver
  MultitagSolver
  PositionReceiver
  UnambiguousEstimator
  SimulationSender
  NetworktableSender
  utils
  camera
)
