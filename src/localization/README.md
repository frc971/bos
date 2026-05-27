# Localization

This directory contains AprilTag detection, pose solving, estimate disambiguation, and publishing/receiving code for robot localization.

## Files

- `position.h` defines shared data records: `TagDetection`, `PositionEstimate`, and `Point3d`.
- `apriltag_detector.h` defines `IAprilTagDetector`, the common interface for all AprilTag detector backends.
- `gpu_apriltag_detector.h` / `gpu_apriltag_detector.cc` wrap the 971 GPU AprilTag detector and convert GPU detections into project `TagDetection` records.
- `opencv_apriltag_detector.h` / `opencv_apriltag_detector.cc` implement a CPU AprilTag 36h11 detector using OpenCV ArUco APIs and undistort returned corners.
- `nvidia_apriltag_detector.h` / `nvidia_apriltag_detector.cc` wrap NVIDIA VPI AprilTag detection for PVA or CPU backends.
- `position_solver.h` defines shared AprilTag geometry constants, the 2026 field layout, estimate variance helper, and `IPositionSolver`.
- `square_solver.h` / `square_solver.cc` solve single-tag poses using OpenCV `SOLVEPNP_IPPE_SQUARE` and convert camera-relative tag poses into robot field poses.
- `multi_tag_solver.h` / `multi_tag_solver.cc` solve one pose from multiple tag observations, while retaining single-tag ambiguous estimates for disambiguation.
- `joint_solver.h` / `joint_solver.cc` refine a robot pose from observations across multiple named cameras using camera matrices and field tag geometry.
- `unambiguous_estimator.h` / `unambiguous_estimator.cc` run a multi-camera localization pipeline, collect ambiguous estimates, choose a consistent pose, stream diagnostics, and publish camera status.
- `position_sender.h` defines `IPositionSender`, the output interface for pose estimate sinks.
- `networktable_sender.h` / `networktable_sender.cc` publish pose estimates, metadata, tag IDs, rejected tags, latency, loss, and logs through NetworkTables/WPILib logging.
- `simulation_sender.h` / `simulation_sender.cc` draw estimated robot poses onto a field image and stream the visualization.
- `position_receiver.h` / `position_receiver.cc` subscribe to the robot drive pose topic and expose the current `frc::Pose2d`.
- `run_localization.h` / `run_localization.cc` compose a `CameraSource`, detector, solver, and senders into the continuous localization loop, with a simulation replay variant.

## Main Interfaces

- `localization::IAprilTagDetector` converts images into tag corner detections.
- `localization::IPositionSolver` converts tag detections into robot pose estimates.
- `localization::IPositionSender` publishes or visualizes pose estimates.

## How to Choose Solvers

- `localization::UnambiguousEstimator` is the default for multiple camera streams (TODO when joint solve is tested this will become the default).
- `localization::MultiTagSolver` is the default for single camera multiple tag detections. It has the potential to be ambiguous with only 1 tag.
- `localization::SquareSolver` should be most accurate for single tag detections (TODO run rigorous test) because it uses more constraints. High rate of ambiguity, should never be used without some method of handling ambiguity.
- `localization::JointSolver` is in progress (TODO update when joint solve is merged) and will be implemented on top of unambiguous to achieve the most stable pose estimates
