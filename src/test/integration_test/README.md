# Integration Tests

This directory contains executable diagnostics and end-to-end tests for camera, localization, pathing, NetworkTables, YOLO, and NVIDIA VPI behavior. Many of these programs are interactive or hardware-dependent rather than hermetic CI tests.

## Files

- `CMakeLists.txt` builds each integration test executable.
- `apriltag_detect_test.cc` opens a configured camera, runs the GPU AprilTag detector, solves poses with `SquareSolver`, logs estimates, and streams an annotated frame.
- `bfs_tester.cc` loads the navigation grid, runs BFS and spline helpers, and displays/prints pathfinding behavior for manual inspection.
- `gamepiece_test.cc` opens a camera, constructs the color YOLO model, and starts the gamepiece detection loop publishing coral and algae poses.
- `intrinsics_test.cc` loads camera intrinsics, streams raw and undistorted camera frames side by side, and validates calibration files visually.
- `localization_test.cc` replays logged camera images from one or more folders, resolves camera constants, runs localization, and streams/publishes simulation outputs.
- `localization_test2.cc` runs the multi-camera `UnambiguousEstimator` against left/right replay folders.
- `networktable_performance_test.cc` subscribes to the drive pose topic and prints observed NetworkTables update frequency and period.
- `path_plan_test.cc` is a standalone visual path-planning playground with its own grid node class and drawing helpers.
- `pva_test.cc` is an NVIDIA VPI AprilTag sample-style test for PVA/CPU AprilTag detection and visualization.
- `solver_test.cc` feeds synthetic tag-corner observations into `SquareSolver` and prints resulting pose estimates.
- `stress_test.cc` is an older multi-threaded localization stress harness for several camera/detector/solver pipelines.
- `yolo_test.cc` opens a camera, runs YOLO inference, draws detections, estimates object angle, and streams the annotated result.
