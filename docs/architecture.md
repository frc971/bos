# Architecture Overview

## High-Level Flow

The runtime system is built around a vision and localization pipeline:

1. Camera modules capture frames.
2. AprilTag detectors produce per-frame detections.
3. Solvers estimate robot pose from detections.
4. Position senders publish results to NetworkTables (and optional simulation outputs).
5. Pathing and controller logic consume localization output and publish control intents.

## Source Tree Map

### `src/camera`

- Camera interfaces and implementations (`cv`, `uvc`, disk replay).
- Frame production wrappers (`CameraSource`, `MultiCameraSource`).
- Optional CSCore streaming and frame writing utilities.
- Camera constants loading from JSON.

### `src/localization`

- AprilTag detector interface and implementations:
  - OpenCV ArUco AprilTag
  - 971 GPU AprilTag
  - NVIDIA VPI AprilTag
- Pose solvers:
  - `SquareSolver` (single tag)
  - `MultiTagSolver` (multi-tag)
  - `JointSolver` (multi-camera optimization)
  - `UnambiguousEstimator` (ambiguity resolution and fusion)
- Main localization loop (`run_localization`).
- Publishers and receivers for position data.

### `src/pathing`

- BFS grid pathfinding.
- Spline generation.
- Control loop integration for command publication.

### `src/yolo` and `src/gamepiece`

- TensorRT-based YOLO wrapper.
- Gamepiece pipeline built on top of camera + yolo + localization utilities.

### `src/utils`

- Shared cross-cutting helpers:
  - logging
  - timers
  - transforms
  - NetworkTables utility helpers
  - JSON constant handling

### `src/calibration`

- Intrinsics calibration tools.
- Frame display and focus calibration binaries.

### `src/test`

- `unit_test`: solver and pathing focused tests.
- `integration_test`: end-to-end and subsystem executable tests.

## Top-Level Binaries

Defined in `src/CMakeLists.txt`:

- `main_bot_main`
- `second_bot_main`
- `unambiguous_first`
- `unambiguous_second`

## Dependency Snapshot

Major linked dependencies across modules:

- OpenCV
- Eigen3
- WPILib (`wpilibc`, `wpiutil`, `ntcore`)
- libuvc
- NVIDIA VPI
- TensorRT / CUDA (`nvinfer`, `nvinfer_plugin`, `cudart`)
- 971 AprilTag
- abseil
- nlohmann/json
- GoogleTest
