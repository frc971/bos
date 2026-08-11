# Utils

This directory contains shared helpers for camera JSON files, NetworkTables setup, logging, timing, and coordinate transforms.

## Files

- `CMakeLists.txt` builds the `utils` library.
- `pch.h` centralizes commonly used standard-library, OpenCV, Eigen, WPILib, NetworkTables, and JSON includes.
- `camera_utils.h` / `camera_utils.cc` read camera intrinsic and extrinsic JSON files from disk.
- `constants_from_json.h` / `constants_from_json.cc` convert JSON camera calibration data into OpenCV, Eigen, WPILib, and project transform types.
- `log.h` / `log.cc` print WPILib poses, transforms, and OpenCV transformation matrices in readable form.
- `log_path.h` / `log_path.cc` generate new log paths under the configured log directory.
- `nt_utils.h` / `nt_utils.cc` start and configure NetworkTables for the robot/team environment.
- `timer.h` / `timer.cc` define `utils::Timer`, an RAII-style timer used to measure and optionally print elapsed time for a scoped operation.
- `transform.h` / `transform.cc` provide coordinate-system conversion and transform helpers between OpenCV, Eigen, and WPILib conventions.

## Main Types And Functions

- `utils::Timer` measures runtime until `Stop()` or destruction.
- `utils::CameraMatrixFromJson` and `utils::DistortionCoefficientsFromJson` create calibration matrices from intrinsic JSON.
- `utils::ExtrinsicsJsonToCameraToRobot` converts extrinsic JSON into a WPILib camera-to-robot transform.
- `utils::ChangeBasis`, `utils::Pose3dToCvMat`, and `utils::ConvertOpencvTransformationMatrixToWpilibPose` keep pose math consistent across libraries.
- `utils::StartNetworktables` initializes NetworkTables for runtime programs.
