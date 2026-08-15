# Calibration

This directory contains camera-calibration tools and the shared ChArUco-board utilities they use.

## Files

- `CMakeLists.txt` builds the standalone calibration executables in this directory.
- `focus_calibrate.cc` opens a selected camera, streams frames through CSCore, and prints a Laplacian-based focus score so the lens can be adjusted live.
- `frame_shower.cc` opens a selected camera and streams raw frames to a configurable port for visual inspection and camera bring-up.
- `intrinsics_calibrate.cc` is the interactive calibration executable. It captures ChArUco detections from a selected camera, generates `calibration_board.png`, calibrates the camera, and writes `intrinsics.json`.
- `intrinsics_calibrate_lib.h` declares the ChArUco calibration data structures, board constants, detector factory, drawing helpers, JSON conversion, and calibration routine.
- `intrinsics_calibrate_lib.cc` implements the shared ChArUco workflow: detector construction, board rendering, board detection, annotation, OpenCV calibration, and intrinsic-parameter JSON serialization.

## Main Types And Functions

- `calibration::DetectionResult` holds one frame's ChArUco corners, IDs, object points, and image points.
- `calibration::CreateDetector` builds an OpenCV `cv::aruco::CharucoDetector` using the project's board geometry.
- `calibration::GenerateBoard` renders the printable ChArUco board image.
- `calibration::DetectCharucoBoard` extracts calibration observations from an image.
- `calibration::CalibrateCamera` converts collected observations into an OpenCV camera matrix and distortion coefficients.
