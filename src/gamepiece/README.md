# Gamepiece

This directory connects YOLO object detections to robot-relative gamepiece positions. TODO extend

## Files

- `CMakeLists.txt` builds the `gamepiece` library.
- `gamepiece.h` declares the gamepiece detection entry points.
- `gamepiece.cc` runs YOLO on camera frames, estimates coral and algae positions from camera intrinsics/extrinsics and bounding-box centers, publishes the resulting `frc::Pose2d` values to NetworkTables, and provides a no-image inference path for model testing.

## Main Functions

- `gamepiece::run_gamepiece_detect` consumes frames from `camera::CameraSource`, runs a `yolo::Yolo` model, converts detections into robot-relative poses, and publishes coral/algae pose topics.
- `gamepiece::run_gamepiece_detect_no_img` exercises the YOLO model without the live camera-position publishing loop.
