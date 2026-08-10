# Camera

This directory defines the camera abstraction used by localization, calibration, gamepiece detection, tests, and replay tooling.

## Files

- `CMakeLists.txt` builds the `camera` library from the camera implementations and helpers.
- `camera.h` defines `camera::TimestampedFrame`, the common frame-plus-timestamp payload, and `camera::ICamera`, the interface implemented by all frame sources.
- `camera_constants.h` declares `camera::CameraConstant`, `camera::DetectorType`, the camera-constant map type, and loaders for configured camera constants.
- `camera_constants.cc` loads camera configuration data from JSON into `CameraConstant` records.
- `camera_source.h` / `camera_source.cc` wrap one `ICamera` in a background acquisition thread and expose the most recent timestamped frame safely.
- `multi_camera_source.h` / `multi_camera_source.cc` coordinate several `ICamera` instances, including replay-oriented modes that wait for consumers to use every frame.
- `cv_camera.h` / `cv_camera.cc` adapt OpenCV/GStreamer `cv::VideoCapture` cameras to the `ICamera` interface and optionally log captured frames.
- `uvc_camera.h` / `uvc_camera.cc` adapt libuvc devices to `ICamera`, including UVC-specific setup, frame buffering, restart, and cleanup.
- `disk_camera.h` / `disk_camera.cc` replay timestamped image logs from disk as an `ICamera`, preserving logged timing with optional speed, start, and end controls.
- `cscore_streamer.h` / `cscore_streamer.cc` publish OpenCV frames through WPILib CSCore MJPEG streams for debugging and dashboards.
- `select_camera.h` / `select_camera.cc` choose a configured camera interactively or from a provided camera name.
- `write_frame.h` / `write_frame.cc` write timestamped frames to disk for later replay.

## Main Types

- `camera::ICamera` is the polymorphic interface for live cameras and replay sources.
- `camera::CameraSource` is a frame provider which independently spins up a thread to read the latest frame.
- `camera::MultiCameraSource` is basically a bundle of camera sources, but it is responsible for maintaining all the latest frames across 2+ cameras.
- `camera::CVCamera`, `camera::UVCCamera`, and `camera::DiskCamera` provide the concrete live and replay frame sources.
  - `camera::CVCamera` is the standard camera stream reader, can process anything like IMX or USB. WARNING usb cameras using usb 2.0 will claim too much bandwidth over this protocol, so use `camera::UVCCamera` instead.
  - `camera::UVCCamera` is for USB specifically, and manually adjusts payload size so that we can get many simultaneous USB camera feeds.
  - `camera::DiskCamera` is for testing, and can process frames in a specific time range and go faster if requested.
- `camera::CscoreStreamer` is the standard debug-stream output path.
