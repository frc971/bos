#pragma once
#include "src/camera/camera.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/disk_camera.h"
#include "src/localization/apriltag_detector.h"
#include "src/localization/position.h"
#include "src/localization/position_solver.h"
#include "src/utils/pch.h"
namespace localization {

class MultiCameraDetector {
 public:
  MultiCameraDetector(std::vector<camera::camera_constant_t> camera_constants);
  [[nodiscard]] auto GetTagDetections()
      -> std::vector<std::vector<tag_detection_t>>;
  [[nodiscard]] auto GetCVFrames() -> std::vector<cv::Mat>;
  [[nodiscard]] inline auto NumCameras() -> double { return cameras_.size(); }
  void StartThreads();
  ~MultiCameraDetector();

 private:
  std::vector<camera::camera_constant_t> camera_constants_;
  std::vector<std::unique_ptr<camera::ICamera>> cameras_;
  std::vector<std::unique_ptr<IAprilTagDetector>> detectors_;
  std::vector<camera::CscoreStreamer> streamers_;
  std::vector<double> last_write_times_;
  std::vector<camera::timestamped_frame_t> timestamped_frames_;
  std::vector<std::vector<tag_detection_t>> tag_detections_;
  std::vector<std::thread> camera_threads_;
  std::mutex mutex_;
  std::atomic<bool> run_cameras_{true};
  static constexpr int kenforced_streamer_fps = 30;
};

}  // namespace localization
