#pragma once
#include "src/camera/camera.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/disk_camera.h"
#include "src/camera/simulated_uvc_camera.h"
#include "src/localization/apriltag_detector.h"
#include "src/localization/position.h"
#include "src/localization/position_solver.h"
#include "src/utils/pch.h"
namespace localization {

class MultiCameraDetector {
 public:
  MultiCameraDetector(std::vector<camera::camera_constant_t> camera_constants,
                      std::optional<std::vector<std::filesystem::path>>
                          image_paths = std::nullopt,
                      double disk_replay_speed = 1.0,
                      std::optional<camera::test::FrameFailureProbabilities>
                          uvc_failure_probabilities = std::nullopt);
  [[nodiscard]] auto GetTagDetections()
      -> std::vector<std::vector<tag_detection_t>>;
  [[nodiscard]] auto GetCVFrames() -> std::vector<cv::Mat>;
  [[nodiscard]] inline auto NumCameras() -> double { return cameras_.size(); }
  [[nodiscard]] auto IsDone() const -> bool { return finished_; }
  ~MultiCameraDetector();

 private:
  std::vector<camera::camera_constant_t> camera_constants_;
  std::vector<std::unique_ptr<camera::ICamera>> cameras_;
  std::vector<std::unique_ptr<IAprilTagDetector>> detectors_;
  std::vector<camera::CscoreStreamer> streamers_;
  std::vector<double> last_write_times_;
  std::vector<camera::timestamped_frame_t> timestamped_frames_;
  std::vector<std::vector<tag_detection_t>> tag_detections_;
  std::vector<std::jthread> camera_threads_;
  std::mutex mutex_;
  std::atomic<bool> run_cameras_{true};
  std::atomic<bool> has_new_detections_{false};
  std::atomic<size_t> active_cameras_;
  std::atomic<bool> finished_{false};
  static constexpr int kdefault_stream_fps = 30;
};

}  // namespace localization
