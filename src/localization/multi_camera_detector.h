#pragma once
#include <condition_variable>
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
  MultiCameraDetector(std::vector<camera::camera_constant_t> camera_constants,
                      std::optional<std::vector<std::filesystem::path>>
                          image_paths = std::nullopt,
                      bool wait_for_all_detections = false);
  [[nodiscard]] auto GetTagDetections()
      -> std::vector<std::vector<tag_detection_t>>;
  [[nodiscard]] auto GetCVFrames() -> std::vector<cv::Mat>;
  [[nodiscard]] inline auto NumCameras() -> double { return cameras_.size(); }
  ~MultiCameraDetector();

 private:
  std::vector<camera::camera_constant_t> camera_constants_;
  std::vector<std::unique_ptr<camera::ICamera>> cameras_;
  std::vector<std::unique_ptr<IAprilTagDetector>> detectors_;
  std::vector<camera::CscoreStreamer> streamers_;
  std::vector<double> last_write_times_;
  std::vector<camera::timestamped_frame_t> timestamped_frames_;
  std::vector<std::vector<tag_detection_t>> tag_detections_;
  std::vector<uint64_t> detection_counts_;
  std::vector<uint64_t> consumed_detection_counts_;
  std::vector<std::jthread> camera_threads_;
  std::mutex mutex_;
  std::condition_variable detection_cv_;
  std::atomic<bool> run_cameras_{true};
  const bool wait_for_all_detections_;
  static constexpr int kdefault_stream_fps = 30;
};

}  // namespace localization
