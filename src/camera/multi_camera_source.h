#pragma once
#include "src/camera/camera.h"
#include "src/camera/disk_camera.h"
#include "src/utils/pch.h"
namespace camera {

// Takes in a ICamera and starts a thread to retreive and process the frame from that ICamera and stores it inside timestamped_frame_.
// When a frame is retreived, a cv::Mat with reference to the same underlying memory is created. And the memory should be automaticly deallocated when the reference counter reaches 0. (The reference counter decrements each time a cv::Mat gets destructed)
// cv::Mat's reference counter seems to be atomic, so it behaves similarly to a std::shared_ptr and it is safe to access the same cv::Mat across multiple threads as longs as we do not write to it
class MultiCameraSource {
 public:
  MultiCameraSource(std::vector<std::unique_ptr<ICamera>>& cameras,
                    bool use_all_frames = false);
  [[nodiscard]] auto GetTimestampedFrames() -> std::vector<timestamped_frame_t>;
  [[nodiscard]] auto GetCVFrames() -> std::vector<cv::Mat>;
  [[nodiscard]] inline auto NumCameras() -> double { return cameras_.size(); }

 private:
  std::vector<std::unique_ptr<ICamera>> cameras_;
  std::vector<double>
      invalid_frame_start_times_;  // time since the frame has not been old
  std::vector<timestamped_frame_t> timestamped_frames_;
  std::vector<std::thread> camera_threads_;
  std::mutex mutex_;
  const bool use_all_frames_;
  bool frames_used_ = true; // only for when use_all_frames_ is true
};

}  // namespace camera
