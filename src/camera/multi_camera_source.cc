#include "multi_camera_source.h"
#include "src/camera/camera.h"
#include "src/utils/log.h"

namespace camera {
MultiCameraSource::MultiCameraSource(
    std::vector<std::unique_ptr<ICamera>>& cameras, bool use_all_frames)
    : cameras_(std::move(cameras)),
      timestamped_frames_(cameras_.size()),
      use_all_frames_(use_all_frames) {
  camera_threads_.reserve(cameras_.size());
  for (size_t i = 0; i < cameras_.size(); i++) {
    camera_threads_.emplace_back([this, i]() -> void {
      while (true) {
        mutex_.lock();
        bool frames_used = frames_used_;
        mutex_.unlock();
        if (!frames_used) {
          std::this_thread::sleep_for(std::chrono::duration<double>(0.002));
          continue;
        }
        timestamped_frame_t timestamped_frame;
        timestamped_frame = cameras_[i]->GetFrame();
        mutex_.lock();
        timestamped_frames_[i] = timestamped_frame;
        frames_used_ = false;
        mutex_.unlock();
      }
    });
  }
}

auto MultiCameraSource::GetTimestampedFrames()
    -> std::vector<timestamped_frame_t> {
  std::vector<timestamped_frame_t> timestamped_frames;
  mutex_.lock();
  timestamped_frames = timestamped_frames_;
  frames_used_ = true;
  mutex_.unlock();
  return timestamped_frames;
}

auto MultiCameraSource::GetCVFrames() -> std::vector<cv::Mat> {
  std::vector<cv::Mat> frames;
  frames.reserve(cameras_.size());
  mutex_.lock();
  for (size_t i = 0; i < cameras_.size(); i++) {
    frames.push_back(timestamped_frames_[i].frame);
  }
  frames_used_ = true;
  mutex_.unlock();
  return frames;
}

}  // namespace camera
