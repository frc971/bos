#include "multi_camera_source.h"
#include "src/camera/camera.h"
#include "src/utils/log.h"

namespace camera {
MultiCameraSource::MultiCameraSource(
    std::vector<std::unique_ptr<ICamera>>& cameras, bool use_all_frames)
    : cameras_(std::move(cameras)), timestamped_frames_(cameras_.size()) {
  camera_threads_.reserve(cameras_.size());
  for (size_t i = 0; i < cameras_.size(); i++) {
    camera_threads_.emplace_back([this, i]() -> void {
      while (true) {
        timestamped_frame_t timestamped_frame;
        timestamped_frame = cameras_[i]->GetFrame();
        if (timestamped_frame.timestamp == last_frame_time_[i]) {
          continue;
        }
        if (mutex_.try_lock()) {  // ok to skip frames
          timestamped_frames_[i] = timestamped_frame;
          last_frame_time_[i] = timestamped_frame.timestamp;
          mutex_.unlock();
        }
      }
    });
  }
}

auto MultiCameraSource::GetTimestampedFrames()
    -> std::vector<timestamped_frame_t> {
  std::vector<timestamped_frame_t> timestamped_frames;
  mutex_.lock();
  for (size_t i = 0; i < timestamped_frames_.size(); i++) {
    timestamped_frames.push_back(timestamped_frames_[i]);
    timestamped_frames_[i].frame.copyTo(timestamped_frames[i].frame);
  }
  mutex_.unlock();
  return timestamped_frames;
}

auto MultiCameraSource::GetCVFrames() -> std::vector<cv::Mat> {
  std::vector<cv::Mat> frames(cameras_.size());
  frames.reserve(cameras_.size());
  mutex_.lock();
  for (size_t i = 0; i < cameras_.size(); i++) {
    timestamped_frames_[i].frame.copyTo(frames[i]);
  }
  mutex_.unlock();
  return frames;
}

}  // namespace camera
