#include "camera_source.h"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <opencv2/imgcodecs.hpp>

namespace camera {
CameraSource::CameraSource(std::string name, std::unique_ptr<ICamera> camera)
    : name_(std::move(name)), camera_(std::move(camera)) {
  timestamped_frame_ = camera_->GetFrame();
  thread_ = std::thread([this] {
    while (true) {
      timestamped_frame_t timestamped_frame;
      timestamped_frame = camera_->GetFrame();
      mutex_.lock();
      timestamped_frame_ = timestamped_frame;
      mutex_.unlock();
    }
  });
}

auto CameraSource::Get() -> timestamped_frame_t {
  mutex_.lock();
  timestamped_frame_t timestamped_frame = timestamped_frame_;
  mutex_.unlock();
  return timestamped_frame;
}

auto CameraSource::GetFrame() -> cv::Mat {
  mutex_.lock();
  cv::Mat frame = timestamped_frame_.frame;
  mutex_.unlock();
  return frame;
}

}  // namespace camera
