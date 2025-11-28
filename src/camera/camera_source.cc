#include "camera_source.h"
#include <wpilibc/frc/Timer.h>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <opencv2/imgcodecs.hpp>

namespace camera {
CameraSource::CameraSource(std::string name, std::unique_ptr<ICamera> camera)
    : name_(name), camera_(std::move(camera)) {
  cv::Mat frame;
  camera_->GetFrame(frame);
  frame_ = frame;
  timestamp_ = frc::Timer::GetFPGATimestamp().to<double>();
  thread_ = std::thread([this] {
    while (true) {
      cv::Mat frame;
      camera_->GetFrame(frame);
      const double timestamp = frc::Timer::GetFPGATimestamp().to<double>();
      mutex_.lock();
      frame_ = frame;
      timestamp_ = timestamp;
      mutex_.unlock();
    }
  });
}

timestamped_frame_t CameraSource::Get() {
  mutex_.lock();
  cv::Mat frame = frame_;
  double timestamp = timestamp_;
  mutex_.unlock();
  return timestamped_frame_t{.frame = frame, .timestamp = timestamp};
}

}  // namespace camera
