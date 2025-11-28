#include "camera_source.h"
#include <wpilibc/frc/Timer.h>
#include <algorithm>
#include <cstdlib>
#include <memory>

namespace camera {

CameraSource::CameraSource(int width, int height, int channels, int image_type,
                           std::unique_ptr<ICamera> camera, int length)
    : width_(width),
      height_(height),
      channels_(channels),
      image_type_(image_type),
      image_size_(width * height * channels * sizeof(uint8_t)),
      camera_(std::move(camera)),
      length_(length),
      head_(-1) {
  buffer_ = static_cast<uint8_t*>(malloc(image_size_ * length));
  thread_ = std::thread([this] {
    while (true) {
      cv::Mat mat(height_, width_, image_type_,
                  buffer_ + image_size_ * (head_ + 1));
      camera_->GetFrame(mat);
      timestamp_[head_ + 1] = frc::Timer::GetFPGATimestamp().to<double>();

      head_ += 1;
    }
  });
}

frame_t CameraSource::Get() {
  cv::Mat mat(height_, width_, image_type_, buffer_ + image_size_ * head_);
  double timestamp = timestamp_[head_];
  return frame_t{.mat = mat, .timestamp = timestamp};
}

}  // namespace camera
