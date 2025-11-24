#include "camera_source.h"
#include <memory>

namespace camera {
CameraSource::CameraSource(std::unique_ptr<ICamera> camera)
    : camera_(std::move(camera)), buffer_(250) {
  thread_ = std::thread([this]() {
    while (true) {
      buffer_.push_back(camera->GetFrame(cv::Mat & mat))
    }
  });
};
}  // namespace camera
