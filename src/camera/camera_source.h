#pragma once
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <queue>
#include <string>
#include <thread>
#include "src/camera/camera.h"
namespace camera {

class CameraSource {
 public:
  CameraSource(std::string name, std::unique_ptr<ICamera> camera);
  auto Get() -> timestamped_frame_t;
  auto GetFrame() -> cv::Mat;
  [[nodiscard]] auto GetName() const -> std::string { return name_; }

 private:
  std::string name_;
  std::unique_ptr<ICamera> camera_;
  timestamped_frame_t timestamped_frame_;
  size_t length_;
  std::thread thread_;
  std::mutex mutex_;
};

}  // namespace camera
