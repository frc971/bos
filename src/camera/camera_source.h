#pragma once
#include "src/camera/camera.h"
#include "src/utils/pch.h"
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
  std::thread thread_;
  std::mutex mutex_;
};

}  // namespace camera
