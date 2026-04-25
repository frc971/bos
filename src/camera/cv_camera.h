#pragma once
#include <memory>
#include "camera_constants.h"
#include "src/camera/camera.h"
#include "src/utils/pch.h"

namespace camera {

// Wrap opencv's camera into the ICamera interface
class CVCamera : public ICamera {
 public:
  CVCamera(const CameraConstant& camera_constant,
           std::optional<std::string> log_path = std::nullopt);
  auto GetFrame() -> timestamped_frame_t override;
  auto Restart() -> void override;
  [[nodiscard]] auto GetCameraConstant() const -> camera_constant_t override;

 private:
  camera_constant_t camera_constant_;
  cv::VideoCapture cap_;
  std::string pipeline_;
  std::optional<std::string> log_path_;
  cv::Mat backup_image_;
  int i_ = 0;
};

}  // namespace camera
