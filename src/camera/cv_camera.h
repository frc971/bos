#pragma once
#include <memory>
#include "camera_constants.h"
#include "src/camera/camera.h"
#include "src/utils/pch.h"

namespace camera {

class CVCamera : public ICamera {
 public:
  CVCamera(const CameraConstant& camera_constants);
  CVCamera(const std::string& pipeline);
  auto GetFrame() -> timestamped_frame_t override;

 private:
  cv::VideoCapture cap_;
};

}  // namespace camera
