#pragma once
#include <memory>
#include "src/camera/camera.h"
#include "src/utils/pch.h"

namespace camera {

class CVCamera : public ICamera {
 public:
  CVCamera(const cv::VideoCapture& cap);
  auto GetFrame() -> timestamped_frame_t override;

 private:
  cv::VideoCapture cap_;
  int current_frame_;
};

}  // namespace camera
