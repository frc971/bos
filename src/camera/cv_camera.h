#pragma once
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include "src/camera/camera.h"

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
