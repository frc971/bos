#pragma once
#include <opencv2/core/mat.hpp>
#include <string>
#include "src/camera/camera.h"

namespace camera {

class UsbCamera : Camera {
 public:
  UsbCamera(std::string pipeline);
  void GetFrame(cv::Mat& frame);

 private:
  std::string pipeline_;
  cv::VideoCapture cap_;
};

}  // namespace camera
