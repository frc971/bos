#pragma once
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include "src/camera/camera.h"

namespace camera {

class CVCamera : Camera {
 public:
  CVCamera(std::unique_ptr<cv::VideoCapture> cap);
  void GetFrame(cv::Mat& frame) override;

 private:
  std::unique_ptr<cv::VideoCapture> cap_;
  int current_frame_;
};

}  // namespace camera
