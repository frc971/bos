#pragma once
#include <opencv2/core/mat.hpp>
#include <string>
#include "src/camera/camera.h"

namespace camera {

class DiskCamera : ICamera {
 public:
  DiskCamera(std::string image_folder_path);
  void GetFrame(cv::Mat& frame);

 private:
  std::string image_folder_path_;
  int current_frame_;
};

}  // namespace camera
