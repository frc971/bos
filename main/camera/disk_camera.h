#pragma once
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <string>

namespace camera {

class DiskCamera {
 public:
  DiskCamera(std::string image_folder_path);
  void getFrame(cv::Mat& frame);

 private:
  std::string image_folder_path_;
  int current_frame_;
};

}  // namespace camera
