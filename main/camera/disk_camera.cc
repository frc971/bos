#include "disk_camera.h"
#include <opencv2/opencv.hpp>

namespace camera {

DiskCamera::DiskCamera(std::string image_folder_path)
    : image_folder_path_(image_folder_path), current_frame_(0) {}

void DiskCamera::getFrame(cv::Mat& frame) {

  std::ostringstream filename;
  filename << image_folder_path_ << "/" << std::setfill('0') << std::setw(4)
           << current_frame_ << ".jpg";
  frame = cv::imread(filename.str());
}

}  // namespace camera
