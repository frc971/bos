#include "realsense_camera.h"
#include <iostream>
#include "opencv2/opencv.hpp"
namespace camera {

RealSenseCamera::RealSenseCamera()
    : pipe_(), frames_(nullptr), color_frame_(nullptr), depth_frame_(nullptr) {
  pipe_.start();
  std::cout << "Device: " << pipe_.get_active_profile() << std::endl;
}

RealSenseCamera::~RealSenseCamera() {
  pipe_.stop();
}

void RealSenseCamera::getFrame(cv::Mat& mat) {
  for (int i = 0; i < 7; i++) {
    std::cout << "Fetching frame " << i << std::endl;
    frames_ = pipe_.wait_for_frames();
    color_frame_ = frames_.get_color_frame();
    depth_frame_ = frames_.get_depth_frame();
  }

  if (!color_frame_) {
    std::cerr << "Invalid color frame!" << std::endl;
    return;
  }
  cv::Mat frameRGB(
      cv::Size(color_frame_.get_width(), color_frame_.get_height()), CV_8UC3,
      (void*)color_frame_.get_data(), cv::Mat::AUTO_STEP);

  // Copy and convert in one go
  cv::cvtColor(frameRGB, mat, cv::COLOR_RGB2BGR);
  mat = mat.clone();  // ensure mat owns its data
}
}  // namespace camera
