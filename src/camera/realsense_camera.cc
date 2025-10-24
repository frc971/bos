#include "realsense_camera.h"
#include <iostream>
namespace camera {

RealSenseCamera::RealSenseCamera() : pipe_(), frames_(nullptr), color_frame_(nullptr), depth_frame_(nullptr) {
  pipe_.start();
  std::cout << "Device: " << pipe_.get_active_profile() << std::endl;
}

RealSenseCamera::~RealSenseCamera() {
  pipe_.stop();
}

void RealSenseCamera::getFrame(cv::Mat& mat) {
  frames_ = pipe_.wait_for_frames();
  color_frame_ = frames_.get_color_frame();
  depth_frame_ = frames_.get_depth_frame();

  mat = cv::Mat(cv::Size(color_frame_.get_width(), color_frame_.get_height()), CV_8UC3, (void*)color_frame_.get_data(), cv::Mat::AUTO_STEP);
}
}  // namespace camera
