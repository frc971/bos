#include "realsense_camera.h"
#include <iostream>
#include "opencv2/opencv.hpp"
namespace camera {

RealSenseCamera::RealSenseCamera() : pipe_() {
  pipe_.start();
  std::cout << "Device: " << pipe_.get_active_profile() << std::endl;
}

RealSenseCamera::~RealSenseCamera() {
  pipe_.stop();
}

void RealSenseCamera::getFrame(cv::Mat& mat) {
  for (int i = 0; i < 14; i++) {
    std::cout << "Fetching frame: " << i << " with pipe address: " << &pipe_
              << std::endl;
    rs2::frameset frames = pipe_.wait_for_frames(5000);
  }
  rs2::frameset frames = pipe_.wait_for_frames(5000);
  std::cout << "Made it past frameset creation" << std::endl;
  rs2::video_frame color_frame = frames.get_color_frame();
  rs2::depth_frame depth_frame = frames.get_depth_frame();
  std::cout << "Made it past frame collection" << std::endl;
  if (!color_frame) {
    std::cerr << "Invalid color frame!" << std::endl;
    return;
  }
  cv::Mat frameRGB(cv::Size(color_frame.get_width(), color_frame.get_height()),
                   CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

  // Copy and convert in one go
  cv::cvtColor(frameRGB, mat, cv::COLOR_RGB2BGR);
  mat = mat.clone();  // ensure mat owns its data
}
}  // namespace camera
