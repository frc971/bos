#include "realsense_camera.h"
#include <iostream>
#include <thread>
#include "opencv2/opencv.hpp"
namespace camera {

RealSenseCamera::RealSenseCamera() : pipe_() {
  pipe_.start();
  rs2::frameset test_frames = pipe_.wait_for_frames(5000);
}

RealSenseCamera::~RealSenseCamera() {
  pipe_.stop();
}

void RealSenseCamera::getFrame(cv::Mat& mat) {
  for (int i = 0; i < 14; i++) {
    rs2::frameset frames = pipe_.wait_for_frames(5000);
  }
  rs2::frameset frames = pipe_.wait_for_frames(5000);
  rs2::video_frame color_frame = frames.get_color_frame();
  if (!color_frame) {
    std::cerr << "Invalid color frame!" << std::endl;
    return;
  }

  cv::Mat frameRGB(cv::Size(color_frame.get_width(), color_frame.get_height()),
                   CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

  cv::cvtColor(frameRGB, mat, cv::COLOR_RGB2BGR);
}
}  // namespace camera
