#include "realsense_camera.h"
#include <iostream>
#include <thread>
#include "opencv2/opencv.hpp"
namespace camera {

RealSenseCamera::RealSenseCamera() : pipe_() {
  std::cout << "RealSenseCamera constructor starting..." << std::endl;
  std::cout << "pipe_ address: " << &pipe_ << std::endl;

  try {
    std::cout << "About to start pipeline..." << std::endl;
    auto profile = pipe_.start();
    std::cout << "Pipeline started successfully!" << std::endl;

    // Verify it works immediately
    std::cout << "Testing initial frame fetch..." << std::endl;
    rs2::frameset test_frames = pipe_.wait_for_frames(5000);
    std::cout << "Initial frame fetch successful!" << std::endl;

  } catch (const rs2::error& e) {
    std::cerr << "RealSense error in constructor: " << e.what() << std::endl;
    throw;
  } catch (const std::exception& e) {
    std::cerr << "Standard exception in constructor: " << e.what() << std::endl;
    throw;
  }

  std::cout << "RealSenseCamera constructor completed successfully"
            << std::endl;
}

RealSenseCamera::~RealSenseCamera() {
  std::cout << "RealSenseCamera destructor called" << std::endl;
  try {
    pipe_.stop();
    std::cout << "Pipeline stopped successfully" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error stopping pipeline: " << e.what() << std::endl;
  }
}

void RealSenseCamera::getFrame(cv::Mat& mat) {
  std::cout << "this pointer: " << this << std::endl;
  std::cout << "pipe_ address: " << &pipe_ << std::endl;
  for (int i = 0; i < 14; i++) {
    try {
      std::cout << "Fetching frame: " << i << " with pipe address: " << &pipe_
                << std::endl;
      rs2::frameset frames = pipe_.wait_for_frames(5000);
    } catch (const rs2::error& e) {
      std::cerr << "RealSense error: " << e.what() << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "Standard exception: " << e.what() << std::endl;
    }
  }
  rs2::frameset frames = pipe_.wait_for_frames(5000);
  rs2::video_frame color_frame = frames.get_color_frame();
  if (!color_frame) {
    std::cerr << "Invalid color frame!" << std::endl;
    return;
  }

  cv::Mat frameRGB(cv::Size(color_frame.get_width(), color_frame.get_height()),
                   CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

  std::cout << "Made it past mat creation" << std::endl;

  cv::cvtColor(frameRGB, mat, cv::COLOR_RGB2BGR);
  std::cout << "Made it past mat coloring" << std::endl;
}
}  // namespace camera
