#include "realsense_camera.h"
#include <iostream>
#include <thread>
#include "opencv2/opencv.hpp"
namespace camera {

RealSenseCamera::RealSenseCamera() : pipe_() {
  /*showDevices();
  exit(0);*/
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 15);
  std::cout << "cfg created" << std::endl;
  pipe_.start(cfg);
  std::cout << "pipe started" << std::endl;
  rs2::frameset test_frames = pipe_.wait_for_frames(5000);
  std::cout << "testframes acquired" << std::endl;
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

void RealSenseCamera::showDevices() {
  rs2::context ctx;
  auto devices = ctx.query_devices();
  if (devices.size() == 0) {
    std::cout << "No RealSense device connected!" << std::endl;
    return;
  }
  auto dev = devices[0];
  std::cout << "Device: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;

  // Query available stream profiles
  auto sensors = dev.query_sensors();
  for (auto sensor : sensors) {
    auto profiles = sensor.get_stream_profiles();
    for (auto profile : profiles) {
      if (auto vp = profile.as<rs2::video_stream_profile>()) {
        std::cout << "Stream: " << vp.stream_type() << " " << vp.width() << "x"
                  << vp.height() << " @ " << vp.fps() << "fps "
                  << " Format: " << vp.format() << std::endl;
      }
    }
  }
}
}  // namespace camera
