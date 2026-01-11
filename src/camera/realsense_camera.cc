#include "realsense_camera.h"
#include <wpilibc/frc/Timer.h>
#include <iostream>
#include <thread>
#include "opencv2/opencv.hpp"
#include "src/utils/log.h"
namespace camera {

RealSenseCamera::RealSenseCamera()
    : pipe_(), align_to_color_(RS2_STREAM_COLOR) {  // NOLINT

  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  std::cout << "cfg created" << std::endl;
  pipe_.start(cfg);
  std::cout << "pipe started" << std::endl;
  rs2::frameset test_frames = pipe_.wait_for_frames(5000);
  std::cout << "testframes acquired" << std::endl;
  auto profile = pipe_.get_active_profile()
                     .get_stream(RS2_STREAM_COLOR)
                     .as<rs2::video_stream_profile>();

  rs2_intrinsics intr = profile.get_intrinsics();

  float fx = intr.fx;
  float fy = intr.fy;
  float cx = intr.ppx;
  float cy = intr.ppy;

  std::cout << "Fx: " << fx << "\tfy: " << fy << "\tcx: " << cx
            << "\tcy: " << cy << std::endl;
}

RealSenseCamera::~RealSenseCamera() {
  pipe_.stop();
}

auto RealSenseCamera::GetFrame() -> timestamped_frame_t {
  timestamped_frame_t timestamped_frame;
  if (!warmed_up_) {
    for (int i = 0; i < 14; i++) {
      rs2::frameset frames = pipe_.wait_for_frames(5000);
    }
    warmed_up_ = true;
  }
  rs2::frameset frames = pipe_.wait_for_frames(5000);
  rs2::video_frame color_frame = frames.get_color_frame();
  if (!color_frame) {
    LOG(ERROR) << "Invalid color frame!";
    exit(0);
  }

  cv::Mat frameRGB(cv::Size(color_frame.get_width(), color_frame.get_height()),
                   CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
  cv::Mat frameBGR;

  cv::cvtColor(frameRGB, frameBGR, cv::COLOR_RGB2BGR);
  return timestamped_frame_t{
      .frame = frameBGR,
      .timestamp = frc::Timer::GetFPGATimestamp().to<double>()};
}

void RealSenseCamera::GetFrame(cv::Mat& color_mat, cv::Mat& depth_mat) {
  if (!warmed_up_) {
    for (int i = 0; i < 14; i++) {
      rs2::frameset frames = pipe_.wait_for_frames(5000);
    }
    warmed_up_ = true;
  }
  rs2::frameset frames = pipe_.wait_for_frames(5000);
  frames = align_to_color_.process(frames);
  rs2::video_frame color_frame = frames.get_color_frame();
  rs2::depth_frame depth_frame = frames.get_depth_frame();
  if (!color_frame) {
    std::cerr << "Invalid color frame!" << std::endl;
    return;
  }
  if (!depth_frame) {
    std::cerr << "Invalid color frame!" << std::endl;
    return;
  }

  cv::Mat frameRGB(cv::Size(color_frame.get_width(), color_frame.get_height()),
                   CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

  cv::cvtColor(frameRGB, color_mat, cv::COLOR_RGB2BGR);
  cv::Mat z16 =
      cv::Mat(cv::Size(depth_frame.get_width(), depth_frame.get_height()),
              CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
  z16.convertTo(depth_mat, CV_32FC1, depth_frame.get_units());
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
  for (const auto& sensor : sensors) {
    auto profiles = sensor.get_stream_profiles();
    for (const auto& profile : profiles) {
      if (auto vp = profile.as<rs2::video_stream_profile>()) {
        std::cout << "Stream: " << vp.stream_type() << " " << vp.width() << "x"
                  << vp.height() << " @ " << vp.fps() << "fps "
                  << " Format: " << vp.format() << std::endl;
      }
    }
  }
}
}  // namespace camera
