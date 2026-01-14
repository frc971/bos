#include <atomic>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <thread>
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"

auto main() -> int {
  camera::CscoreStreamer streamera("frame_shower", 4971, 30, 1080, 1080);
  camera::CscoreStreamer streamerb("frame_shower", 4972, 30, 1080, 1080);
  cv::VideoCapture a(0);
  cv::VideoCapture b(1);

  // std::unique_ptr<camera::ICamera> camera =
  //     camera::GetCameraStream(camera::SelectCameraConfig());

  cv::Mat framea;
  cv::Mat frameb;

  std::cout << "Camera opened successfully" << std::endl;

  while (true) {
    std::cout << "Getting frame" << std::endl;
    // a.read(framea);
    // streamera.WriteFrame(framea);
    b.read(frameb);
    streamerb.WriteFrame(frameb);
    std::cout << framea.size << std::endl;
    std::cout << frameb.size << std::endl;
  }
  cv::destroyAllWindows();
  return 0;
}
