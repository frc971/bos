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

int main() {
  camera::CscoreStreamer streamer(
      camera::IMX296Streamer("frame_shower", 4971, 30));

  camera::Camera camera = camera::SelectCamera();
  camera::CVCamera cap(
      cv::VideoCapture(camera::camera_constants[camera].pipeline));

  cv::Mat frame;

  std::cout << "Camera opened successfully" << std::endl;

  while (true) {

    cv::Mat frame;
    while (true) {
      std::cout << "Getting frame" << std::endl;
      cap.GetFrame(frame);
      streamer.WriteFrame(frame);
      std::cout << frame.size << std::endl;
      std::cout << "Got frame" << std::endl;
    }
  }
  cv::destroyAllWindows();
  return 0;
}
