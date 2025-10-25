#include <atomic>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <thread>
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/imx296_camera.h"
#include "src/camera/select_camera.h"
#include "src/camera/usb_camera.h"

const int k_port = 4971;

int main() {
  std::cout << "OpenCV version: " << CV_VERSION << std::endl;

  std::cout << "Port number: " << k_port << std::endl;

  camera::CscoreStreamer streamer(
      camera::IMX296Streamer("frame_logger", 4971, 30));

  camera::CVCamera camera((cv::VideoCapture(camera::usb_camera0)));

  cv::Mat frame;

  std::cout << "Camera opened successfully" << std::endl;

  while (true) {

    cv::Mat frame;
    while (true) {
      std::cout << "Getting frame" << std::endl;
      camera.GetFrame(frame);
      streamer.WriteFrame(frame);
      std::cout << "Got frame" << std::endl;
    }
  }
  cv::destroyAllWindows();
  return 0;
}
