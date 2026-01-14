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

  std::unique_ptr<camera::ICamera> camera =
      camera::GetCameraStream(camera::SelectCameraConfig());
  camera::CscoreStreamer streamer("frame_shower", 4971, 30, 1080, 1080);

  std::cout << "Camera opened successfully" << std::endl;

  while (true) {
    cv::Mat frame = camera->GetFrame().frame;
    streamer.WriteFrame(frame);
  }
  cv::destroyAllWindows();
  return 0;
}
