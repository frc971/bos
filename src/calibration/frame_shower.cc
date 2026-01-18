#include <atomic>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <thread>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"

ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt, "");  //NOLINT

auto main(int argc, char* argv[]) -> int {
  absl::ParseCommandLine(argc, argv);

  camera::Camera config =
      camera::SelectCameraConfig(absl::GetFlag(FLAGS_camera_name));
  std::unique_ptr<camera::ICamera> camera = camera::GetCameraStream(config);

  camera::CscoreStreamer streamer("frame_shower", 4971, 30, 1080, 1080);

  std::cout << "Camera opened successfully" << std::endl;

  while (true) {
    cv::Mat frame = camera->GetFrame().frame;
    cv::resize(frame, frame, cv::Size(1080, 1080));
    cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
    streamer.WriteFrame(frame);
  }
  cv::destroyAllWindows();
  return 0;
}
