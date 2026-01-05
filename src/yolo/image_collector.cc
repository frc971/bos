#include <wpilibc/frc/Timer.h>
#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include "src/camera/select_camera.h"

auto main() -> int {
  std::string img_dir = "/bos/logs/collected_imgs/";
  std::filesystem::create_directories(img_dir);
  camera::Camera config = camera::SelectCameraConfig();
  std::unique_ptr<camera::ICamera> camera = camera::GetCameraStream(config);
  while (true) {
    cv::Mat frame;
    camera->GetFrame(frame);
    cv::imwrite(img_dir +
                    std::to_string(frc::Timer::GetFPGATimestamp().value()) +
                    ".png",
                frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}
