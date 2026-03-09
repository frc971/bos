#include <wpilibc/frc/Timer.h>
#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <thread>
#include "src/camera/select_camera.h"
#include "src/utils/pch.h"

auto main() -> int {
  std::string img_dir = "/bos/logs/collected_imgs/";
  std::filesystem::create_directories(img_dir);
  camera::camera_constants_t camera_constants =
      camera::GetCameraConstants("/bos/constants/camera_constants.json");
  camera::camera_constant_t camera_constant =
      camera::SelectCameraConfig(camera_constants);
  std::unique_ptr<camera::ICamera> camera =
      std::make_unique<camera::CVCamera>(camera_constant);

  while (true) {
    cv::Mat frame;
    frame = camera->GetFrame().frame;
    cv::imwrite(img_dir +
                    std::to_string(frc::Timer::GetFPGATimestamp().value()) +
                    ".png",
                frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}
