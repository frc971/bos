#include "src/camera/camera_source.h"
#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>
#include <iostream>
#include <memory>
#include <opencv2/core/check.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <thread>
#include <utility>
#include "src/camera/camera.h"

class ImageCamera : public camera::ICamera {
 public:
  ImageCamera(cv::Mat image) : image_(std::move(image)) {}
  auto GetFrame() -> camera::timestamped_frame_t override {
    return {image_.clone(), 0};
  }

 private:
  cv::Mat image_;
};

auto main() -> int {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StopClient();
  inst.StopLocal();
  inst.StartClient4("orin_localization");
  inst.SetServerTeam(971);
  frc::DataLogManager::Start();

  std::cout << "Started networktables!" << std::endl;
  std::unique_ptr<camera::ICamera> camera =
      std::make_unique<ImageCamera>(cv::imread("calibration_board.png"));
  camera::CameraSource source("test", std::move(camera));
  while (true) {
    auto frame = source.Get();
    std::cout << frame.timestamp << std::endl;
  }
}
