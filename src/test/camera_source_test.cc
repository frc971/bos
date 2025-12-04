#include "src/camera/camera_source.h"
#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>
#include <iostream>
#include <memory>
#include <opencv2/core/check.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <thread>
#include "src/camera/camera.h"

class ImageCamera : public camera::ICamera {
 public:
  ImageCamera(cv::Mat image) : image_(image) {}
  void GetFrame(cv::Mat& mat) override { mat = image_.clone(); }

 private:
  cv::Mat image_;
};

int main() {
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
