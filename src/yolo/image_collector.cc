#include <wpilibc/frc/Timer.h>
#include <chrono>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include "src/camera/select_camera.h"

int main() {
  std::string img_dir = "/bos/logs/collected_imgs/";
  std::filesystem::create_directories(img_dir);
  camera::CVCamera camera =
      camera::CVCamera(cv::VideoCapture(camera::SelectCamera()));
  while (true) {
    cv::Mat frame;
    camera.GetFrame(frame);
    cv::imwrite(img_dir +
                    std::to_string(frc::Timer::GetFPGATimestamp().value()) +
                    ".png",
                frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}
