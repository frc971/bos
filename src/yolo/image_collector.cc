#include <wpilibc/frc/Timer.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <thread>
#include "src/camera/select_camera.h"
#include <chrono>
#include <string>

int main() {
  std::string img_dir = "/bos/logs/collected_imgs/";
  std::filesystem::create_directories(img_dir);
  camera::CVCamera camera =
      camera::CVCamera(cv::VideoCapture(camera::SelectCamera()));
  while (true) {
    cv::Mat frame;
    camera.GetFrame(frame);
    cv::imwrite(img_dir + frc::Timer::GetFPGATimestamp()) + ".bmp",
                frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}
