#include <chrono>
#include <ctime>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <thread>
#include "src/camera/select_camera.h"

int main() {
  std::time_t now = std::time(nullptr);
  std::tm* localTime = std::localtime(&now);
  int i = 0;
  const std::string img_dir = "/bos/logs/collected_imgs/";
  std::filesystem::create_directories(img_dir);
  const std::string name_start = "frame_" + std::to_string(localTime->tm_mday) +
                                 "_" + std::to_string(localTime->tm_hour) + "_";
  camera::CVCamera camera =
      camera::CVCamera(cv::VideoCapture(camera::SelectCamera()));
  while (true) {
    cv::Mat frame;
    // cv::namedWindow("Captured Frame");
    camera.GetFrame(frame);
    /*cv::imshow("Captured Frame", frame);
    cv::waitKey(0);*/
    cv::imwrite(img_dir + name_start + std::to_string(localTime->tm_min) + "_" +
                    std::to_string(i) + ".bmp",
                frame);
    i++;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}
