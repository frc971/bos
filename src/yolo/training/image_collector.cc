#include <chrono>
#include <ctime>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <thread>
#include "src/camera/realsense_camera.h"

int main() {
  std::time_t now = std::time(nullptr);
  std::tm* localTime = std::localtime(&now);
  camera::RealSenseCamera rs_camera;
  int i = 0;
  while (true) {
    cv::Mat frame;
    // cv::namedWindow("Captured Frame");
    rs_camera.getFrame(frame);
    /*cv::imshow("Captured Frame", frame);
    cv::waitKey(0);*/
    std::cout << "Writing" << std::endl;
    cv::imwrite("/home/nvidia/Documents/collected_imgs/frame_" +
                    std::to_string(localTime->tm_mday) + "_" +
                    std::to_string(localTime->tm_hour) + "_" +
                    std::to_string(localTime->tm_min) + "_" +
                    std::to_string(i) + ".bmp",
                frame);
    i++;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}
