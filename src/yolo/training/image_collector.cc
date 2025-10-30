#include <chrono>
#include <ctime>
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
    rs_camera.getFrame(frame);
    if (i % 10 == 0) {
      cv::imshow("Display", frame);
      cv::waitKey(0);
      cv::destroyAllWindows();
    }
    std::cout << "Writing" << std::endl;
    cv::imwrite("frame_" + std::to_string(localTime->tm_mday) + "_" +
                    std::to_string(localTime->tm_hour) + "_" +
                    std::to_string(localTime->tm_min) + "_" +
                    std::to_string(i) + ".png",
                frame);
    i++;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
