#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "src/camera/cscore_streamer.h"
#include "src/camera/imx296_camera.h"
#include "src/camera/streamer.h"

int main(int argc, char* argv[]) {
  std::cout << "OpenCV version: " << CV_VERSION << std::endl;

  std::cout << "What is the id of the camera we are using?\n";
  int camera_id;
  std::cin >> camera_id;

  camera::IMX296Camera camera(camera::IMX296Template(camera_id, 30));

  camera::CscoreStreamer streamer(
      camera::IMX296Streamer("focus_calibrate", 4971, 30));

  cv::Mat frame, gray, laplace;
  while (true) {
    camera.GetFrame(frame);
    streamer.WriteFrame(frame);

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Laplacian(gray, laplace, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplace, mean, stddev, cv::Mat());

    std::cout << "Focus(bigger is better): " << stddev * stddev << "\n";
  }
}
