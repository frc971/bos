#include <iostream>
#include <opencv2/videoio.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"

auto main(int argc, char* argv[]) -> int {
  camera::Camera config = camera::SelectCameraConfig();
  std::unique_ptr<camera::ICamera> camera = camera::GetCameraStream(config);

  camera::CscoreStreamer streamer("focus_calibrate", 4971, 30, 1080, 1080);

  cv::Mat frame, gray, laplace;
  while (true) {
    camera->GetFrame(frame);
    streamer.WriteFrame(frame);

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Laplacian(gray, laplace, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplace, mean, stddev, cv::Mat());

    std::cout << "Focus(bigger is better): " << stddev * stddev << "\n";
  }
}
