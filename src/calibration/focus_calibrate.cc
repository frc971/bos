#include "opencv2/imgproc/imgproc.hpp"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"

auto main(int argc, char* argv[]) -> int {
  camera::camera_constant_t camera_constant =
      camera::SelectCameraConfig(camera::GetCameraConstants());
  std::unique_ptr<camera::ICamera> camera =
      std::make_unique<camera::CVCamera>(camera_constant);

  camera::CscoreStreamer streamer("focus_calibrate", 5801, 30,
                                  camera->GetFrame().frame);

  cv::Mat frame, gray, laplace;
  while (true) {
    frame = camera->GetFrame().frame;
    streamer.WriteFrame(frame);

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Laplacian(gray, laplace, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplace, mean, stddev, cv::Mat());

    std::cout << "Focus(bigger is better): " << stddev * stddev << "\n";
  }
}
