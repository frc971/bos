#include <memory>
#include "opencv2/imgproc/imgproc.hpp"
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"

auto main(int argc, char* argv[]) -> int {
  std::unique_ptr<camera::ICamera> camera =
      camera::SelectCameraConfig(camera::GetCameraConstants());

  camera::timestamped_frame_t timestamped_frame;
  camera->GetFrame(&timestamped_frame);
  camera::CscoreStreamer streamer("focus_calibrate", 5801, 30,
                                  timestamped_frame.frame);

  cv::Mat gray, laplace;
  while (true) {
    camera->GetFrame(&timestamped_frame);
    streamer.WriteFrame(timestamped_frame.frame);

    cv::cvtColor(timestamped_frame.frame, gray, cv::COLOR_BGR2GRAY);
    cv::Laplacian(gray, laplace, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplace, mean, stddev, cv::Mat());

    std::cout << "Focus(bigger is better): " << stddev * stddev << "\n";
  }
}
