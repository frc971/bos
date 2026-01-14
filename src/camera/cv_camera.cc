#include "cv_camera.h"
#include <wpilibc/frc/Timer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

namespace camera {

CVCamera::CVCamera(const cv::VideoCapture& cap) : cap_(cap) {}

auto CVCamera::GetFrame() -> timestamped_frame_t {
  timestamped_frame_t timestamped_frame;
  cap_.grab();
  timestamped_frame.timestamp = frc::Timer::GetFPGATimestamp().to<double>();
  cap_.retrieve(timestamped_frame.frame);
  // TODO remove
  cv::rotate(timestamped_frame.frame, timestamped_frame.frame, cv::ROTATE_180);
  return timestamped_frame;
}

}  // namespace camera
