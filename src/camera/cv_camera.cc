#include "cv_camera.h"

namespace camera {

CVCamera::CVCamera(const cv::VideoCapture& cap) : cap_(cap) {
  // cap_.set(cv::CAP_PROP_BACKLIGHT, 0);
  // cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
  // cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  // cap_.set(cv::CAP_PROP_FPS, 60);
  // cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // V4L2: 1 = manual
  // cap_.set(cv::CAP_PROP_EXPOSURE, 500);
  // cap_.set(cv::CAP_PROP_BRIGHTNESS, 0.5);
  // cap_.set(cv::CAP_PROP_SHARPNESS, 100);
}

auto CVCamera::GetFrame() -> timestamped_frame_t {
  timestamped_frame_t timestamped_frame;
  cap_.grab();
  timestamped_frame.timestamp = frc::Timer::GetFPGATimestamp().to<double>();
  cap_.retrieve(timestamped_frame.frame);
  return timestamped_frame;
}

}  // namespace camera
