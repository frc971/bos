#include "cv_camera.h"
#include "src/utils/log.h"

namespace camera {

CVCamera::CVCamera(const CameraConstant& c)
    : cap_(cv::VideoCapture(c.pipeline)) {
  cap_.set(cv::CAP_PROP_BACKLIGHT, 0);
  cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
  cap_.set(cv::CAP_PROP_EXPOSURE, 8000);
  cap_.set(cv::CAP_PROP_BRIGHTNESS, 20);
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  cap_.set(cv::CAP_PROP_FPS, 60);

  // set_if(cv::CAP_PROP_FRAME_WIDTH, c.frame_width);
  // set_if(cv::CAP_PROP_FRAME_HEIGHT, c.frame_height);

  // auto set_if = [&](int prop, const auto& opt) {
  //   if (opt) {
  //     cap_.set(prop, *opt);
  //   }
  // };
  //
  // set_if(cv::CAP_PROP_BACKLIGHT, c.backlight);
  // set_if(cv::CAP_PROP_FRAME_WIDTH, c.frame_width);
  // set_if(cv::CAP_PROP_FRAME_HEIGHT, c.frame_height);
  // set_if(cv::CAP_PROP_FPS, c.fps);
  // set_if(cv::CAP_PROP_BRIGHTNESS, c.brightness);
  // set_if(cv::CAP_PROP_SHARPNESS, c.sharpness);
  //
  // if (c.exposure) {
  //   cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // V4L2: 1 = manual
  //   cap_.set(cv::CAP_PROP_EXPOSURE, static_cast<double>(*c.exposure));
  // }
}

CVCamera::CVCamera(const std::string& pipeline)
    : cap_(cv::VideoCapture(pipeline)) {}

auto CVCamera::GetFrame() -> timestamped_frame_t {
  timestamped_frame_t timestamped_frame;
  cap_.grab();
  timestamped_frame.timestamp = frc::Timer::GetFPGATimestamp().to<double>();
  cap_.retrieve(timestamped_frame.frame);
  return timestamped_frame;
}

}  // namespace camera
