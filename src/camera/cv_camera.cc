#include "cv_camera.h"

#include <filesystem>
#include <utility>
#include "src/camera/camera_constants.h"
#include "src/camera/write_frame.h"
#include "src/utils/log.h"
#include "src/utils/timer.h"
#include "src/utils/wpilog_time.h"

namespace fs = std::filesystem;

namespace camera {

auto FileSystemAlmostFull() {
  fs::space_info info = fs::space("/");
  return static_cast<float>(info.free) / static_cast<float>(info.capacity) <
         0.2;
}

CVCamera::CVCamera(const CameraConstant& c, std::optional<std::string> log_path)
    : camera_constant_(c),
      cap_(cv::VideoCapture(c.pipeline.value())),
      pipeline_(c.pipeline.value()),
      log_path_(std::move(log_path)) {
  cap_.release();
  cap_ = cv::VideoCapture(pipeline_);
  if (FileSystemAlmostFull()) {
    log_path_ = std::nullopt;
    LOG(WARNING) << "Filesystem almost full! Not logging any frames";
  }
  PCHECK(c.pipeline.has_value()) << "Pipeline needs value";

  LOG(INFO) << c.pipeline.value();

  backup_image_ = cv::imread("/bos/constants/dont_worry_about_it.jpg");
  LOG(INFO) << "Backup image empty: " << backup_image_.empty();
  if (c.frame_height.has_value() && c.frame_width.has_value()) {
    cv::resize(backup_image_, backup_image_,
               cv::Size(c.frame_width.value(), c.frame_height.value()));
  }

  auto set_if = [&](int prop, const auto& opt) {
    if (opt) {
      cap_.set(prop, *opt);
    }
  };

  set_if(cv::CAP_PROP_BACKLIGHT, c.backlight);
  set_if(cv::CAP_PROP_FRAME_WIDTH, c.frame_width);
  set_if(cv::CAP_PROP_FRAME_HEIGHT, c.frame_height);
  set_if(cv::CAP_PROP_FPS, c.fps);
  set_if(cv::CAP_PROP_BRIGHTNESS, c.brightness);
  set_if(cv::CAP_PROP_SHARPNESS, c.sharpness);

  if (c.exposure) {
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // V4L2: 1 = manual
    cap_.set(cv::CAP_PROP_EXPOSURE, static_cast<double>(*c.exposure));
  } else {
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);
  }
  if (log_path_.has_value()) {
    LOG(INFO) << "Creating log folder " << log_path_.value() << ": "
              << std::filesystem::create_directory(log_path_.value());
  }
}

auto CVCamera::GetFrame() -> timestamped_frame_t {
  timestamped_frame_t timestamped_frame;
  cv::Mat raw_image;
  if (!cap_.grab()) {
    Restart();
    LOG(WARNING) << "Restarting camera";
  }
  timestamped_frame.timestamp = utils::GetLogTimestampSeconds();
  cap_.retrieve(raw_image);

  raw_image.copyTo(timestamped_frame.frame);

  if (timestamped_frame.frame.empty()) {
    timestamped_frame.frame = backup_image_;
  }
  if (timestamped_frame.frame.channels() == 4) {
    cv::cvtColor(timestamped_frame.frame, timestamped_frame.frame,
                 cv::COLOR_BGRA2BGR);
  }
  if (log_path_.has_value()) {
    WriteFrame(log_path_.value(), timestamped_frame);
  }
  return timestamped_frame;
}

auto CVCamera::Restart() -> void {
  cap_.release();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  LOG(INFO) << "Restarting camera with pipeline: " << pipeline_;
  cap_ = cv::VideoCapture(pipeline_);
  std::this_thread::sleep_for(std::chrono::seconds(3));
}

auto CVCamera::GetCameraConstant() const -> camera_constant_t {
  return camera_constant_;
}
}  // namespace camera
