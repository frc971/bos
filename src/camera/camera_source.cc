#include "camera_source.h"
#include "src/utils/log.h"

namespace camera {
CameraSource::CameraSource(std::string name, std::unique_ptr<ICamera> camera,
                           bool simulation)
    : name_(std::move(name)),
      camera_(std::move(camera)),
      simulation_(simulation) {
  timestamped_frame_ = camera_->GetFrame();
  thread_ = std::jthread([this](const std::stop_token& stop_token) {
    while (!stop_token.stop_requested()) {
      if (camera_->IsDone()) {
        exit(0);
        return;
      }
      timestamped_frame_t timestamped_frame;
      timestamped_frame = camera_->GetFrame();
      mutex_.lock();
      timestamped_frame_ = timestamped_frame;
      mutex_.unlock();
    }
  });
}

auto CameraSource::Get() -> timestamped_frame_t {
  mutex_.lock();
  cv::Mat frame;
  timestamped_frame_t timestamped_frame{
      .frame = timestamped_frame_.frame.clone(),
      .timestamp = timestamped_frame_.timestamp,
      .invalid = timestamped_frame_.invalid};
  mutex_.unlock();
  if (!simulation_ && !camera_->IsDone()) {
    auto current_time = frc::Timer::GetFPGATimestamp().to<double>();
    if (current_time - timestamped_frame.timestamp > 5.0) {
      LOG(INFO) << "Restarting camera because of old timestamp";
      mutex_.lock();
      timestamped_frame_.timestamp =
          current_time;  // 5 second buffer to get frame camera frames again
      camera_->Restart();
      mutex_.unlock();
    }
  }
  return timestamped_frame;
}

auto CameraSource::GetFrame() -> cv::Mat {
  mutex_.lock();
  cv::Mat frame = timestamped_frame_.frame;
  mutex_.unlock();
  return frame;
}

}  // namespace camera
