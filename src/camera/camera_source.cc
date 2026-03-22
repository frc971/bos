#include "camera_source.h"
#include "src/utils/log.h"

namespace camera {
CameraSource::CameraSource(std::string name, std::unique_ptr<ICamera> camera,
                           bool simulation, bool use_fetcher_thread)
    : name_(std::move(name)),
      camera_(std::move(camera)),
      simulation_(simulation),
      use_fetcher_thread_(use_fetcher_thread) {
  if (use_fetcher_thread) {
    timestamped_frame_ = camera_->GetFrame();
    thread_ = std::thread([this] {
      while (true) {
        timestamped_frame_t timestamped_frame;
        timestamped_frame = camera_->GetFrame();
        mutex_.lock();
        timestamped_frame_ = timestamped_frame;
        mutex_.unlock();
      }
    });
  }
}

auto CameraSource::Get() -> timestamped_frame_t {
  if (use_fetcher_thread_) {
    mutex_.lock();
    timestamped_frame_t timestamped_frame = timestamped_frame_;
    mutex_.unlock();
    if (!simulation_) {
      auto current_time = frc::Timer::GetFPGATimestamp().to<double>();
      if (current_time - timestamped_frame.timestamp > 5.0) {
        LOG(INFO) << "Restarting camera because of old timestamp";
        timestamped_frame_.timestamp = current_time;
        mutex_.lock();
        camera_->Restart();
        mutex_.unlock();
      }
    }
    return timestamped_frame;
  } else {
    return camera_->GetFrame();
  }
}

auto CameraSource::GetFrame() -> cv::Mat {
  if (use_fetcher_thread_) {
    mutex_.lock();
    cv::Mat frame = timestamped_frame_.frame;
    mutex_.unlock();
    return frame;
  } else {
    return camera_->GetFrame().frame;
  }
}

}  // namespace camera
