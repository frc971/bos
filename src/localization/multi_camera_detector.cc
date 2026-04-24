#include "src/localization/multi_camera_detector.h"
#include "absl/status/status.h"
#include "src/camera/camera.h"
#include "src/camera/cv_camera.h"
#include "src/camera/uvc_camera.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/utils/camera_utils.h"
#include "src/utils/log.h"

namespace localization {
MultiCameraDetector::MultiCameraDetector(
    std::vector<camera::camera_constant_t> camera_constants,
    std::optional<std::vector<std::filesystem::path>> image_paths)
    : camera_constants_(std::move(camera_constants)),
      last_write_times_(camera_constants_.size()),
      timestamped_frames_(camera_constants_.size()),
      tag_detections_(camera_constants_.size()) {
  std::string log_path = frc::DataLogManager::GetLogDir();
  cameras_.reserve(camera_constants_.size());
  camera_threads_.reserve(camera_constants_.size());
  streamers_.reserve(camera_constants_.size());
  for (size_t i = 0; i < camera_constants_.size(); i++) {
    streamers_.emplace_back(camera_constants_[i].name, 5801 + i, 30, 1080,
                            1080);  // TODO move constants to steamer class
    const std::string camera_log_dest =
        fmt::format("{}/{}", log_path, camera_constants_[i].name);
    if (image_paths.has_value()) {
      cameras_.push_back(std::make_unique<camera::DiskCamera>(
          image_paths.value()[i], camera_constants_[i]));
    } else if (camera_constants_[i].serial_id.has_value()) {
      absl::Status status;
      cameras_.push_back(std::make_unique<camera::UVCCamera>(
          camera_constants_[i], status, camera_log_dest));
      if (!status.ok()) {
        LOG(WARNING) << "Unable to create uvc camera for unambiguous solver: "
                     << status.message();
      }
    } else {
      cameras_.push_back(std::make_unique<camera::CVCamera>(
          camera_constants_[i], camera_log_dest));
    }
    auto intrinsics =
        utils::ReadIntrinsics(camera_constants_[i].intrinsics_path.value());
    if (true /*camera_constants_[i].use_cpu.has_value() &&
        camera_constants[i].use_cpu.value()*/) {
      detectors_.push_back(std::make_unique<OpenCVAprilTagDetector>(
          camera_constants_[i].frame_width.value(),
          camera_constants_[i].frame_height.value(), intrinsics));
    } else {
      detectors_.push_back(std::make_unique<GPUAprilTagDetector>(
          camera_constants_[i].frame_width.value(),
          camera_constants_[i].frame_height.value(), intrinsics));
    }
  }
  for (size_t i = 0; i < cameras_.size(); i++) {
    camera_threads_.emplace_back([this, i, image_paths]() -> void {
      while (run_cameras_) {
        camera::timestamped_frame_t timestamped_frame;
        timestamped_frame = cameras_[i]->GetFrame();
        if (timestamped_frame.invalid) {
          continue;  // this is ok because GetFrame is blocking
        }
        if (timestamped_frame.timestamp - last_write_times_[i] >
            1.0 / kenforced_streamer_fps) {
          streamers_[i].WriteFrame(timestamped_frame.frame);
          last_write_times_[i] = timestamped_frame.timestamp;
        }
        std::vector<localization::tag_detection_t> detections =
            detectors_[i]->GetTagDetections(timestamped_frame);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          timestamped_frames_[i] = std::move(timestamped_frame);
          tag_detections_[i] = std::move(detections);
        }
        has_new_detections_.release();
      }
    });
  }
}

auto MultiCameraDetector::GetTagDetections()
    -> std::vector<std::vector<tag_detection_t>> {
  has_new_detections_.acquire();
  std::vector<std::vector<tag_detection_t>> tag_detections;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tag_detections = tag_detections_;
  }
  return tag_detections;
}

// Expensive, do not use inside of a loop because of copy
auto MultiCameraDetector::GetCVFrames() -> std::vector<cv::Mat> {
  std::vector<cv::Mat> frames;
  frames.reserve(cameras_.size());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < cameras_.size(); i++) {
      frames.push_back(timestamped_frames_[i].frame.clone());
    }
  }
  return frames;
}

MultiCameraDetector::~MultiCameraDetector() {
  run_cameras_ = false;
  for (auto& t : camera_threads_) {
    if (t.joinable())
      t.join();
  }
}

}  // namespace localization
