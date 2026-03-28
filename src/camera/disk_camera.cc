#include "disk_camera.h"
#include "absl/log/check.h"
#include "src/camera/camera_source.h"

namespace camera {

DiskCamera::DiskCamera(std::string image_folder_path, double speed,
                       std::optional<double> start, std::optional<double> end)
    : speed_(speed),
      start_(start),
      end_(end),
      image_folder_path_(std::move(image_folder_path)) {
  CHECK(!start_.has_value() || !end_.has_value() ||
        end_.value() > start_.value());
  for (auto& entry : std::filesystem::directory_iterator(image_folder_path_)) {
    std::string entry_name = entry.path().filename().string();

    double timestamp = std::stod(entry_name.erase(entry_name.size() - 4, 4));

    if (start.has_value() && timestamp < start_) {
      continue;
    }
    if (end.has_value() && timestamp > end_) {
      continue;
    }
    // remove .png for timestamp
    image_paths_.push(
        timestamped_frame_path_t{.path = entry.path(), .timestamp = timestamp});
  }

  auto offset = image_paths_.top().timestamp;
  std::priority_queue<TimestampedFramePath, std::vector<TimestampedFramePath>,
                      CompareTimestampedFramePath>
      normalized;
  while (!image_paths_.empty()) {
    auto entry = image_paths_.top();
    if (cv::imread(image_paths_.top().path).empty()) {
      std::cout << "EMPTY FRAME" << std::endl;
      break;
    }
    image_paths_.pop();
    entry.timestamp -= offset;
    normalized.push(entry);
  }
  image_paths_ = std::move(normalized);
}
auto DiskCamera::GetFrame() -> timestamped_frame_t {
  if (image_paths_.empty()) {
    std::cout << "Finished reading all frames from DiskCamera. Folder path: "
              << image_folder_path_ << std::endl;
    frc::DataLogManager::Stop();
    return {.invalid = true};
  }

  double recorded_ts = image_paths_.top().timestamp;
  std::cout << "Recorded ts: " << recorded_ts + start_.value_or(0) << std::endl;
  timestamped_frame_t timestamped_frame{
      .frame = cv::imread(image_paths_.top().path),
      .timestamp = recorded_ts + start_.value_or(0)};
  image_paths_.pop();

  if (!image_paths_.empty()) {
    double delta = image_paths_.top().timestamp - recorded_ts;
    std::this_thread::sleep_for(std::chrono::duration<double>(delta / speed_));
  }

  return timestamped_frame;
}

auto DiskCamera::Restart() -> void {}

}  // namespace camera
