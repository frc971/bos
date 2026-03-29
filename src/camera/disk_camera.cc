#include "disk_camera.h"
#include <optional>
#include <utility>
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"

namespace camera {

DiskCamera::DiskCamera(std::string image_folder_path,
                       std::optional<camera_constant_t> camera_constant,
                       double speed)
    : speed(speed),
      camera_constant_(std::move(camera_constant)),
      image_folder_path_(std::move(image_folder_path)) {
  for (auto& entry : std::filesystem::directory_iterator(image_folder_path_)) {
    std::string entry_name = entry.path().filename().string();

    // remove .png for timestamp
    image_paths_.push(timestamped_frame_path_t{
        .path = entry.path(),
        .timestamp = std::stod(entry_name.erase(entry_name.size() - 4, 4))});
  }

  auto offset = image_paths_.top().timestamp;
  std::priority_queue<TimestampedFramePath, std::vector<TimestampedFramePath>,
                      CompareTimestampedFramePath>
      normalized;
  while (!image_paths_.empty()) {
    auto entry = image_paths_.top();
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
    exit(0);
    // return {.invalid = true};
  }

  double recorded_ts = image_paths_.top().timestamp;
  timestamped_frame_t timestamped_frame{
      .frame = cv::imread(image_paths_.top().path), .timestamp = recorded_ts};
  image_paths_.pop();

  if (!image_paths_.empty()) {
    double delta = image_paths_.top().timestamp - recorded_ts;
    std::this_thread::sleep_for(std::chrono::duration<double>(delta / speed));
  }

  return timestamped_frame;
}

auto DiskCamera::Restart() -> void {}

auto DiskCamera::GetCameraConstant() const -> camera_constant_t {
  return *camera_constant_;
}
}  // namespace camera
