#include "disk_camera.h"
#include "src/camera/camera_source.h"

namespace camera {

DiskCamera::DiskCamera(std::string image_folder_path)
    : image_folder_path_(std::move(image_folder_path)) {
  for (auto& entry : std::filesystem::directory_iterator(image_folder_path)) {
    std::cout << entry;
    std::string folder_path = entry.path().string();

    // remove .png for timestamp
    image_paths_.push(timestamped_frame_path_t{
        .path = entry.path(),
        .timestamp = std::stod(entry.path().filename().string().erase(
            folder_path.size() - 4, 4))});
  }
}
auto DiskCamera::GetFrame() -> timestamped_frame_t {
  if (image_paths_.empty()) {
    std::cout << "Finished reading all frames from DiskCamera. Folder path: "
              << image_folder_path_ << std::endl;
    exit(0);
  }
  timestamped_frame_t timestamped_frame{
      .frame = cv::imread(image_paths_.top().path),
      .timestamp = image_paths_.top().timestamp};
  image_paths_.pop();

  auto timestamp = frc::Timer::GetFPGATimestamp().to<double>();
  while (timestamp < image_paths_.top().timestamp) {
    std::this_thread::sleep_for(std::chrono::duration<double>(
        image_paths_.top().timestamp - timestamp));
    timestamp = frc::Timer::GetFPGATimestamp().to<double>();
  }
  return timestamped_frame;
}
}  // namespace camera
