#include "disk_camera.h"
#include "src/camera/camera_source.h"

namespace camera {

DiskCamera::DiskCamera(std::string image_folder_path)
    : image_folder_path_(std::move(image_folder_path)) {
  for (auto& entry : std::filesystem::directory_iterator(image_folder_path_)) {
    std::string entry_name = entry.path().filename().string();

    // remove .png for timestamp
    image_paths_.push(timestamped_frame_path_t{
        .path = entry.path(),
        .timestamp = std::stod(entry_name.erase(entry_name.size() - 4, 4))});
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

  double timestamp;
  do {
    timestamp = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch() -
                    start_time_.time_since_epoch())
                    .count() +
                4.10;
    const double time_diff = image_paths_.top().timestamp - timestamp;
    std::this_thread::sleep_for(std::chrono::duration<double>(time_diff));
  } while (timestamp < image_paths_.top().timestamp);
  return timestamped_frame;
}

auto DiskCamera::Restart() -> void {}

}  // namespace camera
