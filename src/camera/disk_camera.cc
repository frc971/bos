#include "disk_camera.h"
#include <wpilibc/frc/Timer.h>
#include <cstdlib>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include "src/camera/camera_source.h"

namespace camera {

DiskCamera::DiskCamera(std::string image_folder_path)
    : image_folder_path_(std::move(image_folder_path)), current_frame_(0) {
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
void DiskCamera::GetFrame(cv::Mat& frame) {
  if (image_paths_.empty()) {
    std::cout << "Finished reading all frames from DiskCamera. Folder path: "
              << image_folder_path_ << std::endl;
    exit(0);
  }
  frame = cv::imread(image_paths_.top().path);

  auto timestamp = frc::Timer::GetFPGATimestamp().to<double>();
  while (timestamp < image_paths_.top().timestamp) {
    std::this_thread::sleep_for(std::chrono::duration<double>(
        image_paths_.top().timestamp - timestamp));
    timestamp = frc::Timer::GetFPGATimestamp().to<double>();
  }
  image_paths_.pop();
}
}  // namespace camera
