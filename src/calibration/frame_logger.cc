#include <wpilibc/frc/Timer.h>
#include <filesystem>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"
#include "src/camera/write_frame.h"
#include "src/utils/nt_utils.h"
int main() {
  utils::StartNetworktables();

  camera::CscoreStreamer streamer("frame_shower", 4971, 30, 1080, 1080);

  camera::Camera config = camera::SelectCameraConfig();
  std::unique_ptr<camera::ICamera> camera = camera::GetCameraStream(config);

  std::string log_name;
  std::cout << "What is the log name?" << std::endl;
  std::cin >> log_name;
  std::string log_folder = "/bos/logs/" + log_name;
  std::cout << "Writing to: " << log_folder << std::endl;

  std::cout << "Successfully created folder: "
            << std::filesystem::create_directory(log_folder);

  cv::Mat frame;
  while (true) {
    camera->GetFrame(frame);
    camera::timestamped_frame_t timestamped_frame{
        .frame = frame,
        .timestamp = frc::Timer::GetFPGATimestamp().to<double>()};
    camera::WriteFrame(log_folder, timestamped_frame);
    streamer.WriteFrame(frame);
  }
}
