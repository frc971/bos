#include <string>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/log.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/disk_camera.h"

ABSL_FLAG(std::string, image_folder, "", "Path to the folder of test images");
ABSL_FLAG(std::string, camera_name, "", "Camera name");
ABSL_FLAG(uint, fps, 30, "Streaming frame rate");

auto main(int argc, char** argv) -> int {
  absl::ParseCommandLine(argc, argv);

  const auto camera_constants = camera::GetCameraConstants();
  const std::string camera_name = absl::GetFlag(FLAGS_camera_name);
  if (!camera_constants.contains(camera_name)) {
    LOG(FATAL) << "Unknown camera name: " << camera_name;
  }

  camera::DiskCamera camera(
      absl::GetFlag(FLAGS_image_folder), camera_constants.at(camera_name));
  auto frame = camera.GetFrame();
  if (frame.invalid || frame.frame.empty()) {
    LOG(FATAL) << "No readable images found in folder: "
               << absl::GetFlag(FLAGS_image_folder);
  }

  camera::CscoreStreamer streamer("frame_shower_test", 5801,
                                  absl::GetFlag(FLAGS_fps), frame.frame);
  while (!frame.invalid && !frame.frame.empty()) {
    streamer.WriteFrame(frame.frame);
    frame = camera.GetFrame();
  }

  return 0;
}
