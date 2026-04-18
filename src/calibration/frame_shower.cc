#include <memory>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"
#include "src/utils/log.h"

ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt, "");  //NOLINT
ABSL_FLAG(std::optional<int>, port, std::nullopt, "");                 //NOLINT
ABSL_FLAG(std::optional<std::string>, log_path, std::nullopt, "");     //NOLINT

auto main(int argc, char* argv[]) -> int {
  absl::ParseCommandLine(argc, argv);

  std::unique_ptr<camera::ICamera> camera = camera::SelectCameraConfig(
      absl::GetFlag(FLAGS_camera_name), camera::GetCameraConstants());

  camera::timestamped_frame_t timestamped_frame;
  camera->GetFrame(&timestamped_frame);
  camera::CscoreStreamer streamer("frame_shower",
                                  absl::GetFlag(FLAGS_port).value_or(5801), 30,
                                  timestamped_frame.frame);

  LOG(INFO) << "Camera opened successfully" << std::endl;

  LOG(INFO) << "Size of frame: " << timestamped_frame.frame.size;
  while (true) {
    camera->GetFrame(&timestamped_frame);
    streamer.WriteFrame(timestamped_frame.frame);
  }
  return 0;
}
