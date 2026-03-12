#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
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

  camera::camera_constant_t camera_constant = camera::SelectCameraConfig(
      absl::GetFlag(FLAGS_camera_name),
      camera::GetCameraConstants("/bos/constants/camera_constants.json"));

  camera::CVCamera camera(camera_constant, absl::GetFlag(FLAGS_log_path));

  camera::CscoreStreamer streamer("frame_shower",
                                  absl::GetFlag(FLAGS_port).value_or(4971), 30,
                                  camera.GetFrame().frame);

  LOG(INFO) << "Camera opened successfully" << std::endl;

  cv::Mat frame = camera.GetFrame().frame;
  LOG(INFO) << "Size of frame" << frame.size;
  while (true) {
    frame = camera.GetFrame().frame;
    streamer.WriteFrame(frame);
  }
  return 0;
}
