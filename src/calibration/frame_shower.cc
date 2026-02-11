#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/select_camera.h"
#include "src/utils/log.h"

ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt, "");  //NOLINT
ABSL_FLAG(std::optional<int>, port, std::nullopt, "");                 //NOLINT

auto main(int argc, char* argv[]) -> int {
  absl::ParseCommandLine(argc, argv);

  camera::Camera config =
      camera::SelectCameraConfig(absl::GetFlag(FLAGS_camera_name));
  std::unique_ptr<camera::ICamera> camera = camera::GetCameraStream(config);

  camera::CscoreStreamer streamer(
      "frame_shower", absl::GetFlag(FLAGS_port).value_or(4971), 30, 1080, 1080);

  LOG(INFO) << "Camera opened successfully" << std::endl;

  while (true) {
    cv::Mat frame = camera->GetFrame().frame;
    streamer.WriteFrame(frame);
  }
  cv::destroyAllWindows();
  return 0;
}
