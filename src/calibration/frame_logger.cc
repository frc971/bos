#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"
#include "src/camera/write_frame.h"
#include "src/utils/nt_utils.h"

ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt, "");  //NOLINT

auto main(int argc, char* argv[]) -> int {
  absl::ParseCommandLine(argc, argv);
  utils::StartNetworktables();

  camera::CscoreStreamer streamer("frame_shower", 4971, 30, 1080, 1080);

  camera::Camera config =
      camera::SelectCameraConfig(absl::GetFlag(FLAGS_camera_name));
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
    frame = camera->GetFrame().frame;
    camera::timestamped_frame_t timestamped_frame{
        .frame = frame,
        .timestamp = frc::Timer::GetFPGATimestamp().to<double>()};
    camera::WriteFrame(log_folder, timestamped_frame);
    streamer.WriteFrame(frame);
  }
}
