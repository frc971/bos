#include <filesystem>
#include <optional>
#include "absl/flags/flag.h"
#include "absl/flags/internal/flag.h"
#include "absl/flags/parse.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/disk_camera.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/utils/camera_utils.h"
#include "src/utils/log.h"
#include "src/utils/nt_utils.h"

// for reference, example command:
// ./build/src/test/integration_test/localization_test --log_path=logs/log0/ --camera_name=main_bot_right --image_folder=logs/log181/right --speed=0.5

ABSL_FLAG(std::string, image_folder, "",  //NOLINT
          "Path to folder of test images");
ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt,  //NOLINT
          "Camera name");
ABSL_FLAG(int, port, 5801, "Port");  //NOLINT
ABSL_FLAG(std::optional<std::string>, log_path, std::nullopt,
          "Log path");                                   //NOLINT
ABSL_FLAG(double, speed, 0.01, "Delay between frames");  //NOLINT

auto main(int argc, char** argv) -> int {

  absl::ParseCommandLine(argc, argv);

  const std::string image_folder = absl::GetFlag(FLAGS_image_folder);
  const std::filesystem::path image_path(image_folder);
  if (image_folder.empty() || !std::filesystem::exists(image_path) ||
      !std::filesystem::is_directory(image_path)) {
    LOG(FATAL) << "Folder empty or doesn't exist";
  }

  std::string log_path = absl::GetFlag(FLAGS_log_path).has_value()
                             ? absl::GetFlag(FLAGS_log_path).value()
                             : frc::DataLogManager::GetLogDir();

  camera::CameraSource camera(
      "disk", std::make_unique<camera::DiskCamera>(image_folder,
                                                   absl::GetFlag(FLAGS_speed)));

  auto frame = camera.GetFrame();
  if (frame.empty()) {
    LOG(FATAL) << "No readable images found in folder: " << image_folder;
  }

  auto constants = camera::GetCameraConstants();
  std::string camera_name = absl::GetFlag(FLAGS_camera_name).value();

  localization::RunLocalization(
      camera,
      std::make_unique<localization::OpenCVAprilTagDetector>(
          frame.cols, frame.rows,
          utils::ReadIntrinsics(
              constants[camera_name].intrinsics_path.value())),
      std::make_unique<localization::MultiTagSolver>(constants[camera_name]),
      constants[camera_name].extrinsics_path.value(), absl::GetFlag(FLAGS_port),
      true);
}
