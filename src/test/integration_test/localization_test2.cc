#include <frc/DataLogManager.h>
#include <filesystem>
#include <optional>
#include "absl/flags/flag.h"
#include "absl/flags/internal/flag.h"
#include "absl/flags/parse.h"
#include "frc/DataLogManager.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/disk_camera.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/unambiguous_estimator.h"
#include "src/utils/camera_utils.h"
#include "src/utils/log.h"

// for reference, example command:
// ./build/src/test/integration_test/l2calization_test --camera_name=main_bot_right --image_folder=logs/log181/right --speed=0.5

ABSL_FLAG(std::string, image_folder, "",  //NOLINT
          "Path to folder of test images");
ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt,  //NOLINT
          "Camera name");
ABSL_FLAG(int, port, 5801, "Port");                      //NOLINT
ABSL_FLAG(double, speed, 0.01, "Delay between frames");  //NOLINT

auto main(int argc, char** argv) -> int {

  absl::ParseCommandLine(argc, argv);

  const std::string image_folder = absl::GetFlag(FLAGS_image_folder);
  const std::filesystem::path image_path(image_folder);
  if (image_folder.empty() || !std::filesystem::exists(image_path) ||
      !std::filesystem::is_directory(image_path)) {
    LOG(FATAL) << "Folder empty or doesn't exist";
  }

  auto constants = camera::GetCameraConstants();
  std::string camera_name = absl::GetFlag(FLAGS_camera_name).value();

  std::vector<std::pair<camera::CameraConstant, localization::Detector>>
      cameras{{constants.at(camera_name), localization::OPENCV_CPU}, {constants.at("main_bot_right"), localization::OPENCV_CPU}};

  auto paths = std::make_optional<std::vector<std::filesystem::path>>(
      {image_path, std::filesystem::path{"/bos/bos_logs/real_log/right"}});

  localization::UnambiguousEstimator estimator(cameras, paths);
  estimator.Run();
}
