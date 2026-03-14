#include <memory>
#include <optional>
#include <string>
#include "absl/flags/flag.h"
#include "absl/flags/internal/flag.h"
#include "absl/flags/parse.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cv_camera.h"
#include "src/camera/disk_camera.h"
#include "src/camera/select_camera.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"

ABSL_FLAG(std::string, image_folder, "",  //NOLINT
          "Path to folder of test images");
ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt,  //NOLINT
          "Camera name");
ABSL_FLAG(int, port, 5801, "Port");                            //NOLINT
ABSL_FLAG(std::optional<std::string>, log_path, std::nullopt,  //NOLINT
          "Log path");

auto main(int argc, char** argv) -> int {

  // TODO: Should I use the log path here or just the DataLogManager's default log directory
  std::string log_path = frc::DataLogManager::GetLogDir();

  absl::ParseCommandLine(argc, argv);

  camera::CameraSource camera("disk", std::make_unique<camera::DiskCamera>(
                                          absl::GetFlag(FLAGS_image_folder)));

  auto frame = camera.GetFrame();

  camera::Camera config =
      camera::SelectCameraConfig(absl::GetFlag(FLAGS_camera_name));

  localization::RunLocalization(
      camera,
      std::make_unique<localization::OpenCVAprilTagDetector>(
          frame.cols, frame.rows,
          utils::GetJson(camera::camera_constants[config].intrinsics_path)),
      std::make_unique<localization::MultiTagSolver>(config),
      camera::camera_constants[config].extrinsics_path,
      absl::GetFlag(FLAGS_port), false);
}
