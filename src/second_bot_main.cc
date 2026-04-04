#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cv_camera.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"

using camera::camera_constants_t;
auto main() -> int {
  utils::StartNetworktables();

  std::string log_path = frc::DataLogManager::GetLogDir();
  camera_constants_t camera_constants = camera::GetCameraConstants();

  LOG(INFO) << "Starting cameras";
  camera::CameraSource front_camera =
      camera::CameraSource("Front", std::make_unique<camera::CVCamera>(
                                        camera_constants.at("second_bot_front"),
                                        fmt::format({"{}/front"}, log_path)));

  camera::CameraSource left_camera = camera::CameraSource(
      "Left",
      std::make_unique<camera::CVCamera>(camera_constants.at("second_bot_left"),
                                         fmt::format("{}/left", log_path)));

  camera::CameraSource right_camera =
      camera::CameraSource("Right", std::make_unique<camera::CVCamera>(
                                        camera_constants.at("second_bot_right"),
                                        fmt::format("{}/right", log_path)));

  LOG(INFO) << "Started cameras";
  LOG(INFO) << "Starting estimators";

  std::thread front_thread(
      localization::RunLocalization, std::ref(front_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          front_camera.GetFrame().cols, front_camera.GetFrame().rows,
          utils::ReadIntrinsics(
              camera_constants.at("second_bot_front").intrinsics_path.value())),
      std::make_unique<localization::MultiTagSolver>(
          camera_constants.at("second_bot_front")),
      camera_constants.at("second_bot_front").extrinsics_path.value(), 5801,
      false);

  std::thread left_thread(
      localization::RunLocalization, std::ref(left_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          left_camera.GetFrame().cols, left_camera.GetFrame().rows,
          utils::ReadIntrinsics(
              camera_constants.at("second_bot_left").intrinsics_path.value())),
      std::make_unique<localization::MultiTagSolver>(
          camera_constants.at("second_bot_left")),
      camera_constants.at("second_bot_left").extrinsics_path.value(), 5802,
      false);

  std::thread right_thread(
      localization::RunLocalization, std::ref(right_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          right_camera.GetFrame().cols, right_camera.GetFrame().rows,
          utils::ReadIntrinsics(
              camera_constants.at("second_bot_right").intrinsics_path.value())),
      std::make_unique<localization::MultiTagSolver>(
          camera_constants.at("second_bot_right")),
      camera_constants.at("second_bot_right").extrinsics_path.value(), 5803,
      false);

  LOG(INFO) << "Started estimators";

  // TODO find better way
  right_thread.join();
}
