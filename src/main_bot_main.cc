#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cv_camera.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"

using camera::Camera;
using camera::camera_constants;
auto main() -> int {
  utils::StartNetworktables();
  // TODO configure vision bot camera paths

  std::string log_path = frc::DataLogManager::GetLogDir();

  LOG(INFO) << "Starting cameras with right camera disabled";
  camera::CameraSource left_camera = camera::CameraSource(
      "Left", std::make_unique<camera::CVCamera>(
                  camera_constants[Camera::MAIN_ROBOT_LEFT_CAMERA]));

  camera::CameraSource right_camera = camera::CameraSource(
      "Right", std::make_unique<camera::CVCamera>(
                   camera_constants[Camera::MAIN_ROBOT_RIGHT_CAMERA]));

  LOG(INFO) << "Started cameras";
  std::this_thread::sleep_for(std::chrono::seconds(2));
  LOG(INFO) << "Starting estimators";

  std::thread left_thread(
      localization::RunLocalization, std::ref(left_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          left_camera.GetFrame().cols, left_camera.GetFrame().rows,
          utils::ReadIntrinsics(camera_constants[Camera::MAIN_ROBOT_LEFT_CAMERA]
                                    .intrinsics_path)),
      std::make_unique<localization::MultiTagSolver>(
          Camera::MAIN_ROBOT_LEFT_CAMERA),
      camera::camera_constants[Camera::MAIN_ROBOT_LEFT_CAMERA].extrinsics_path,
      4972, false);

  std::thread right_thread(
      localization::RunLocalization, std::ref(right_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          right_camera.GetFrame().cols, right_camera.GetFrame().rows,
          utils::ReadIntrinsics(
              camera_constants[Camera::MAIN_ROBOT_RIGHT_CAMERA]
                  .intrinsics_path)),
      std::make_unique<localization::MultiTagSolver>(
          Camera::MAIN_ROBOT_RIGHT_CAMERA),
      camera_constants[Camera::MAIN_ROBOT_RIGHT_CAMERA].extrinsics_path, 4973,
      false);

  LOG(INFO) << "Started estimators";

  right_thread.join();
}
