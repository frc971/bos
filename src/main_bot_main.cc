#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cv_camera.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"
auto main() -> int {
  utils::StartNetworktables();
  // TODO configure vision bot camera paths

  LOG(INFO) << "Starting cameras";
  camera::CameraSource front_camera = camera::CameraSource(
      "front",
      std::make_unique<camera::CVCamera>(
          camera::camera_constants[camera::Camera::MAIN_ROBOT_FRONT_CAMERA]));

  camera::CameraSource left_camera = camera::CameraSource(
      "front_left",
      std::make_unique<camera::CVCamera>(
          camera::camera_constants[camera::Camera::MAIN_ROBOT_LEFT_CAMERA]));

  camera::CameraSource right_camera = camera::CameraSource(
      "back_right",
      std::make_unique<camera::CVCamera>(
          camera::camera_constants[camera::Camera::MAIN_ROBOT_RIGHT_CAMERA]));

  LOG(INFO) << "Started cameras";
  LOG(INFO) << "Starting estimators";

  std::thread front_thread(
      localization::run_localization, std::ref(front_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          front_camera.GetFrame().cols, front_camera.GetFrame().rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::MAIN_ROBOT_FRONT_CAMERA]
                  .intrinsics_path)),
      std::make_unique<localization::SquareSolver>(
          camera::Camera::MAIN_ROBOT_LEFT_CAMERA),
      camera::camera_constants[camera::Camera::MAIN_ROBOT_FRONT_CAMERA]
          .extrinsics_path,
      4971, false);

  std::thread left_thread(
      localization::run_localization, std::ref(left_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          left_camera.GetFrame().cols, left_camera.GetFrame().rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::MAIN_ROBOT_LEFT_CAMERA]
                  .intrinsics_path)),
      std::make_unique<localization::SquareSolver>(
          camera::Camera::MAIN_ROBOT_LEFT_CAMERA),
      camera::camera_constants[camera::Camera::MAIN_ROBOT_LEFT_CAMERA]
          .extrinsics_path,
      4972, false);

  std::thread right_thread(
      localization::run_localization, std::ref(right_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          right_camera.GetFrame().cols, right_camera.GetFrame().rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::MAIN_ROBOT_RIGHT_CAMERA]
                  .intrinsics_path)),
      std::make_unique<localization::SquareSolver>(
          camera::Camera::MAIN_ROBOT_RIGHT_CAMERA),
      camera::camera_constants[camera::Camera::MAIN_ROBOT_RIGHT_CAMERA]
          .extrinsics_path,
      4973, false);

  LOG(INFO) << "Started estimators";

  cv::waitKey(0);
}
