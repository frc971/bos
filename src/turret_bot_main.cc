#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/localization/joint_solver.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"
auto main() -> int {
  utils::StartNetworktables(9972);

  camera::CameraSource front_right_camera = camera::CameraSource(
      "FrontRight",
      std::make_unique<camera::CVCamera>(
          camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_RIGHT]));

  camera::CameraSource front_left_camera = camera::CameraSource(
      "FrontLeft",
      std::make_unique<camera::CVCamera>(
          camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_LEFT]));

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // camera::CameraSource back_right_camera = camera::CameraSource(
  //     "back_right",
  //     std::make_unique<camera::CVCamera>(
  //         camera::camera_constants[camera::Camera::IMX296_0])));
  //
  // camera::CameraSource back_left_camera = camera::CameraSource(
  //     "back_left",
  //     std::make_unique<camera::CVCamera>(
  //         camera::camera_constants[camera::Camera::IMX296_1])));

  LOG(INFO) << "Starting estimators";
  std::thread front_right_thread(
      localization::run_localization, std::ref(front_right_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          front_right_camera.GetFrame().cols,
          front_right_camera.GetFrame().rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_RIGHT]
                  .intrinsics_path)),
      std::make_unique<localization::SquareSolver>(
          camera::Camera::TURRET_BOT_FRONT_RIGHT),
      camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_RIGHT]
          .extrinsics_path,
      4971, false);

  std::thread front_left_thread(
      localization::run_localization, std::ref(front_left_camera),
      std::make_unique<localization::GPUAprilTagDetector>(
          front_left_camera.GetFrame().cols, front_left_camera.GetFrame().rows,
          utils::read_intrinsics(
              camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_LEFT]
                  .intrinsics_path)),
      std::make_unique<localization::SquareSolver>(
          camera::Camera::TURRET_BOT_FRONT_LEFT),
      camera::camera_constants[camera::Camera::TURRET_BOT_FRONT_LEFT]
          .extrinsics_path,
      4972, false);

  front_left_thread.join();
}
