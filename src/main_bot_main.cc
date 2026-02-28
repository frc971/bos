#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cv_camera.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/pathing/controller.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"

using camera::Camera;
using camera::camera_constants;
auto main() -> int {
  utils::StartNetworktables();
  // TODO configure vision bot camera paths

  // NT for enabling/disabling pathing controller
  auto instance = nt::NetworkTableInstance::GetDefault();
  auto table = instance.GetTable("Pathing");
  auto enabled_entry = table->GetEntry("Enabled");

  LOG(INFO) << "Starting cameras";
  // camera::CameraSource front_camera = camera::CameraSource(
  //     "front",
  //     std::make_unique<camera::CVCamera>(
  //         camera::camera_constants[camera::Camera::MAIN_ROBOT_FRONT_CAMERA]));

  camera::CameraSource left_camera = camera::CameraSource(
      "Left", std::make_unique<camera::CVCamera>(
                  camera_constants[Camera::MAIN_ROBOT_LEFT_CAMERA]));

  camera::CameraSource right_camera = camera::CameraSource(
      "Right", std::make_unique<camera::CVCamera>(
                   camera_constants[Camera::MAIN_ROBOT_RIGHT_CAMERA]));

  LOG(INFO) << "Started cameras";
  std::this_thread::sleep_for(std::chrono::seconds(2));
  LOG(INFO) << "Starting estimators";

  // std::thread front_thread(
  //     localization::RunLocalization, std::ref(front_camera),
  //     std::make_unique<localization::GPUAprilTagDetector>(
  //         front_camera.GetFrame().cols, front_camera.GetFrame().rows,
  //         utils::ReadIntrinsics(
  //             camera_constants[Camera::MAIN_ROBOT_FRONT_CAMERA]
  //                 .intrinsics_path)),
  //     std::make_unique<localization::MultiTagSolver>(
  //         Camera::MAIN_ROBOT_FRONT_CAMERA),
  //     camera_constants[Camera::MAIN_ROBOT_FRONT_CAMERA].extrinsics_path, 4971,
  //     false);

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

  left_thread.join();

  // Main control loop for pathing controller
  std::unique_ptr<pathing::Controller> controller;
  std::thread controller_thread;
  bool last_enabled = false;

  LOG(INFO) << "Entering main control loop";

  while (true) {
    bool enabled = enabled_entry.GetBoolean(false);

    if (enabled && !last_enabled) {
      LOG(INFO) << "Starting pathing controller";
      controller = std::make_unique<pathing::Controller>();
      controller_thread = std::thread([&controller]() { controller->Send(); });
    } else if (!enabled && last_enabled) {
      LOG(INFO) << "Stopping pathing controller";
      if (controller) {
        controller->Stop();
        if (controller_thread.joinable()) {
          controller_thread.join();
        }
        controller.reset();
      }
    }

    last_enabled = enabled;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::this_thread::sleep_for(std::chrono::hours::max());
}
