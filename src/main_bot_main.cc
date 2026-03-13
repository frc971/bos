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
  std::unique_ptr<camera::CameraSource> front_camera =
      std::make_unique<camera::CameraSource>(
          "Front", std::make_unique<camera::CVCamera>(
                       camera_constants[Camera::MAIN_ROBOT_FRONT_CAMERA]));

  std::unique_ptr<camera::CameraSource> left_camera =
      std::make_unique<camera::CameraSource>(
          "Left", std::make_unique<camera::CVCamera>(
                      camera_constants[Camera::MAIN_ROBOT_LEFT_CAMERA],
                      fmt::format("{}/left", log_path)));

  std::unique_ptr<camera::CameraSource> right_camera =
      std::make_unique<camera::CameraSource>(
          "Right", std::make_unique<camera::CVCamera>(
                       camera_constants[Camera::MAIN_ROBOT_RIGHT_CAMERA],
                       fmt::format("{}/right", log_path)));

  LOG(INFO) << "Started cameras";
  std::this_thread::sleep_for(std::chrono::seconds(2));
  LOG(INFO) << "Starting estimators";
  std::vector<std::pair<camera::Camera, std::unique_ptr<camera::CameraSource>>>
      camera_sources;
  camera_sources.emplace_back(camera::MAIN_ROBOT_FRONT_CAMERA,
                              std::move(front_camera));
  camera_sources.emplace_back(camera::MAIN_ROBOT_LEFT_CAMERA,
                              std::move(left_camera));
  camera_sources.emplace_back(camera::MAIN_ROBOT_RIGHT_CAMERA,
                              std::move(right_camera));
  std::thread joint_solve_thread(
      localization::RunJointSolve, std::ref(camera_sources),
      std::make_unique<localization::OpenCVAprilTagDetector>(
          right_camera->GetFrame().cols, right_camera->GetFrame().rows,
          utils::ReadIntrinsics(
              camera_constants[Camera::MAIN_ROBOT_RIGHT_CAMERA]
                  .intrinsics_path)),
      5801, false, false);
  LOG(INFO) << "Started estimators";
}
