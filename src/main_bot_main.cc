#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cv_camera.h"
#include "src/camera/disk_camera.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"

auto main() -> int {
  utils::StartNetworktables();
  // TODO configure vision bot camera paths

  // std::string log_path = frc::DataLogManager::GetLogDir();
  camera::camera_constants_t camera_constants = camera::GetCameraConstants();

  LOG(INFO) << "Starting cameras";
  std::unique_ptr<camera::ICamera> front_camera =
      std::make_unique<camera::DiskCamera>("/bos/bos_logs/left");
  LOG(INFO) << "Front camera created";

  std::unique_ptr<camera::CameraSource> front_src =
      std::make_unique<camera::CameraSource>("name", std::move(front_camera));
  LOG(INFO) << "Source created";

  // std::unique_ptr<camera::CameraSource> left_camera =
  //     std::make_unique<camera::CameraSource>(
  //         "Left", std::make_unique<camera::CVCamera>(
  //                     camera_constants[Camera::MAIN_ROBOT_LEFT_CAMERA],
  //                     fmt::format("{}/left", log_path)));
  //
  // std::unique_ptr<camera::CameraSource> right_camera =
  //     std::make_unique<camera::CameraSource>(
  //         "Right", std::make_unique<camera::CVCamera>(
  //                      camera_constants[Camera::MAIN_ROBOT_RIGHT_CAMERA],
  //                      fmt::format("{}/right", log_path)));

  LOG(INFO) << "Started cameras";
  std::this_thread::sleep_for(std::chrono::seconds(2));
  LOG(INFO) << "Starting estimators";
  std::vector<
      std::pair<camera::CameraConstant, std::unique_ptr<camera::CameraSource>>>
      camera_sources;
  camera_sources.emplace_back(camera_constants.at("main_bot_left"),
                              std::move(front_src));
  std::cout << "Made camera source" << std::endl;
  // camera_sources.emplace_back(camera::MAIN_ROBOT_LEFT_CAMERA,
  //                             std::move(left_camera));
  // camera_sources.emplace_back(camera::MAIN_ROBOT_RIGHT_CAMERA,
  //                             std::move(right_camera));
  std::thread joint_solve_thread(
      localization::RunJointSolve, std::ref(camera_sources),
      std::make_unique<localization::OpenCVAprilTagDetector>(
          camera_sources[0].second->GetFrame().cols,
          camera_sources[0].second->GetFrame().rows,
          utils::ReadIntrinsics(
              camera_constants.at("main_bot_left").intrinsics_path.value())),
      5801, false, false);
  LOG(INFO) << "Started estimators";
  joint_solve_thread.join();
}
