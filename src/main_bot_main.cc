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

  std::cout << "Starting camera stuff" << std::endl;
  camera::camera_constants_t camera_constants = camera::GetCameraConstants();

  std::string log_path = frc::DataLogManager::GetLogDir();

  std::unique_ptr<camera::CameraSource> front_camera =
      std::make_unique<camera::CameraSource>(
          "Front", std::make_unique<camera::CVCamera>(
                       camera_constants.at("main_bot_front")));

  std::unique_ptr<camera::CameraSource> left_camera =
      std::make_unique<camera::CameraSource>(
          "Left", std::make_unique<camera::CVCamera>(
                      camera_constants.at("main_bot_left"),
                      fmt::format("{}/left", log_path)));

  std::unique_ptr<camera::CameraSource> right_camera =
      std::make_unique<camera::CameraSource>(
          "Right", std::make_unique<camera::CVCamera>(
                       camera_constants.at("main_bot_right"),
                       fmt::format("{}/right", log_path)));

  LOG(INFO) << "Started cameras";
  LOG(INFO) << "Starting estimators";
  std::vector<
      std::pair<camera::CameraConstant, std::unique_ptr<camera::CameraSource>>>
      camera_sources;
  camera_sources.emplace_back(camera_constants.at("main_bot_left"),
                              std::move(left_camera));
  camera_sources.emplace_back(camera_constants.at("main_bot_right"),
                              std::move(right_camera));
  camera_sources.emplace_back(camera_constants.at("main_bot_front"),
                              std::move(front_camera));
  std::cout << "Made camera sources" << std::endl;
  std::thread joint_solve_thread(localization::RunJointSolve,
                                 std::ref(camera_sources), 5801, false, false);
  LOG(INFO) << "Started estimators";
  joint_solve_thread.join();
}
