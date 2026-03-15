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

  std::unique_ptr<camera::CameraSource> left_src =
      std::make_unique<camera::CameraSource>(
          "Left",
          std::make_unique<camera::DiskCamera>("/bos/logs/log181/left"));
  std::unique_ptr<camera::CameraSource> right_src =
      std::make_unique<camera::CameraSource>(
          "Right",
          std::make_unique<camera::DiskCamera>("/bos/logs/log181/right"));

  LOG(INFO) << "Started cameras";
  LOG(INFO) << "Starting estimators";
  std::vector<
      std::pair<camera::CameraConstant, std::unique_ptr<camera::CameraSource>>>
      camera_sources;
  camera_sources.emplace_back(camera_constants.at("main_bot_left"),
                              std::move(left_src));
  camera_sources.emplace_back(camera_constants.at("main_bot_right"),
                              std::move(right_src));
  std::cout << "Made camera sources" << std::endl;
  std::thread joint_solve_thread(localization::RunJointSolve,
                                 std::ref(camera_sources), 5801, false, false);
  LOG(INFO) << "Started estimators";
  joint_solve_thread.join();
}
