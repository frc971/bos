#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cv_camera.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/networktable_sender.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/localization/unambiguous_estimator.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"

using camera::camera_constants_t;
auto main() -> int {
  utils::StartNetworktables(9971);

  std::string log_path = frc::DataLogManager::GetLogDir();
  camera_constants_t camera_constants = camera::GetCameraConstants();

  LOG(INFO) << "Starting cameras";

  std::vector<camera::CameraConstant> cameras{
      camera_constants.at("second_bot_left"),
      camera_constants.at("second_bot_right"),
      camera_constants.at("second_bot_front")};
  localization::MultiCameraDetector detector(cameras);

  LOG(INFO) << "Started cameras";
  std::this_thread::sleep_for(std::chrono::duration<double>(2));

  localization::UnambiguousEstimator localizer(cameras);
  std::vector<std::unique_ptr<localization::IPositionSender>> joint_sender;
  joint_sender.emplace_back(
      std::make_unique<localization::NetworkTableSender>("Left"));
}
