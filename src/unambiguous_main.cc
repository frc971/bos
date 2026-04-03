#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cv_camera.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/localization/unambiguous_estimator.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"

using camera::camera_constants_t;
auto main() -> int {
  utils::StartNetworktables();
  // TODO configure vision bot camera paths

  std::string log_path = frc::DataLogManager::GetLogDir();
  camera_constants_t camera_constants = camera::GetCameraConstants();

  LOG(INFO) << "Starting cameras";

  std::vector<std::pair<camera::CameraConstant, localization::Detector>>
      cameras{
          //   {camera_constants.at("main_bot_front"),
          //    localization::Detector::OPENCV_CPU},
          {camera_constants.at("second_bot_left"),
           localization::Detector::OPENCV_CPU},
          {camera_constants.at("second_bot_right"),
           localization::Detector::OPENCV_CPU},
          {camera_constants.at("second_bot_front"),
           localization::Detector::OPENCV_CPU},
      };

  LOG(INFO) << "Started cameras";

  localization::UnambiguousEstimator localizer(cameras,
                                               std::make_optional<uint>(5801));
  localizer.Run();
}
