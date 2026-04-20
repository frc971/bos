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
  utils::StartNetworktables(9971);

  std::string log_path = frc::DataLogManager::GetLogDir();
  camera_constants_t camera_constants = camera::GetCameraConstants();

  LOG(INFO) << "Loading constants";

  std::vector<camera::camera_constant_t> cameras = {
      camera_constants.at("main_bot_left"),
      camera_constants.at("main_bot_right"),
  };

  LOG(INFO) << "Loaded constants";

  localization::UnambiguousEstimator localizer(cameras,
                                               std::make_optional<uint>(5801));
  localizer.Run();
}
