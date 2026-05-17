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

  std::vector<camera::CameraConstant> cameras{camera_constants.at("dev_orin")};

  std::jthread thread([cameras] {
    localization::MultiCameraDetector detector_source(cameras);
    LOG(INFO) << "Started cameras";
    std::this_thread::sleep_for(std::chrono::duration<double>(2));
    localization::RunJointLocalization(
        detector_source,
        std::make_unique<localization::UnambiguousEstimator>(cameras),
        std::make_unique<localization::NetworkTableSender>("Left", false));
  });
}
