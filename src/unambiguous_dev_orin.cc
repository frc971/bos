#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cv_camera.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/networktable_sender.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/localization/unambiguous_estimator.h"
#include "src/pathing/controller.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"
#include "src/utils/stop.h"

using camera::camera_constants_t;
auto main() -> int {
  stop::RegisterHandler();
<<<<<<< HEAD:src/unambiguous_first.cc
  auto inst = nt::NetworkTableInstance::GetDefault();
  inst.StartClient4("unambiguous_first");
  inst.SetServer("localhost");
=======
  // utils::StartNetworktables();
>>>>>>> 95efd2cf5e34df43516de59dbd33f6950af9eadd:src/unambiguous_dev_orin.cc

  std::string log_path = frc::DataLogManager::GetLogDir();
  camera_constants_t camera_constants = camera::GetCameraConstants();

  std::vector<camera::CameraConstant> cameras{camera_constants.at("dev_orin")};

  std::jthread pathing(pathing::RunController,
                       "/root/bos/constants/navgrid.json", true);

  std::jthread thread([cameras](const std::stop_token& stop_token) {
    localization::MultiCameraDetector detector_source(cameras);
    std::this_thread::sleep_for(std::chrono::duration<double>(2));
    localization::RunJointLocalization(
        stop_token, detector_source,
        std::make_unique<localization::UnambiguousEstimator>(cameras),
        std::make_unique<localization::NetworkTableSender>("Left", false));
  });

  LOG(INFO) << "Started localization";
  stop::WaitUntilStop();
  LOG(INFO) << "Stopping";
}
