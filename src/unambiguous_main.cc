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
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"

using camera::camera_constants_t;

ABSL_FLAG(std::string, robot, "main_bot",  // NOLINT
          "Robot name used to select robot-specific camera constants");

namespace {

auto GetRobotCameraConstantsPath(const std::string& robot) -> std::string {
  return "/bos/constants/" + robot + "/camera_constants.json";
}

auto IsMainBot(const std::string& robot) -> bool { return robot == "main_bot"; }

auto IsSecondBot(const std::string& robot) -> bool {
  return robot == "second_bot";
}

}  // namespace

auto main(int argc, char** argv) -> int {
  absl::ParseCommandLine(argc, argv);

  const std::string robot = absl::GetFlag(FLAGS_robot);
  if (IsMainBot(robot)) {
    utils::StartNetworktables(9971);
  } else if (IsSecondBot(robot)) {
    utils::StartNetworktables();
  } else {
    LOG(FATAL) << "Unsupported robot for unambiguous_main: " << robot;
  }

  camera_constants_t camera_constants =
      camera::GetCameraConstants(GetRobotCameraConstantsPath(robot));

  LOG(INFO) << "Starting cameras";

  std::vector<camera::CameraConstant> cameras{camera_constants.at("left"),
                                              camera_constants.at("right")};
  if (IsSecondBot(robot)) {
    cameras.emplace_back(camera_constants.at("front"));
  }

  LOG(INFO) << "Started cameras";

  localization::UnambiguousEstimator localizer(cameras,
                                               std::make_optional<uint>(5801));
  localizer.Run();
}
