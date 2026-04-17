#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cv_camera.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/networktable_sender.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
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

auto MakeDetector(const camera::camera_constant_t& camera_constant,
                  int image_width, int image_height)
    -> std::unique_ptr<localization::IAprilTagDetector> {
  const auto intrinsics =
      utils::ReadIntrinsics(camera_constant.intrinsics_path.value());
  if (camera_constant.detector_backend == camera::DetectorBackend::kGpu) {
    return std::make_unique<localization::GPUAprilTagDetector>(
        image_width, image_height, intrinsics);
  }
  return std::make_unique<localization::OpenCVAprilTagDetector>(
      image_width, image_height, intrinsics);
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
    LOG(FATAL) << "Unsupported robot for robot_main: " << robot;
  }

  std::string log_path = frc::DataLogManager::GetLogDir();
  camera_constants_t camera_constants =
      camera::GetCameraConstants(GetRobotCameraConstantsPath(robot));

  LOG(INFO) << "Starting estimators";

  std::thread left_thread([&]() {
    auto left_camera = std::make_unique<camera::CameraSource>(
        "Left", std::make_unique<camera::CVCamera>(camera_constants.at("left"),
                                                   fmt::format("{}/left", log_path)));
    cv::Mat left_camera_frame = left_camera->GetFrame();

    std::vector<std::unique_ptr<localization::IPositionSender>> left_sender;
    left_sender.emplace_back(std::make_unique<localization::NetworkTableSender>(
        camera_constants.at("left").name));

    localization::RunLocalization(
        std::move(left_camera),
        MakeDetector(camera_constants.at("left"), left_camera_frame.cols,
                     left_camera_frame.rows),
        std::make_unique<localization::MultiTagSolver>(
            camera_constants.at("left")),
        std::move(left_sender),
        camera_constants.at("left").extrinsics_path.value(), 5802, false);
  });

  std::thread right_thread([&]() {
    auto right_camera = std::make_unique<camera::CameraSource>(
        "Right", std::make_unique<camera::CVCamera>(
                     camera_constants.at("right"),
                     fmt::format("{}/right", log_path)));
    cv::Mat right_camera_frame = right_camera->GetFrame();

    std::vector<std::unique_ptr<localization::IPositionSender>> right_sender;
    right_sender.emplace_back(
        std::make_unique<localization::NetworkTableSender>(
            camera_constants.at("right").name));

    localization::RunLocalization(
        std::move(right_camera),
        MakeDetector(camera_constants.at("right"), right_camera_frame.cols,
                     right_camera_frame.rows),
        std::make_unique<localization::MultiTagSolver>(
            camera_constants.at("right")),
        std::move(right_sender),
        camera_constants.at("right").extrinsics_path.value(), 5803, false);
  });

  std::optional<std::thread> front_thread = std::nullopt;
  if (IsMainBot(robot)) {
    front_thread.emplace([&]() {
      auto front_camera = std::make_unique<camera::CameraSource>(
          "Front",
          std::make_unique<camera::CVCamera>(camera_constants.at("front")));
      cv::Mat front_camera_frame = front_camera->GetFrame();

      std::vector<std::unique_ptr<localization::IPositionSender>> front_sender;
      front_sender.emplace_back(
          std::make_unique<localization::NetworkTableSender>(
              camera_constants.at("front").name));

      localization::RunLocalization(
          std::move(front_camera),
          MakeDetector(camera_constants.at("front"), front_camera_frame.cols,
                       front_camera_frame.rows),
          std::make_unique<localization::MultiTagSolver>(
              camera_constants.at("front")),
          std::move(front_sender),
          camera_constants.at("front").extrinsics_path.value(), 5801, false);
    });
  }

  LOG(INFO) << "Started estimators";

  left_thread.join();
  return 0;
}
