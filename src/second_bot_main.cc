#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cv_camera.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/networktable_sender.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/nt_utils.h"

using camera::camera_constants_t;
auto main() -> int {
  utils::StartNetworktables();

  std::string log_path = frc::DataLogManager::GetLogDir();
  camera_constants_t camera_constants = camera::GetCameraConstants();

  LOG(INFO) << "Starting estimators";

  std::thread left_thread([&]() {
    auto left_camera = std::make_unique<camera::CameraSource>(
        "Left", std::make_unique<camera::CVCamera>(
                    camera_constants.at("second_bot_left"),
                    fmt::format("{}/left", log_path)));
    cv::Mat left_camera_frame = left_camera->GetFrame();

    std::vector<std::unique_ptr<localization::IPositionSender>> left_sender;
    left_sender.emplace_back(std::make_unique<localization::NetworkTableSender>(
        camera_constants.at("second_bot_left").name));

    localization::RunLocalization(
        std::move(left_camera),
        std::make_unique<localization::GPUAprilTagDetector>(
            left_camera_frame.cols, left_camera_frame.rows,
            utils::ReadIntrinsics(camera_constants.at("second_bot_left")
                                      .intrinsics_path.value())),
        std::make_unique<localization::MultiTagSolver>(
            camera_constants.at("second_bot_left")),
        std::move(left_sender), false);
  });

  std::thread right_thread([&]() {
    auto right_camera = std::make_unique<camera::CameraSource>(
        "Right", std::make_unique<camera::CVCamera>(
                     camera_constants.at("second_bot_right"),
                     fmt::format("{}/right", log_path)));
    cv::Mat right_camera_frame = right_camera->GetFrame();

    std::vector<std::unique_ptr<localization::IPositionSender>> right_sender;
    right_sender.emplace_back(
        std::make_unique<localization::NetworkTableSender>(
            camera_constants.at("second_bot_right").name));

    localization::RunLocalization(
        std::move(right_camera),
        std::make_unique<localization::GPUAprilTagDetector>(
            right_camera_frame.cols, right_camera_frame.rows,
            utils::ReadIntrinsics(camera_constants.at("second_bot_right")
                                      .intrinsics_path.value())),
        std::make_unique<localization::MultiTagSolver>(
            camera_constants.at("second_bot_right")),
        std::move(right_sender), false);
  });

  // TODO front camera

  LOG(INFO) << "Started estimators";

  left_thread.join();
}
