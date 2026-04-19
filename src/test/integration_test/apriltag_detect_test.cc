#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/nvidia_apriltag_detector.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/timer.h"

ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt, "");  //NOLINT
ABSL_FLAG(std::optional<bool>, time, std::nullopt, "");                //NOLINT

using json = nlohmann::json;

auto main(int argc, char* argv[]) -> int {
  absl::ParseCommandLine(argc, argv);

  bool time = absl::GetFlag(FLAGS_time).value_or(false);

  std::unique_ptr<camera::ICamera> camera = camera::SelectCameraConfig(
      absl::GetFlag(FLAGS_camera_name), camera::GetCameraConstants());
  auto camera_constant = camera->GetCameraConstant();
  camera::CameraSource source("stress_test_camera", std::move(camera));
  LOG(INFO) << "CAMERA CREATED";
  localization::SquareSolver solver(camera_constant);
  cv::Mat frame = source.GetFrame();
  LOG(INFO) << "FIRST FRAME ACQUIRED";

  camera::CscoreStreamer streamer("tag_estimator_test", 5801, 30, frame);

  LOG(INFO) << "STREAMER MADE";
  localization::GPUAprilTagDetector detector(
      frame.cols, frame.rows,
      utils::ReadIntrinsics(camera_constant.intrinsics_path.value()));
  LOG(INFO) << "GPU DETECTOR INITIALIZED";

  camera::timestamped_frame_t timestamped_frame;
  cv::Mat display_frame;
  while (true) {
    utils::Timer timer("tag estimator apriltag", time);
    timestamped_frame = source.Get();

    std::vector<localization::tag_detection_t> tag_detections =
        detector.GetTagDetections(timestamped_frame);
    std::vector<localization::position_estimate_t> position_estimates =
        solver.EstimatePosition(tag_detections);
    for (auto& position_estimate : position_estimates) {
      LOG(INFO) << position_estimate;
    }

    timestamped_frame.frame.copyTo(display_frame);
    for (auto& tag_detection : tag_detections) {
      for (auto& corner : tag_detection.corners) {
        cv::circle(display_frame, corner, 10, cv::Scalar(0, 0, 255));
      }
    }

    streamer.WriteFrame(display_frame);
  }
}
