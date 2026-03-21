#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/disk_camera.h"
#include "src/camera/select_camera.h"
#include "src/localization/gpu_apriltag_detector.h"
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

  camera::camera_constant_t camera_constant =
      camera::GetCameraConstants().at("main_bot_left");
  camera::CameraSource source(
      "stress_test_camera",
      std::make_unique<camera::DiskCamera>("/bos/joint_bad_frames/", 2), true);
  cv::Mat frame = source.GetFrame();

  camera::CscoreStreamer streamer("tag_estimator_test", 5801, 30, frame);

  localization::OpenCVAprilTagDetector detector(
      frame.cols, frame.rows,
      utils::ReadIntrinsics(camera_constant.intrinsics_path.value()));

  localization::SquareSolver solver(camera_constant);

  camera::timestamped_frame_t timestamped_frame;
  while (true) {
    utils::Timer timer("tag estimator apriltag", time);
    timestamped_frame = source.Get();

    std::vector<localization::tag_detection_t> tag_detections =
        detector.GetTagDetections(timestamped_frame);
    // std::vector<localization::position_estimate_t> position_estimates =
    //     solver.EstimatePosition(tag_detections);
    // for (auto& position_estimate : position_estimates) {
    //   LOG(INFO) << position_estimate;
    // }
    //

    for (auto& tag_detection : tag_detections) {
      for (auto& corner : tag_detection.corners) {
        cv::circle(timestamped_frame.frame, corner, 10, cv::Scalar(0, 0, 255));
      }
    }

    streamer.WriteFrame(timestamped_frame.frame);
    std::this_thread::sleep_for(std::chrono::duration<double>(.05));
    if (tag_detections.size() > 0) {
      cv::imwrite(fmt::format("processed/{}.jpg", timestamped_frame.timestamp),
                  timestamped_frame.frame);
    }
  }
}
