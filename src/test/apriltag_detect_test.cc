#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"
#include "src/localization/get_field_relitive_position.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/timer.h"

using json = nlohmann::json;

auto main() -> int {

  camera::CscoreStreamer streamer("tag_estimator_test", 4971, 30, 1080, 1080);

  camera::Camera config = camera::SelectCameraConfig();
  camera::CameraSource source("stress_test_camera",
                              camera::GetCameraStream(config));
  cv::Mat frame = source.GetFrame();

  localization::GPUAprilTagDetector detector(
      frame.cols, frame.rows,
      utils::read_intrinsics(camera::camera_constants[config].intrinsics_path));

  localization::SquareSolver solver(
      camera::camera_constants[config].intrinsics_path,
      camera::camera_constants[config].extrinsics_path);

  camera::timestamped_frame_t timestamped_frame;
  while (true) {
    utils::Timer timer("tag estimator apriltag");
    timestamped_frame = source.Get();

    std::vector<localization::tag_detection_t> tag_detections =
        detector.GetTagDetections(timestamped_frame);
    std::vector<localization::position_estimate_t> position_estimates =
        solver.EstimatePosition(tag_detections);
    for (auto& position_estimate : position_estimates) {
      LOG(INFO) << position_estimate;
    }

    for (auto& tag_detection : tag_detections) {
      for (auto& corner : tag_detection.corners) {
        cv::circle(frame, corner, 10, cv::Scalar(0, 0, 255));
      }
    }

    streamer.WriteFrame(frame);
    timer.Stop();
  }
}
