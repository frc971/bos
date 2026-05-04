#include <gtest/gtest.h>
#include <XAD/StdCompatibility.hpp>
#include <XAD/XAD.hpp>
#include <filesystem>
#include "src/localization/joint_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/timer.h"
#include "src/utils/transform.h"

namespace fs = std::filesystem;

using camera::camera_constant_t;
using camera::GetCameraConstants;
using camera::timestamped_frame_t;
using localization::JointSolver;
using localization::kapriltag_layout;
using localization::OpenCVAprilTagDetector;
using localization::position_estimate_t;
using localization::SquareSolver;
using localization::tag_detection_t;
using utils::CameraMatrixFromJson;
using utils::ReadIntrinsics;

TEST(JointSolveTest, TestJointSolve) {  // NOLINT
  cv::Mat image = cv::imread("/bos-logs/log181/right/7.047703.jpg");
  auto camera_constant = GetCameraConstants().at("second_bot_right");
  auto square_solver = std::make_unique<SquareSolver>(camera_constant);
  auto joint_solver =
      std::make_unique<JointSolver>(std::vector{camera_constant});
  auto detector = std::make_unique<OpenCVAprilTagDetector>(
      camera_constant.frame_width.value(), camera_constant.frame_height.value(),
      ReadIntrinsics(camera_constant.intrinsics_path.value()));

  timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};
  auto detections = detector->GetTagDetections(timestamped_frame);
  auto detection = detections[0];

  auto square_solver_solution = square_solver->EstimatePosition({detection})[0];

  const std::map<std::string, std::vector<tag_detection_t>> camera_detections{
      {camera_constant.name, {detection}},
  };

  auto joint_solve_solution = joint_solver->EstimatePosition(
      camera_detections, square_solver_solution.pose);
}
