#include <gtest/gtest.h>
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"

using camera::camera_constant_t;
using camera::GetCameraConstants;
using camera::timestamped_frame_t;
using localization::kapriltag_layout;
using localization::MultiTagSolver;
using localization::OpenCVAprilTagDetector;
using localization::position_estimate_t;
using localization::SquareSolver;
using localization::tag_detection_t;
using utils::CameraMatrixFromJson;
using utils::ReadIntrinsics;

TEST(MultiTagTest, EqualToSquareSolve) {  // NOLINT
  const auto camera_constant = GetCameraConstants().at("second_bot_right");

  cv::Mat image = cv::imread("/bos-logs/log181/right/15.739774.jpg");

  auto multi_tag_solver = std::make_unique<MultiTagSolver>(camera_constant);
  auto square_solver = std::make_unique<SquareSolver>(camera_constant);

  auto detector = std::make_unique<OpenCVAprilTagDetector>(
      camera_constant.frame_width.value(), camera_constant.frame_height.value(),
      ReadIntrinsics(camera_constant.intrinsics_path.value()));

  timestamped_frame_t timestamped_frame{
      .frame = std::move(image), .timestamp = 0, .invalid = false};

  auto detections = detector->GetTagDetections(timestamped_frame);
  auto detection = std::vector{detections[0]};

  position_estimate_t multi_tag_solution =
      multi_tag_solver->EstimatePosition(detection)[0];
  position_estimate_t square_solution =
      square_solver->EstimatePosition(detection)[0];

  // LOG(INFO) << multi_tag_solution.pose.ToMatrix();
  // LOG(INFO) << square_solution.pose.ToMatrix();
  // LOG(INFO) << multi_tag_solution;
  // LOG(INFO) << square_solution;

  ASSERT_TRUE(multi_tag_solution.pose.ToMatrix().isApprox(
      square_solution.pose.ToMatrix(), 0.1));
}
