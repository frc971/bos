#include <gtest/gtest.h>
#include <XAD/StdCompatibility.hpp>
#include <XAD/XAD.hpp>
#include <filesystem>
#include "src/localization/joint_solver.h"
#include "src/localization/multi_tag_solver.h"
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
using localization::MultiTagSolver;
using localization::OpenCVAprilTagDetector;
using localization::position_estimate_t;
using localization::SquareSolver;
using localization::tag_detection_t;
using utils::CameraMatrixFromJson;
using utils::ReadIntrinsics;

TEST(JointSolveTest, TestJointSolve) {  // NOLINT
  cv::Mat image = cv::imread("/bos-logs/log181/right/15.739774.jpg");
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
  const frc::Pose3d expected_pose = square_solver_solution.pose;

  const std::map<std::string, std::vector<tag_detection_t>> camera_detections{
      {camera_constant.name, {detection}},
  };

  square_solver_solution.pose =
      square_solver_solution.pose.TransformBy(frc::Transform3d(
          frc::Translation3d(units::meter_t{0.2}, units::meter_t{-0.15},
                             units::meter_t{0.05}),
          frc::Rotation3d(units::radian_t{0.06}, units::radian_t{-0.05},
                          units::radian_t{0.04})));

  auto joint_solve_solution = joint_solver->EstimatePosition(
      camera_detections, square_solver_solution.pose);

  ASSERT_LT(joint_solve_solution.loss, 1e-7);
  ASSERT_TRUE(joint_solve_solution.pose.ToMatrix().isApprox(
      expected_pose.ToMatrix(), 0.01));
}

TEST(JointSolveTest, TestJointSolveMultipleInputImages) {  // NOLINT
  const auto camera_constant_right =
      GetCameraConstants().at("second_bot_right");
  const auto camera_constant_left = GetCameraConstants().at("second_bot_left");

  cv::Mat image_right = cv::imread("/bos-logs/log181/right/11.067757.jpg");
  cv::Mat image_left = cv::imread("/bos-logs/log181/left/11.031403.jpg");
  ASSERT_FALSE(image_right.empty());
  ASSERT_FALSE(image_left.empty());

  auto square_solver = std::make_unique<SquareSolver>(camera_constant_right);
  auto joint_solver = std::make_unique<JointSolver>(
      std::vector{camera_constant_right, camera_constant_left});

  auto detector_right = std::make_unique<OpenCVAprilTagDetector>(
      camera_constant_right.frame_width.value(),
      camera_constant_right.frame_height.value(),
      ReadIntrinsics(camera_constant_right.intrinsics_path.value()));
  auto detector_left = std::make_unique<OpenCVAprilTagDetector>(
      camera_constant_left.frame_width.value(),
      camera_constant_left.frame_height.value(),
      ReadIntrinsics(camera_constant_left.intrinsics_path.value()));

  timestamped_frame_t timestamped_frame_right{
      .frame = std::move(image_right), .timestamp = 0, .invalid = false};
  timestamped_frame_t timestamped_frame_left{
      .frame = std::move(image_left), .timestamp = 0, .invalid = false};

  auto detections_right =
      detector_right->GetTagDetections(timestamped_frame_right);
  auto detections_left =
      detector_left->GetTagDetections(timestamped_frame_left);

  ASSERT_FALSE(detections_right.empty());
  ASSERT_FALSE(detections_left.empty());

  auto square_solution_right =
      square_solver->EstimatePosition(detections_right)[0];

  const std::map<std::string, std::vector<tag_detection_t>> camera_detections{
      {camera_constant_right.name, detections_right},
      // {camera_constant_left.name, detections_left},
  };

  auto fudged_pose = square_solution_right.pose.TransformBy(frc::Transform3d(
      frc::Translation3d(units::meter_t{1.25}, units::meter_t{-0.2},
                         units::meter_t{0.98}),
      frc::Rotation3d(units::radian_t{0.58}, units::radian_t{-0.07},
                      units::radian_t{0.06})));

  auto joint_solve_solution =
      joint_solver->EstimatePosition(camera_detections, fudged_pose);

  ASSERT_LT(joint_solve_solution.loss, 1e-8);
  ASSERT_FALSE(joint_solve_solution.invalid);
}

TEST(JointSolveTest, TestJointSolveLoss) {  // NOLINT
  const auto camera_constant_right =
      GetCameraConstants().at("second_bot_right");

  cv::Mat image_right = cv::imread("/bos-logs/log181/right/17.979762.jpg");
  ASSERT_FALSE(image_right.empty());

  auto square_solver = std::make_unique<SquareSolver>(camera_constant_right);
  auto joint_solver =
      std::make_unique<JointSolver>(std::vector{camera_constant_right});

  auto detector_right = std::make_unique<OpenCVAprilTagDetector>(
      camera_constant_right.frame_width.value(),
      camera_constant_right.frame_height.value(),
      ReadIntrinsics(camera_constant_right.intrinsics_path.value()));

  timestamped_frame_t timestamped_frame_right{
      .frame = std::move(image_right), .timestamp = 0, .invalid = false};

  auto detections_right =
      detector_right->GetTagDetections(timestamped_frame_right);

  ASSERT_GE(detections_right.size(), 4);

  const auto single_tag_detection = std::vector{detections_right[0]};

  std::vector<position_estimate_t> square_solutions =
      square_solver->EstimatePosition(detections_right);

  ASSERT_FALSE(square_solutions.empty());

  const std::map<std::string, std::vector<tag_detection_t>> camera_detections{
      {camera_constant_right.name, detections_right},
  };

  for (const auto& square_solution : square_solutions) {
    double square_loss = joint_solver->CalculateResidualLoss(
        square_solution.pose, camera_detections);
    ASSERT_LE(square_loss, 1e-3);
  }
}
