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

  cv::Mat image_right = cv::imread("/bos-logs/log181/right/20.971783.jpg");
  cv::Mat image_left = cv::imread("/bos-logs/log181/left/20.935361.jpg");
  ASSERT_FALSE(image_right.empty());
  ASSERT_FALSE(image_left.empty());

  auto multi_tag_solver =
      std::make_unique<MultiTagSolver>(camera_constant_right);

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

  std::vector<tag_detection_t> all_detections = detections_right;
  all_detections.insert(all_detections.end(), detections_left.begin(),
                        detections_left.end());

  auto multi_tag_solver_solution =
      multi_tag_solver->EstimatePosition(detections_right)[0];

  const std::map<std::string, std::vector<tag_detection_t>> camera_detections{
      {camera_constant_right.name, detections_right},
      {camera_constant_left.name, detections_left},
  };

  multi_tag_solver_solution.pose =
      multi_tag_solver_solution.pose.TransformBy(frc::Transform3d(
          frc::Translation3d(units::meter_t{5.25}, units::meter_t{-0.2},
                             units::meter_t{0.08}),
          frc::Rotation3d(units::radian_t{0.08}, units::radian_t{-0.07},
                          units::radian_t{0.06})));

  auto joint_solve_solution = joint_solver->EstimatePosition(
      camera_detections, multi_tag_solver_solution.pose);

  ASSERT_LT(joint_solve_solution.loss, 1e-2);
  ASSERT_FALSE(joint_solve_solution.invalid);
  ASSERT_EQ(joint_solve_solution.num_tags, all_detections.size());
}
