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

// TEST(JointSolveTest, TestJointSolve) {  // NOLINT
//   cv::Mat image = cv::imread("/bos-logs/log181/right/7.047703.jpg");
//   auto camera_constant = GetCameraConstants().at("second_bot_right");
//   auto square_solver = std::make_unique<SquareSolver>(camera_constant);
//   auto joint_solver =
//       std::make_unique<JointSolver>(std::vector{camera_constant});
//   auto detector = std::make_unique<OpenCVAprilTagDetector>(
//       camera_constant.frame_width.value(), camera_constant.frame_height.value(),
//       ReadIntrinsics(camera_constant.intrinsics_path.value()));
//
//   timestamped_frame_t timestamped_frame{
//       .frame = std::move(image), .timestamp = 0, .invalid = false};
//   auto detections = detector->GetTagDetections(timestamped_frame);
//   auto detection = detections[0];
//
//   auto square_solver_solution = square_solver->EstimatePosition({detection})[0];
//   const frc::Pose3d expected_pose = square_solver_solution.pose;
//
//   const std::map<std::string, std::vector<tag_detection_t>> camera_detections{
//       {camera_constant.name, {detection}},
//   };
//
//   // LOG(INFO) << "square_solver_solution\n"
//   //           << square_solver_solution.pose.ToMatrix();
//
//   square_solver_solution.pose =
//       square_solver_solution.pose.TransformBy(frc::Transform3d(
//           frc::Translation3d(units::meter_t{0.2}, units::meter_t{-0.15},
//                              units::meter_t{0.05}),
//           frc::Rotation3d(units::radian_t{0.06}, units::radian_t{-0.05},
//                           units::radian_t{0.04})));
//
//   auto joint_solve_solution = joint_solver->EstimatePosition(
//       camera_detections, square_solver_solution.pose);
//
//   ASSERT_LT(joint_solve_solution.loss, 1e-7);
//   ASSERT_TRUE(joint_solve_solution.pose.ToMatrix().isApprox(
//       expected_pose.ToMatrix(), 0.01));
//
//   // LOG(INFO) << "joint_solve_solution\n" << joint_solve_solution.pose.ToMatrix();
// }

TEST(JointSolveTest, TestJointSolveMultipleInputImages) {  // NOLINT
  const auto camera_constant = GetCameraConstants().at("second_bot_right");

  cv::Mat image_1 = cv::imread("/bos-logs/log181/right/7.047703.jpg");
  cv::Mat image_2 = cv::imread("/bos-logs/log181/right/7.143727.jpg");
  ASSERT_FALSE(image_1.empty());
  ASSERT_FALSE(image_2.empty());

  auto multi_tag_solver = std::make_unique<MultiTagSolver>(camera_constant);
  auto joint_solver =
      std::make_unique<JointSolver>(std::vector{camera_constant});
  auto detector = std::make_unique<OpenCVAprilTagDetector>(
      camera_constant.frame_width.value(), camera_constant.frame_height.value(),
      ReadIntrinsics(camera_constant.intrinsics_path.value()));

  timestamped_frame_t timestamped_frame_1{
      .frame = std::move(image_1), .timestamp = 0, .invalid = false};
  timestamped_frame_t timestamped_frame_2{
      .frame = std::move(image_2), .timestamp = 0.1, .invalid = false};

  auto detections_1 = detector->GetTagDetections(timestamped_frame_1);
  auto detections_2 = detector->GetTagDetections(timestamped_frame_2);

  ASSERT_FALSE(detections_1.empty());
  ASSERT_FALSE(detections_2.empty());

  std::vector<tag_detection_t> all_detections = detections_1;
  all_detections.insert(all_detections.end(), detections_2.begin(),
                        detections_2.end());

  auto multi_tag_solver_solution =
      multi_tag_solver->EstimatePosition(all_detections)[0];

  const std::map<std::string, std::vector<tag_detection_t>> camera_detections{
      {camera_constant.name, all_detections},
  };

  multi_tag_solver_solution.pose =
      multi_tag_solver_solution.pose.TransformBy(frc::Transform3d(
          frc::Translation3d(units::meter_t{0.25}, units::meter_t{-0.2},
                             units::meter_t{0.08}),
          frc::Rotation3d(units::radian_t{0.08}, units::radian_t{-0.07},
                          units::radian_t{0.06})));

  auto joint_solve_solution = joint_solver->EstimatePosition(
      camera_detections, multi_tag_solver_solution.pose);

  ASSERT_LT(joint_solve_solution.loss, 5e-3);
  ASSERT_FALSE(joint_solve_solution.invalid);
  ASSERT_EQ(joint_solve_solution.num_tags, all_detections.size());
}
