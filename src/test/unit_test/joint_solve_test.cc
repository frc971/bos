#include <gtest/gtest.h>
#include <random>
#include "src/localization/joint_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/square_solver.h"
#include "src/test/unit_test/unit_test_utils.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

class JointSolverTest : public ::testing::Test {
 protected:
  camera::camera_constants_t camera_constants = camera::GetCameraConstants();

  std::vector<camera::CameraConstant> joint_solve_cameras =
      std::vector<camera::CameraConstant>{camera_constants.at("main_bot_left")};
  localization::SquareSolver square_solver;
  localization::JointSolver joint_solver;
  localization::OpenCVAprilTagDetector detector;
  JointSolverTest()
      : square_solver(camera_constants.at("main_bot_left")),
        joint_solver(joint_solve_cameras),
        detector(utils::ReadIntrinsics(
            camera_constants.at("main_bot_left").intrinsics_path.value())) {}
};

// TEST_F(JointSolverTest, MaintainsValidEstimateOpenRotation) {  // NOLINT
//   const localization::position_estimate_t square_solver_solution =
//       square_solver.EstimatePosition(test_utils::fake_detections, false)[0];
//   std::map<camera::CameraConstant, std::vector<localization::tag_detection_t>>
//       associated_detections;
//   associated_detections.insert({config, test_utils::fake_detections});
//   const localization::position_estimate_t joint_solver_solution =
//       joint_solver.EstimatePosition(associated_detections,
//                                     square_solver_solution.pose, false);
//   EXPECT_EQ(square_solver_solution, joint_solver_solution);
// }
//
// TEST_F(JointSolverTest, CloseConvergenceOpenRotation) {  // NOLINT
//   const frc::Transform3d joint_solve_input_noise(
//       frc::Translation3d(units::meter_t{0.4}, units::meter_t{0.5},
//                          units::meter_t{0.2}),
//       frc::Rotation3d(units::degree_t{test_utils::deg2rad(3)},
//                       units::degree_t{test_utils::deg2rad(3)},
//                       units::degree_t{test_utils::deg2rad(3)}));
//   const localization::position_estimate_t square_solver_solution =
//       square_solver.EstimatePosition(test_utils::fake_detections, false)[0];
//   frc::Pose3d noisy_pose =
//       square_solver_solution.pose.TransformBy(joint_solve_input_noise);
//   std::map<camera::CameraConstant, std::vector<localization::tag_detection_t>>
//       associated_detections;
//   associated_detections.insert({config, test_utils::fake_detections});
//   const localization::position_estimate_t joint_solver_solution =
//       joint_solver.EstimatePosition(associated_detections, noisy_pose, false,
//                                     true);
//   // std::cout << "sq: " << square_solver_solution
//   //           << "\njoint: " << joint_solver_solution << std::endl;
//   EXPECT_EQ(square_solver_solution, joint_solver_solution);
// }

// TEST_F(JointSolverTest, IncreasingDistanceYawOnly) {  // NOLINT
//   double xDiff = rand();
//   double yDiff = rand();
//   double zDiff = rand();
//   const double translationMax = std::hypot(xDiff, yDiff, zDiff) * 4.0;
//   xDiff /= translationMax;
//   yDiff /= translationMax;
//   zDiff /= translationMax;
//   double rollDiff = rand();
//   double pitchDiff = rand();
//   double yawDiff = rand();
//   const double rotationDivider = std::hypot(rollDiff, pitchDiff, yawDiff) / 4.0;
//   // rollDiff /= rotationDivider;
//   // pitchDiff /= rotationDivider;
//   rollDiff = 0;
//   pitchDiff = 0;
//   yawDiff /= rotationDivider;
//   localization::position_estimate_t square_solver_solution =
//       square_solver.EstimatePosition(test_utils::fake_detections, false)[0];
//   std::map<camera::CameraConstant, std::vector<localization::tag_detection_t>>
//       associated_detections;
//   associated_detections.insert(
//       {camera_constants.at("main_bot_left"), test_utils::fake_detections});
//   localization::position_estimate_t joint_solver_solution;
//   for (int i = 0; i < 15; i++) {
//     const frc::Transform3d joint_solve_input_noise(
//         frc::Translation3d(units::meter_t{i * xDiff}, units::meter_t{i * yDiff},
//                            units::meter_t{i * zDiff}),
//         frc::Rotation3d(units::degree_t{i * rollDiff},
//                         units::degree_t{i * pitchDiff},
//                         units::degree_t{i * yawDiff}));
//
//     frc::Pose3d noisy_pose =
//         square_solver_solution.pose.TransformBy(joint_solve_input_noise);
//     joint_solver_solution = joint_solver.EstimatePosition(
//         associated_detections, noisy_pose, true, false);
//     std::cout << "iter: " << i << " sq: " << square_solver_solution
//               << "\njoint: " << joint_solver_solution << std::endl;
//     std::cout << "Noise" << std::endl;
//     utils::PrintTransform3d(joint_solve_input_noise);
//     if (square_solver_solution != joint_solver_solution) {
//       joint_solver_solution = joint_solver.EstimatePosition(
//           associated_detections, noisy_pose, true, true);
//       std::cout << "Noisy pose" << std::endl;
//       utils::PrintPose3d(noisy_pose);
//       FAIL() << "Failed with above transformation: "
//              << "\nJoint solve solution: " << joint_solver_solution
//              << "\nSq solution: " << square_solver_solution;
//
//       break;
//     }
//   }
// }
//
// TEST_F(JointSolverTest, MaintainsValidEstimateYawOnly) {  // NOLINT
//   const localization::position_estimate_t square_solver_solution =
//       square_solver.EstimatePosition(test_utils::fake_detections, false)[0];
//   std::map<camera::CameraConstant, std::vector<localization::tag_detection_t>>
//       associated_detections;
//   associated_detections.insert({config, test_utils::fake_detections});
//   const localization::position_estimate_t joint_solver_solution =
//       joint_solver.EstimatePosition(associated_detections,
//                                     square_solver_solution.pose, true);
//   // std::cout << "sq: " << square_solver_solution
//   //           << "\njoint: " << joint_solver_solution << std::endl;
//   EXPECT_EQ(square_solver_solution, joint_solver_solution);
// }

TEST_F(JointSolverTest, MaintainsValidEstimateRealImageYawOnly) {  // NOLINT
  const cv::Mat image = cv::imread("/bos/logs/log181/left/11.963395.jpg");
  camera::timestamped_frame_t frame{.frame = image, .timestamp = 0.0};
  const std::vector<localization::tag_detection_t> detections =
      detector.GetTagDetections(frame);
  const localization::position_estimate_t square_solver_solution =
      square_solver.EstimatePosition(detections, false)[0];
  std::map<camera::CameraConstant, std::vector<localization::tag_detection_t>>
      associated_detections;
  associated_detections.insert(
      {camera_constants.at("main_bot_left"), detections});
  const localization::position_estimate_t joint_solver_solution =
      joint_solver.EstimatePosition(associated_detections,
                                    square_solver_solution.pose, false, true);
  // std::cout << "sq: " << square_solver_solution
  //           << "\njoint: " << joint_solver_solution << std::endl;
  EXPECT_EQ(square_solver_solution, joint_solver_solution);
}

// TEST_F(JointSolverTest, CloseConvergenceYawOnly) {  // NOLINT
//   const frc::Transform3d joint_solve_input_noise(
//       frc::Translation3d(units::meter_t{0.4}, units::meter_t{0.5},
//                          units::meter_t{0.2}),
//       frc::Rotation3d(units::degree_t{0}, units::degree_t{0},
//                       units::degree_t{3}));
//   localization::position_estimate_t square_solver_solution =
//       square_solver.EstimatePosition(test_utils::fake_detections, false)[0];
//   frc::Pose3d noisy_pose =
//       square_solver_solution.pose.TransformBy(joint_solve_input_noise);
//   std::map<camera::CameraConstant, std::vector<localization::tag_detection_t>>
//       associated_detections;
//   associated_detections.insert({config, test_utils::fake_detections});
//   const localization::position_estimate_t joint_solver_solution =
//       joint_solver.EstimatePosition(associated_detections, noisy_pose, true);
//   // std::cout << "sq: " << square_solver_solution
//   //           << "\njoint: " << joint_solver_solution << std::endl;
//   EXPECT_EQ(square_solver_solution, joint_solver_solution);
// }
