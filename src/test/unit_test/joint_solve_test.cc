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
  static constexpr camera::Camera config = camera::DEV_ORIN;
  localization::SquareSolver square_solver;
  localization::JointSolver joint_solver;
  localization::OpenCVAprilTagDetector detector;
  JointSolverTest()
      : square_solver(config),
        joint_solver(test_utils::joint_solve_cameras),
        detector(utils::ReadIntrinsics(
            camera::camera_constants[config].intrinsics_path)) {}
};

// TEST_F(JointSolverTest, MaintainsValidEstimateOpenRotation) {  // NOLINT
//   const localization::position_estimate_t square_solver_solution =
//       square_solver.EstimatePosition(test_utils::fake_detections, false)[0];
//   std::map<camera::Camera, std::vector<localization::tag_detection_t>>
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
//   std::map<camera::Camera, std::vector<localization::tag_detection_t>>
//       associated_detections;
//   associated_detections.insert({config, test_utils::fake_detections});
//   const localization::position_estimate_t joint_solver_solution =
//       joint_solver.EstimatePosition(associated_detections, noisy_pose, false,
//                                     true);
//   // std::cout << "sq: " << square_solver_solution
//   //           << "\njoint: " << joint_solver_solution << std::endl;
//   EXPECT_EQ(square_solver_solution, joint_solver_solution);
// }

TEST_F(JointSolverTest, IncreasingDistanceYawOnly) {  // NOLINT
  double xDiff = rand();
  double yDiff = rand();
  double zDiff = rand();
  const double translationMax = std::hypot(xDiff, yDiff, zDiff);
  xDiff /= translationMax;
  yDiff /= translationMax;
  zDiff /= translationMax;
  double rollDiff = rand();
  double pitchDiff = rand();
  double yawDiff = rand();
  const double rotationDivider = std::hypot(rollDiff, pitchDiff, yawDiff) * 2.0;
  rollDiff /= rotationDivider;
  pitchDiff /= rotationDivider;
  yawDiff /= rotationDivider;
  localization::position_estimate_t square_solver_solution;
  localization::position_estimate_t joint_solver_solution;
  for (int i = 0; i < 10; i++) {
    const frc::Transform3d joint_solve_input_noise(
        frc::Translation3d(units::meter_t{i * xDiff}, units::meter_t{i * yDiff},
                           units::meter_t{i * zDiff}),
        frc::Rotation3d(units::degree_t{test_utils::deg2rad(i * rollDiff)},
                        units::degree_t{test_utils::deg2rad(i * pitchDiff)},
                        units::degree_t{test_utils::deg2rad(i * yawDiff)}));

    square_solver_solution =
        square_solver.EstimatePosition(test_utils::fake_detections, false)[0];
    frc::Pose3d noisy_pose =
        square_solver_solution.pose.TransformBy(joint_solve_input_noise);
    std::map<camera::Camera, std::vector<localization::tag_detection_t>>
        associated_detections;
    associated_detections.insert({config, test_utils::fake_detections});
    joint_solver_solution =
        joint_solver.EstimatePosition(associated_detections, noisy_pose, true);
    std::cout << "iter: " << i << " sq: " << square_solver_solution
              << "\njoint: " << joint_solver_solution << std::endl;
    if (square_solver_solution != joint_solver_solution) {
      utils::PrintTransform3d(joint_solve_input_noise);
      joint_solver.EstimatePosition(associated_detections, noisy_pose, true,
                                    true);
      FAIL() << "Failed with above transformation: "
             << "\nJoint solve solution: " << joint_solver_solution
             << "\nSq solution: " << square_solver_solution;

      break;
    }
  }
}
//
// TEST_F(JointSolverTest, MaintainsValidEstimateYawOnly) {  // NOLINT
//   const localization::position_estimate_t square_solver_solution =
//       square_solver.EstimatePosition(test_utils::fake_detections, false)[0];
//   std::map<camera::Camera, std::vector<localization::tag_detection_t>>
//       associated_detections;
//   associated_detections.insert({config, test_utils::fake_detections});
//   const localization::position_estimate_t joint_solver_solution =
//       joint_solver.EstimatePosition(associated_detections,
//                                     square_solver_solution.pose, true);
//   // std::cout << "sq: " << square_solver_solution
//   //           << "\njoint: " << joint_solver_solution << std::endl;
//   EXPECT_EQ(square_solver_solution, joint_solver_solution);
// }
//
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
//   std::map<camera::Camera, std::vector<localization::tag_detection_t>>
//       associated_detections;
//   associated_detections.insert({config, test_utils::fake_detections});
//   const localization::position_estimate_t joint_solver_solution =
//       joint_solver.EstimatePosition(associated_detections, noisy_pose, true);
//   // std::cout << "sq: " << square_solver_solution
//   //           << "\njoint: " << joint_solver_solution << std::endl;
//   EXPECT_EQ(square_solver_solution, joint_solver_solution);
// }
