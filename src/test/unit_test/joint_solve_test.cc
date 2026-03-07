#include <gtest/gtest.h>
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

TEST_F(JointSolverTest, MaintainsValidEstimate) {  // NOLINT
  const localization::position_estimate_t square_solver_solution =
      square_solver.EstimatePosition(test_utils::fake_detections, false)[0];
  std::map<camera::Camera, std::vector<localization::tag_detection_t>>
      associated_detections;
  associated_detections.insert({config, test_utils::fake_detections});
  const localization::position_estimate_t joint_solver_solution =
      joint_solver.EstimatePosition(associated_detections,
                                    square_solver_solution.pose);
  EXPECT_EQ(square_solver_solution, joint_solver_solution);
}

TEST_F(JointSolverTest, CloseConvergence) {  // NOLINT
  static const frc::Transform3d joint_solve_input_noise(
      frc::Translation3d(units::meter_t{0.4}, units::meter_t{0.5},
                         units::meter_t{0.2}),
      frc::Rotation3d(units::degree_t{test_utils::deg2rad(3)},
                      units::degree_t{test_utils::deg2rad(3)},
                      units::degree_t{test_utils::deg2rad(3)}));
  const localization::position_estimate_t square_solver_solution =
      square_solver.EstimatePosition(test_utils::fake_detections, false)[0];
  frc::Pose3d noisy_pose =
      square_solver_solution.pose.TransformBy(joint_solve_input_noise);
  std::map<camera::Camera, std::vector<localization::tag_detection_t>>
      associated_detections;
  associated_detections.insert({config, test_utils::fake_detections});
  const localization::position_estimate_t joint_solver_solution =
      joint_solver.EstimatePosition(associated_detections, noisy_pose);
  EXPECT_EQ(square_solver_solution, joint_solver_solution);
}
