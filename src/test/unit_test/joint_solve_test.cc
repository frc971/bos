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
      std::vector<camera::CameraConstant>{
          camera_constants.at("main_bot_left"),
          camera_constants.at("main_bot_right")};
  localization::SquareSolver left_square_solver;
  localization::SquareSolver right_square_solver;
  localization::JointSolver joint_solver;
  localization::OpenCVAprilTagDetector left_detector;
  localization::OpenCVAprilTagDetector right_detector;
  JointSolverTest()
      : left_square_solver(camera_constants.at("main_bot_left")),
        right_square_solver(camera_constants.at("main_bot_right")),
        joint_solver(joint_solve_cameras),
        left_detector(utils::ReadIntrinsics(
            camera_constants.at("main_bot_left").intrinsics_path.value())),
        right_detector(utils::ReadIntrinsics(
            camera_constants.at("main_bot_right").intrinsics_path.value())) {}
};

TEST_F(JointSolverTest, MaintainsValidEstimateRealImageYawOnly) {  // NOLINT
  const cv::Mat left_image =
      cv::imread("/bos/bos-logs/log181/main_bot_left/5.611357.jpg");
  const cv::Mat right_image =
      cv::imread("/bos/bos-logs/log181/main_bot_right/5.647740.jpg");
  camera::timestamped_frame_t left_frame{.frame = left_image, .timestamp = 0.0};
  camera::timestamped_frame_t right_frame{.frame = right_image,
                                          .timestamp = 0.0};
  const std::vector<localization::tag_detection_t> left_detections =
      left_detector.GetTagDetections(left_frame);
  const std::vector<localization::tag_detection_t> right_detections =
      right_detector.GetTagDetections(right_frame);
  CHECK_NE(left_detections.size() + right_detections.size(), 0);
  const std::vector<localization::position_estimate_t>
      left_square_solver_solutions =
          left_square_solver.EstimatePosition(left_detections, false);
  const std::optional<localization::position_estimate_t>
      left_square_solver_solution =
          left_square_solver_solutions.size() == 0
              ? std::nullopt
              : std::make_optional<localization::position_estimate_t>(
                    left_square_solver_solutions[0]);
  const std::vector<localization::position_estimate_t>
      right_square_solver_solutions =
          right_square_solver.EstimatePosition(right_detections, false);
  const std::optional<localization::position_estimate_t>
      right_square_solver_solution =
          right_square_solver_solutions.size() == 0
              ? std::nullopt
              : std::make_optional<localization::position_estimate_t>(
                    right_square_solver_solutions[0]);
  std::vector<std::vector<localization::tag_detection_t>> associated_detections{
      left_detections, right_detections};
  const frc::Transform3d joint_solve_input_noise(
      frc::Translation3d(units::meter_t{0.1}, units::meter_t{0.1},
                         units::meter_t{0.1}),
      frc::Rotation3d(units::degree_t{0}, units::degree_t{0},
                      units::degree_t{0}));
  joint_solver.SetStartPosition(
      (left_square_solver_solution.has_value() ? left_square_solver_solution
                                               : right_square_solver_solution)
          ->pose.TransformBy(joint_solve_input_noise));
  const localization::position_estimate_t joint_solver_solution =
      joint_solver.EstimatePosition(associated_detections, false).value();
  if (left_square_solver_solution) {
    std::cout << "sq left: " << left_square_solver_solution.value();
  }
  if (right_square_solver_solution) {
    std::cout << "\nsq right: " << right_square_solver_solution.value();
  }
  std::cout << "\njoint: " << joint_solver_solution << std::endl;
  std::cout << "loss: " << joint_solver_solution.loss << std::endl;
  std::cout << "numtags: " << joint_solver_solution.num_tags << std::endl;
}
