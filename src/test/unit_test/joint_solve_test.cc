#include <gtest/gtest.h>
#include "src/localization/joint_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

using camera::Camera;

const auto config = camera::Camera::DEV_ORIN;

std::vector<localization::tag_detection_t> detections(
    {{.tag_id = 31,
      .corners = {cv::Point2d(100.0, 200.0), cv::Point2d(200.0, 200.0),
                  cv::Point2d(200.0, 100.0), cv::Point2d(100.0, 100.0)},
      .timestamp = 0.0,
      .confidence = 0.0}});

TEST(JointSolveTest, EstimatePosition) {  // NOLINT
  localization::JointSolver joint_solver({config});
  localization::SquareSolver square_solver(config);

  auto square_solver_solution =
      square_solver.EstimatePosition(detections, false)[0];
  utils::PrintTransformationMatrix(
      utils::EigenToCvMat(square_solver_solution.pose.ToMatrix()));
  std::cout << square_solver_solution.pose.ToMatrix() << std::endl;

  frc::Transform3d noise(
      frc::Translation3d(units::meter_t{0.4}, units::meter_t{0.3},
                         units::meter_t{-0.1}),
      frc::Rotation3d(units::degree_t{1.0}, units::degree_t{1.0},
                      units::degree_t{1.0}));
  std::cout << "Square solver solution: " << square_solver_solution
            << std::endl;

  frc::Pose3d noisy_pose = square_solver_solution.pose.TransformBy(noise);

  std::map<camera::Camera, std::vector<localization::tag_detection_t>>
      associated_detections;
  associated_detections.insert({config, detections});
  auto joint_solver_solution =
      joint_solver.EstimatePosition(associated_detections, noisy_pose);
  std::cout << "Joint solver solution: " << joint_solver_solution << std::endl;
}
