#include <gtest/gtest.h>
#include "src/localization/joint_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"

using camera::Camera;

const auto config = camera::Camera::DEV_ORIN;

std::vector<localization::tag_detection_t> detections(
    {{.tag_id = 25,
      .corners = {cv::Point2d(100.0, 200.0), cv::Point2d(200.0, 200.0),
                  cv::Point2d(200.0, 100.0), cv::Point2d(100.0, 100.0)},
      .timestamp = 0.0,
      .confidence = 0.0}});

TEST(JointSolveTest, EstimatePosition) {  // NOLINT
  localization::JointSolver joint_solver({config});
  localization::SquareSolver square_solver(config);

  auto square_solver_solution = square_solver.EstimatePosition(detections)[0];

  frc::Transform3d noise(
      frc::Translation3d(units::meter_t{0.067}, units::meter_t{0.1},
                         units::meter_t{-0.1}),
      {});

  square_solver_solution.pose = square_solver_solution.pose.TransformBy(noise);

  std::map<camera::Camera, std::vector<localization::tag_detection_t>>
      associated_detections;
  associated_detections.insert({config, detections});
  auto joint_solver_solution = joint_solver.EstimatePosition(
      associated_detections, square_solver_solution.pose);
  std::cout << joint_solver_solution << std::endl;
}
