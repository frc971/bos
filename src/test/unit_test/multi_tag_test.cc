#include <gtest/gtest.h>
#include "src/localization/multi_tag_solver.h"
#include "src/localization/square_solver.h"

using camera::Camera;

TEST(MultiTagTest, EqualToSquareSolve) {  // NOLINT
  auto config = camera::Camera::DEV_ORIN;

  std::vector<localization::tag_detection_t> detections(
      {{.tag_id = 25,
        .corners = {cv::Point2d(100.0f, 200.0f), cv::Point2d(200.0f, 200.0f),
                    cv::Point2d(200.0f, 100.0f), cv::Point2d(100.0f, 100.0f)},
        .timestamp = 0.0,
        .confidence = 0.0}});

  localization::SquareSolver square_solver(config);
  localization::MultiTagSolver multi_tag_solver(config);

  auto square_solver_solution = square_solver.EstimatePosition(detections)[0];
  auto multi_tag_solution = square_solver.EstimatePosition(detections)[0];

  ASSERT_TRUE(square_solver_solution.pose.ToMatrix() ==
              multi_tag_solution.pose.ToMatrix());
}
