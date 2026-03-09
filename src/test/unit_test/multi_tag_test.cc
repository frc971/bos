#include <gtest/gtest.h>
#include "src/localization/multi_tag_solver.h"
#include "src/localization/square_solver.h"

TEST(MultiTagTest, EqualToSquareSolve) {  // NOLINT
  // camera::camera_constant_t camera_constant =
  //     camera::GetCameraConstants("/bos/constants/camera_constants.json")
  //         .at("dev_orin");

  auto a = camera::GetCameraConstants("/bos/constants/camera_constants.json");

  // std::vector<localization::tag_detection_t> detections(
  //     {{.tag_id = 25,
  //       .corners = {cv::Point2d(100.0f, 100.0f), cv::Point2d(200.0f, 100.0f),
  //                   cv::Point2d(200.0f, 200.0f), cv::Point2d(100.0f, 200.0f)},
  //       .timestamp = 0.0,
  //       .confidence = 0.0}});
  //
  // localization::SquareSolver square_solver(camera_constant);
  // localization::MultiTagSolver multi_tag_solver(camera_constant);
  //
  // auto square_solver_solution = square_solver.EstimatePosition(detections)[0];
  // auto multi_tag_solution = square_solver.EstimatePosition(detections)[0];
  //
  // ASSERT_TRUE(square_solver_solution.pose.ToMatrix() ==
  //             multi_tag_solution.pose.ToMatrix());
}
