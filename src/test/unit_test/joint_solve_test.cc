#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "src/camera/camera_constants.h"
#include "src/localization/joint_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/transform.h"

constexpr double ktag_size = 0.1651;

auto GetTagCorners(double tag_size) -> std::vector<cv::Point3f> {
  double half_size = tag_size / 2.0;
  return {
      cv::Point3f(-half_size, half_size, 0.0f),  // top-left
      cv::Point3f(half_size, half_size, 0.0f),   // top-right
      cv::Point3f(half_size, -half_size, 0.0f),  // bottom-right
      cv::Point3f(-half_size, -half_size, 0.0f)  // bottom-left
  };
}

TEST(LocalizationTest, AverageSquareSolveVsJointSolve) {
  camera::Camera config = camera::Camera::DUMMY_CAMERA;

  std::vector<cv::Point3f> tag_corners = GetTagCorners(ktag_size);

  localization::SquareSolver square_solver(config);
  LOG(INFO) << "init square solver";

  localization::JointSolver joint_solver(std::vector<camera::Camera>{config});
  LOG(INFO) << "init joint solver";

  std::vector<localization::tag_detection_t> tag_detections;

  localization::tag_detection_t detection1;
  detection1.tag_id = 1;
  detection1.timestamp = 0.0;
  detection1.corners = {
      cv::Point2f(100.0f, 100.0f), cv::Point2f(200.0f, 100.0f),
      cv::Point2f(200.0f, 200.0f), cv::Point2f(100.0f, 200.0f)};
  tag_detections.push_back(detection1);

  utils::PrintTransformationMatrix(square_solver.EstimatePosition(detection1));
  std::cout << square_solver.EstimatePosition(tag_detections)[0] << std::endl;
}
