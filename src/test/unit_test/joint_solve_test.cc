#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "src/camera/camera_constants.h"
#include "src/localization/joint_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/transform.h"

TEST(LocalizationTest, AverageSquareSolveVsJointSolve) {
  camera::Camera config = camera::Camera::DUMMY_CAMERA;

  localization::SquareSolver square_solver(config);
  LOG(INFO) << "init square solver";

  localization::JointSolver joint_solver(std::vector<camera::Camera>{config});
  LOG(INFO) << "init joint solver";

  std::vector<localization::tag_detection_t> tag_detections;

  localization::tag_detection_t detection1;
  detection1.tag_id = 31;
  detection1.timestamp = 0.0;
  detection1.corners = {cv::Point2f(5.0f, 5.0f), cv::Point2f(15.0f, 5.0f),
                        cv::Point2f(5.0f, 15.0f), cv::Point2f(15.0f, 15.0f)};
  tag_detections.push_back(detection1);

  utils::PrintTransformationMatrix(square_solver.EstimatePosition(detection1));
  localization::position_estimate_t square_estimate =
      square_solver.EstimatePosition(tag_detections)[0];
  cv::Mat square_estimate2 = square_solver.EstimatePosition(detection1);
  std::cout << square_estimate << std::endl;

  std::map<camera::Camera, std::vector<localization::tag_detection_t>>
      associated_detections;
  associated_detections.insert(
      {camera::Camera::DUMMY_CAMERA,
       std::vector<localization::tag_detection_t>{detection1}});
  Eigen::Matrix4d square_estimate_mat = square_estimate.pose.ToMatrix();
  utils::ChangeBasis(square_estimate_mat, utils::WPI_TO_CV);
  joint_solver.robot_to_field_ = square_estimate_mat.inverse();
  joint_solver.EstimatePosition(associated_detections);
}
