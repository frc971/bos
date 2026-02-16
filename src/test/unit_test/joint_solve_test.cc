#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "src/camera/camera_constants.h"
#include "src/localization/joint_solver.h"
#include "src/localization/square_solver.h"

constexpr double ktag_size = 0.1651;
const frc::AprilTagFieldLayout kapriltag_layout =
    frc::AprilTagFieldLayout("/bos/constants/2026-rebuilt-andymark.json");

auto GetTagCorners(double tag_size) -> std::vector<cv::Point3f> {
  double half_size = tag_size / 2.0;
  return {
      cv::Point3f(-half_size, half_size, 0.0f),  // top-left
      cv::Point3f(half_size, half_size, 0.0f),   // top-right
      cv::Point3f(half_size, -half_size, 0.0f),  // bottom-right
      cv::Point3f(-half_size, -half_size, 0.0f)  // bottom-left
  };
}

Eigen::Matrix4d TransformationMatrix(const double rx, const double ry,
                                     const double rz, const double x,
                                     const double y, const double z) {
  Eigen::Matrix3d roll_mat;
  roll_mat << std::cos(rz), -std::sin(rz), 0.0, std::sin(rz), std::cos(rz), 0.0,
      0.0, 0.0, 1.0;

  Eigen::Matrix3d pitch_mat;
  pitch_mat << std::cos(rx), 0.0, std::sin(rx), 0.0, 1.0, 0.0, -std::sin(rx),
      0.0, std::cos(rx);

  Eigen::Matrix3d yaw_mat;
  yaw_mat << 1.0, 0.0, 0.0, 0.0, std::cos(ry), -std::sin(ry), 0.0, std::sin(ry),
      std::cos(ry);

  const Eigen::Matrix3d R = roll_mat * pitch_mat * yaw_mat;

  Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
  A.block<3, 3>(0, 0) = R;
  A(0, 3) = x;
  A(1, 3) = y;
  A(2, 3) = z;

  return A;
}

auto wpi2cv(const Eigen::Matrix4d& wpi) -> Eigen::Matrix4d {
  const Eigen::Matrix3d wpi2cv =
      (Eigen::Matrix3d() << 0, -1, 0, 0, 0, -1, 1, 0, 0).finished();
  Eigen::Matrix4d wpi2cv4 = Eigen::Matrix4d::Identity();
  wpi2cv4.block<3, 3>(0, 0) = wpi2cv;
  auto result = wpi2cv4 * wpi * wpi2cv4.inverse();
  return result;
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

  std::cout << square_solver.EstimatePosition(tag_detections)[0] << std::endl;

  // Eigen::Matrix4d res1 = square_solver.EstimatePosition(detection1);
  // wpi2cv
}
