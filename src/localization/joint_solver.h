#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"

using json = nlohmann::json;

namespace localization {
using data_point_t = struct DataPoint {
  Eigen::Vector2d undistorted_point;
  size_t source_index;
  Eigen::Vector4d field_to_tag_corner_homogeneous_cv;
};
struct CameraMatrices {
  Eigen::Matrix<double, 3, 4> image_to_robot;
  cv::Mat distortion_coefficients;
  cv::Mat camera_matrix;
};
static constexpr int kmax_tags = 50;
class JointSolver : IJointPositionSolver {
 public:
  JointSolver(const std::vector<camera::camera_constant_t>& camera_constants_);
  auto EstimatePosition(
      std::vector<std::vector<tag_detection_t>>& detection_batches,
      bool reject_far_tags = true)
      -> std::optional<position_estimate_t> override;
  void SetStartPosition(const frc::Pose3d& pose);
  void ComputeResidual(const std::vector<data_point_t>& data_points,
                       const Eigen::Matrix4d& robot_to_field,
                       Eigen::VectorXd& residual,
                       Eigen::MatrixXd* d_residual_d_twist_jacobian = nullptr);

 private:
  static constexpr double kacceptable_reprojection_error = 0.005;
  static constexpr double kmaximum_lambda = 1e10;
  std::vector<CameraMatrices> camera_matrices_;
  std::array<std::optional<std::array<Eigen::Vector4d, 4>>, kmax_tags>
      tag_corners_;
  Eigen::Matrix4d robot_to_field_;
};
}  // namespace localization
