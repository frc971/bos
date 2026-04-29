#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/transform.h"

using json = nlohmann::json;

namespace localization {

using data_point_t = struct DataPoint {
  Eigen::Vector2d undistorted_point;
  size_t source_index;
  Eigen::Vector4d field_to_tag_corner_homogenous;
};

using joint_estimate_t = struct JointEstimate {
  position_estimate_t pose_estimate;
  double loss;
  bool stuck_in_minimum;
};

struct CameraMatrices {
  Eigen::Matrix<double, 3, 4> image_to_robot;
  frc::Transform3d camera_to_robot;
  cv::Mat distortion_coefficients;
  cv::Mat camera_matrix;
};
static constexpr int kmax_tags = 50;
class JointSolver : public IJointPositionSolver {
 public:
  JointSolver(const std::vector<camera::CameraConstant>& camera_constants_,
              const frc::AprilTagFieldLayout& layout = kapriltag_layout);
  auto Forward(const utils::TransformDecomposition& current_estimate,
               Eigen::Vector4d& Rx_activation, Eigen::Vector4d& Ry_activation,
               Eigen::Vector4d& Rz_activation, Eigen::Vector3d& projection,
               Eigen::Vector2d& projection_error,
               const data_point_t& data_point) -> double;
  auto Forward(const utils::TransformDecomposition& current_estimate,
               std::vector<Eigen::Vector4d>& Rx_activations,
               std::vector<Eigen::Vector4d>& Ry_activations,
               std::vector<Eigen::Vector4d>& Rz_activations,
               std::vector<Eigen::Vector3d>& projections,
               std::vector<Eigen::Vector2d>& projection_errors,
               const std::vector<data_point_t>& data_points) -> double;
  auto ComputeStep(const utils::TransformValues translation_and_rotation,
                   const utils::TransformDecomposition& position_decomposition,
                   const Eigen::Vector4d& Rx_activation,
                   const Eigen::Vector4d& Ry_activation,
                   const Eigen::Vector4d& Rz_activation,
                   const Eigen::Vector3d& projection,
                   const Eigen::Vector2d& projection_error,
                   const data_point_t& data_point, bool yaw_only)
      -> utils::TransformValues;
  auto ComputeNetStep(
      const utils::TransformValues translation_and_rotation,
      const utils::TransformDecomposition& position_decomposition,
      const std::vector<Eigen::Vector4d>& Rx_activations,
      const std::vector<Eigen::Vector4d>& Ry_activations,
      const std::vector<Eigen::Vector4d>& Rz_activations,
      const std::vector<Eigen::Vector3d>& projections,
      const std::vector<Eigen::Vector2d>& projection_errors,
      const std::vector<data_point_t>& data_points, bool yaw_only)
      -> utils::TransformValues;
  void SetStartingPose(const frc::Pose3d& field_to_robot);
  auto EstimatePosition(
      std::vector<std::vector<tag_detection_t>>& all_cam_detections,
      bool reject_far_tags = true)
      -> std::optional<position_estimate_t> override;

 private:
  Eigen::Matrix4d robot_to_field_cv_;
  static constexpr double kacceptable_reprojection_error = 0.2;
  static constexpr double starting_step_size_ = 1e-5;
  static constexpr double kyaw_prioritization = 1e1;
  static constexpr double krotation_step_scalar = 3e-1;
  static constexpr size_t kmax_iters = 1e7;
  std::vector<CameraMatrices> camera_matrices_;
  std::array<std::optional<std::array<Eigen::Vector4d, 4>>, kmax_tags>
      tag_corners_;
  static const Eigen::Matrix4d rotate_yaw_cv_;
  const frc::AprilTagFieldLayout layout_;
  static constexpr double kvariance_scalar_ = 0.5;
  static constexpr double kvariance_min_ = 1.0;
  const bool verbose_ = true;   // TODO remove
  const bool yaw_only_ = true;  // TODO remove
};
}  // namespace localization
