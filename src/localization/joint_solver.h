#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"

using json = nlohmann::json;

namespace localization {
struct CameraMatrices {
  Eigen::Matrix<double, 3, 4> image_to_robot;
  cv::Mat distortion_coefficients;
  cv::Mat camera_matrix;
};
static constexpr int kmax_tags = 50;
class JointSolver {
 public:
  JointSolver(const std::vector<camera::camera_constant_t>& camera_constants_,
              const frc::AprilTagFieldLayout& layout = kapriltag_layout);
  auto EstimatePosition(
      const std::map<std::string, std::vector<tag_detection_t>>&
          all_cam_detections,
      const frc::Pose3d& starting_pose) -> position_estimate_t;
  Eigen::Matrix4d robot_to_field_;

 private:
  static constexpr double kacceptable_reprojection_error = 0.005;
  std::map<std::string, CameraMatrices> camera_matrices_;
  std::array<std::optional<std::array<Eigen::Vector4d, 4>>, kmax_tags>
      tag_corners_;
};
}  // namespace localization
