#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"

using json = nlohmann::json;

namespace localization {
static constexpr int kmax_tags = 50;
class JointSolver {
 public:
  JointSolver(const std::vector<camera::Camera>& camera_constants_,
              const frc::AprilTagFieldLayout& layout = kapriltag_layout);
  auto EstimatePosition(
      const std::map<camera::Camera, std::vector<tag_detection_t>>&
          all_cam_detections) -> position_estimate_t;
  Eigen::Matrix4d field_to_robot_;

 private:
  static constexpr double kacceptable_reprojection_error = 0.005;
  std::map<camera::Camera, Eigen::Matrix<double, 3, 4>> robot_to_image_;
  std::array<std::optional<Eigen::Matrix4d>, kmax_tags> tag_poses_;
};
}  // namespace localization
