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
  JointSolver(const std::vector<camera::camera_constant_t>& camera_constants_,
              const frc::AprilTagFieldLayout& layout = kapriltag_layout);
  auto EstimatePosition(
      const std::vector<std::vector<tag_detection_t>>& all_cam_detections_)
      -> position_estimate_t;

 private:
  std::map<camera::camera_constant_t, Eigen::Matrix<double, 3, 4>>
      image_to_camera_;  // per camera
  std::map<camera::camera_constant_t, Eigen::Matrix4d> camera_to_robot_;
  std::array<std::optional<Eigen::Matrix4d>, kmax_tags> tag_poses_;
};
}  // namespace localization
