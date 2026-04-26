#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"
#include "src/utils/transform.h"

namespace localization {

using frc::AprilTagFieldLayout;

JointSolver::JointSolver(
    const std::vector<camera::camera_constant_t>& camera_constants_,
    const AprilTagFieldLayout& layout)
    : robot_to_field_(Eigen::Matrix4d::Identity()) {
  // clang-format off
}

auto JointSolver::EstimatePosition(
    const std::map<std::string, std::vector<tag_detection_t>>&
        all_cam_detections,
    const frc::Pose3d& starting_pose) -> position_estimate_t {
  return {};
}

}  // namespace localization
