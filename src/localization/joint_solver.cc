#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"

namespace localization {
using frc::AprilTagFieldLayout;

JointSolver::JointSolver(
    const std::vector<camera::camera_constant_t>& camera_constants_,
    const AprilTagFieldLayout& layout) {
  for (const frc::AprilTag& tag : layout.GetTags()) {
    tag_poses_[tag.ID] = tag.pose.ToMatrix();
  }
  Eigen::Matrix<double, 3, 4> pi = Eigen::Matrix<double, 3, 4>::Zero();
  pi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  for (auto& camera_config : camera_constants_) {
    image_to_camera_.insert(
        {camera_config,
         utils::camera_matrix_from_json<Eigen::Matrix3d>(
             utils::read_intrinsics(camera_config.intrinsics_path)) *
             pi});
    camera_to_robot_.insert(
        {camera_config,
         utils::ExtrinsicsJsonToCameraToRobot(camera_config.extrinsics_path)
             .ToMatrix()});
  }
}

auto JointSolver::EstimatePosition(
    const std::vector<std::vector<tag_detection_t>>& all_cam_detections)
    -> position_estimate_t {
  if (all_cam_detections.empty()) {
    return {};
  }
  Eigen::Matrix4d field_to_robot;
  std::vector<Eigen::Vector4d> corners;
  for (const std::vector<tag_detection_t>& detections : all_cam_detections) {
    for (const tag_detection_t& detection : detections) {
      if (!tag_poses_[detection.tag_id]) {
        continue;
      }
      for (const cv::Point2f& image_corner : detection.corners) {
        corners.emplace_back(
            tag_poses_[detection.tag_id].value() *
            (Eigen::Vector4d() << image_corner.x, image_corner.y, 0, 1)
                .finished());
      }
    }
  }

  return {};
}

}  // namespace localization
