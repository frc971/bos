#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"

struct Detection {
  camera::camera_constant_t& camera;
  Eigen::Vector3d image_point;
  Eigen::Vector4d field_relative_tag_corner;
};

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
    image_to_robot_.insert(
        {camera_config,
         utils::camera_matrix_from_json<Eigen::Matrix3d>(
             utils::read_intrinsics(camera_config.intrinsics_path)) *
             pi *
             utils::ExtrinsicsJsonToCameraToRobot(camera_config.extrinsics_path)
                 .ToMatrix()});  // im_to_cam * 2dcam_to_3dcam * cam2robot
  }
}

auto JointSolver::EstimatePosition(
    const std::map<camera::camera_constant_t, std::vector<tag_detection_t>>&
        all_cam_detections) -> position_estimate_t {
  if (all_cam_detections.empty()) {
    return {};
  }
  Eigen::Matrix4d field_to_robot = Eigen::Matrix4d::Identity();
  std::map<camera::camera_constant_t, std::vector<Eigen::Vector4d>>
      field_relative_corners;
  for (const auto& pair : all_cam_detections) {
    std::vector<Eigen::Vector4d> homogenized_relative_corners;
    homogenized_relative_corners.reserve(4 * pair.second.size());
    for (const tag_detection_t& detection : pair.second) {
      if (!tag_poses_[detection.tag_id]) {
        continue;
      }
      for (const cv::Point2f& image_corner : detection.corners) {
        homogenized_relative_corners.emplace_back(
            tag_poses_[detection.tag_id].value() *
            (Eigen::Vector4d() << image_corner.x, image_corner.y, 0, 1)
                .finished());
      }
    }
    field_relative_corners.insert({pair.first, homogenized_relative_corners});
  }
  double error = INFINITY;
  while (error > kacceptable_reprojection_error) {
    for (const auto& pair : field_relative_corners) {
      for (const Eigen::Vector4d& field_relative_tag_corner : pair.second) {
        Eigen::Vector3d projection = image_to_robot_.at(pair.first) *
                                     field_to_robot * field_relative_tag_corner;
      }
    }
  }

  return {};
}

}  // namespace localization
