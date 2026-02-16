#include "src/localization/joint_solver.h"
#include <opencv2/calib3d.hpp>
#include <utility>
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"

struct Detection {
  const camera::Camera camera;
  Eigen::Vector2d image_point;
  Eigen::Vector4d field_relative_tag_corner;
};

namespace localization {
using frc::AprilTagFieldLayout;

JointSolver::JointSolver(const std::vector<camera::Camera>& camera_constants_,
                         const AprilTagFieldLayout& layout) {
  for (const frc::AprilTag& tag : layout.GetTags()) {
    tag_poses_[tag.ID] = tag.pose.ToMatrix();
  }
  Eigen::Matrix<double, 3, 4> pi = Eigen::Matrix<double, 3, 4>::Zero();
  pi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  for (auto& camera_config : camera_constants_) {
    image_to_robot_.insert(
        {camera_config,
         utils::CameraMatrixFromJson<Eigen::Matrix3d>(utils::ReadIntrinsics(
             camera::camera_constants[camera_config].intrinsics_path)) *
             pi *
             utils::ExtrinsicsJsonToCameraToRobot(
                 camera::camera_constants[camera_config].extrinsics_path)
                 .ToMatrix()});  // im_to_cam * 2dcam_to_3dcam * cam2robot
  }
}

auto JointSolver::EstimatePosition(
    const std::map<camera::Camera, std::vector<tag_detection_t>>&
        all_cam_detections) -> position_estimate_t {
  if (all_cam_detections.empty()) {
    return {};
  }
  field_to_robot_ = Eigen::Matrix4d::Identity();
  std::vector<Detection> detections;
  for (const auto& pair : all_cam_detections) {
    for (const tag_detection_t& detection : pair.second) {
      if (!tag_poses_[detection.tag_id]) {
        continue;
      }
      for (size_t i = 0; i < detection.corners.size(); i++) {
        Eigen::Vector2d image_point_normalized;
        image_point_normalized << detection.corners[i].x,
            detection.corners[i].y, 0;
        image_point_normalized /= image_point_normalized.maxCoeff();
        detections.push_back(
            Detection{.camera = pair.first,
                      .image_point = image_point_normalized,
                      .field_relative_tag_corner =
                          tag_poses_[detection.tag_id].value() *
                          (Eigen::Vector4d() << kapriltag_corners_eigen[i], 1)
                              .finished()});
      }
    }
  }
  double error = INFINITY;
  while (error > kacceptable_reprojection_error) {
    for (const Detection& detection : detections) {
      Eigen::Vector3d projection = image_to_robot_.at(detection.camera) *
                                   field_to_robot_ *
                                   detection.field_relative_tag_corner;
      const double scale_factor = projection.maxCoeff();
      projection /= scale_factor;
      const double MSE_derivative =
          std::hypot(projection[0] - detection.image_point[0],
                     projection[1] - detection.image_point[1]);
      std::cout << MSE_derivative << std::endl;
    }
  }

  return {};
}

}  // namespace localization
