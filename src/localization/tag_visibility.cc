#include "src/localization/tag_visibility.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>

#include "src/localization/position_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/constants_from_json.h"

namespace localization {
namespace {

auto PolygonArea(const std::array<cv::Point2d, 4>& corners) -> double {
  double twice_area = 0.0;
  for (size_t i = 0; i < corners.size(); ++i) {
    const cv::Point2d& current = corners[i];
    const cv::Point2d& next = corners[(i + 1) % corners.size()];
    twice_area += current.x * next.y - current.y * next.x;
  }
  return std::abs(twice_area) / 2.0;
}

auto IsInsideImage(const cv::Point2d& point,
                   const VisibilityCameraModel& camera) -> bool {
  return point.x >= 0.0 && point.x < camera.image_width && point.y >= 0.0 &&
         point.y < camera.image_height;
}

}  // namespace

auto MakeVisibilityCameraModel(const camera::camera_constant_t& camera_constant)
    -> VisibilityCameraModel {
  if (!camera_constant.intrinsics_path || !camera_constant.extrinsics_path ||
      !camera_constant.frame_width || !camera_constant.frame_height) {
    throw std::invalid_argument("Camera " + camera_constant.name +
                                " is missing calibration or image dimensions");
  }

  const nlohmann::json intrinsics =
      utils::ReadIntrinsics(*camera_constant.intrinsics_path);
  // The existing helper returns camera-to-robot; the simulator needs the pose
  // of the camera in the robot frame.
  const frc::Transform3d robot_to_camera =
      utils::ExtrinsicsJsonToCameraToRobot(
          utils::ReadExtrinsics(*camera_constant.extrinsics_path))
          .Inverse();

  return {.name = camera_constant.name,
          .image_width = static_cast<int>(*camera_constant.frame_width),
          .image_height = static_cast<int>(*camera_constant.frame_height),
          .fx = intrinsics.at("fx"),
          .fy = intrinsics.at("fy"),
          .cx = intrinsics.at("cx"),
          .cy = intrinsics.at("cy"),
          .robot_to_camera = robot_to_camera};
}

auto GetVisibleTags(const frc::Pose3d& robot_pose,
                    const VisibilityCameraModel& camera,
                    const std::vector<frc::AprilTag>& tags)
    -> std::vector<VisibleTag> {
  const Eigen::Matrix4d camera_to_field =
      robot_pose.TransformBy(camera.robot_to_camera).ToMatrix();
  const Eigen::Matrix4d field_to_camera = camera_to_field.inverse();
  const Eigen::Vector3d camera_position = camera_to_field.block<3, 1>(0, 3);

  constexpr double kMinimumDepthMeters = 1e-6;
  const double half_tag = ktag_size / 2.0;
  // AprilTag poses use +x as the outward face normal, so the tag lies in its
  // local yz plane.
  const std::array<Eigen::Vector4d, 4> local_corners = {
      Eigen::Vector4d{0.0, half_tag, half_tag, 1.0},
      Eigen::Vector4d{0.0, -half_tag, half_tag, 1.0},
      Eigen::Vector4d{0.0, -half_tag, -half_tag, 1.0},
      Eigen::Vector4d{0.0, half_tag, -half_tag, 1.0}};

  std::vector<VisibleTag> visible_tags;
  visible_tags.reserve(tags.size());
  for (const frc::AprilTag& tag : tags) {
    const Eigen::Matrix4d tag_to_field = tag.pose.ToMatrix();
    const Eigen::Vector3d tag_position = tag_to_field.block<3, 1>(0, 3);
    const Eigen::Vector3d tag_normal = tag_to_field.block<3, 1>(0, 0);
    const double distance = (tag_position - camera_position).norm();

    // A camera must be on the front side of the one-sided AprilTag target.
    if (tag_normal.dot(camera_position - tag_position) <= 0.0 ||
        distance > camera.maximum_tag_distance_meters) {
      continue;
    }

    std::array<cv::Point2d, 4> image_corners;
    bool entirely_in_frame = true;
    for (size_t i = 0; i < local_corners.size(); ++i) {
      const Eigen::Vector4d camera_corner =
          field_to_camera * tag_to_field * local_corners[i];
      const double depth = camera_corner.x();
      if (depth <= kMinimumDepthMeters) {
        entirely_in_frame = false;
        break;
      }

      // Convert WPILib camera axes (x forward, y left, z up) to image axes
      // (u right, v down).
      image_corners[i] =
          cv::Point2d{camera.cx - camera.fx * camera_corner.y() / depth,
                      camera.cy - camera.fy * camera_corner.z() / depth};
      if (!IsInsideImage(image_corners[i], camera)) {
        entirely_in_frame = false;
        break;
      }
    }

    if (!entirely_in_frame) {
      continue;
    }
    const double area = PolygonArea(image_corners);
    if (area < camera.minimum_tag_area_pixels) {
      continue;
    }

    visible_tags.push_back({.id = tag.ID,
                            .pose = tag.pose,
                            .image_corners = image_corners,
                            .distance_meters = distance,
                            .image_area_pixels = area});
  }
  return visible_tags;
}

}  // namespace localization
