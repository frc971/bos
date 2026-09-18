#pragma once

#include <array>
#include <string>
#include <vector>

#include <frc/apriltag/AprilTag.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <opencv2/core/types.hpp>

#include "src/camera/camera_constants.h"

namespace localization {

// A pinhole camera model expressed in WPILib's robot coordinate system
// (x forward, y left, z up).
struct VisibilityCameraModel {
  std::string name;
  int image_width;
  int image_height;
  double fx;
  double fy;
  double cx;
  double cy;
  frc::Transform3d robot_to_camera;
  double minimum_tag_area_pixels = 100.0;
  double maximum_tag_distance_meters = 5.0;
};

struct VisibleTag {
  int id;
  frc::Pose3d pose;
  std::array<cv::Point2d, 4> image_corners;
  double distance_meters;
  double image_area_pixels;
};

// Builds a model from the same calibration files used by localization.
auto MakeVisibilityCameraModel(const camera::camera_constant_t& camera_constant)
    -> VisibilityCameraModel;

// Predicts tags whose entire face projects inside the image. Tags that face
// away from the camera, are behind it, too far away, or too small are rejected.
auto GetVisibleTags(const frc::Pose3d& robot_pose,
                    const VisibilityCameraModel& camera,
                    const std::vector<frc::AprilTag>& tags)
    -> std::vector<VisibleTag>;

}  // namespace localization
