#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Transform3d.h>
#include <opencv2/core/mat.hpp>
#include "src/camera/camera_source.h"
#include "src/localization/position.h"

namespace localization {

constexpr double ktag_size = 0.1651;            // meters
constexpr double kmin_tag_area_pixels = 100.0;  // px²
const std::vector<cv::Point3d> kapriltag_corners = {
    {-ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, -ktag_size / 2, 0},
    {-ktag_size / 2, -ktag_size / 2, 0}};

const std::vector<Eigen::Vector3d> kapriltag_corners_eigen = {
    {-ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, -ktag_size / 2, 0},
    {-ktag_size / 2, -ktag_size / 2, 0}};

const frc::AprilTagFieldLayout kapriltag_layout =
    frc::AprilTagFieldLayout::LoadField(
        frc::AprilTagField::k2026RebuiltAndyMark);

inline auto Variance(const size_t num_tags_detected, const double distance,
                     const double min_variance, const double scalar)
    -> double {  // distance can be avg
  return distance * scalar / std::pow(2, num_tags_detected - 1) + min_variance;
}

// Interface for a class when given a apriltag detections, uses the detections to get the position of the robot
class IPositionSolver {
 public:
  virtual auto EstimatePosition(const std::vector<tag_detection_t>& detections,
                                bool reject_far_tags = true)
      -> std::vector<position_estimate_t> = 0;
  virtual ~IPositionSolver() = default;
};

}  // namespace localization
