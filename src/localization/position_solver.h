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

const std::vector<Eigen::Vector4d> kapriltag_corners_eigen_homogenized = {
    {0, ktag_size / 2, -ktag_size / 2, 1},   // Bottom left
    {0, -ktag_size / 2, -ktag_size / 2, 1},  // Bottom right
    {0, -ktag_size / 2, ktag_size / 2, 1},   // Top right
    {0, ktag_size / 2, ktag_size / 2, 1},    // Top left
};

const frc::AprilTagFieldLayout kapriltag_layout =
    frc::AprilTagFieldLayout::LoadField(
        frc::AprilTagField::k2026RebuiltAndyMark);

inline auto Variance(const int num_tags_detected, const double distance,
                     const double min_variance, const double scalar)
    -> double {  // distance can be avg
  return distance * scalar / (num_tags_detected * num_tags_detected) +
         min_variance;
}
static constexpr double kmax_tag_distance = 5.0;

// Interface for a class when given a apriltag detections, uses the detections to get the position of the robot
class IPositionSolver {
 public:
  virtual auto EstimatePosition(const std::vector<tag_detection_t>& detections,
                                bool reject_far_tags = true)
      -> std::vector<position_estimate_t> = 0;
  virtual ~IPositionSolver() = default;
};

}  // namespace localization
