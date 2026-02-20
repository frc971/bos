#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Transform3d.h>
#include <opencv2/core/mat.hpp>
#include "src/camera/camera_source.h"
#include "src/localization/position.h"

namespace localization {

constexpr double ktag_size = 0.1651;  // meters
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

// Interface for a class when given a apriltag detections, uses the detections to get the position of the robot
class IPositionSolver {
 public:
  virtual auto EstimatePosition(const std::vector<tag_detection_t>& detections)
      -> std::vector<position_estimate_t> = 0;
  virtual ~IPositionSolver() = default;
};

}  // namespace localization
