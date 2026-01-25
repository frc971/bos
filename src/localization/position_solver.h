#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Transform3d.h>
#include <opencv2/core/mat.hpp>
#include "src/camera/camera_source.h"
#include "src/localization/position.h"

namespace localization {

constexpr double ktag_size = 0.1651;  // meters
const std::vector<cv::Point3f> kapriltag_corners = {
    {-ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, -ktag_size / 2, 0},
    {-ktag_size / 2, -ktag_size / 2, 0}};

const frc::AprilTagFieldLayout kapriltag_layout =
    frc::AprilTagFieldLayout("/bos/constants/2026-rebuilt-andymark.json");

class IPositionSolver {
 public:
  virtual auto EstimatePosition(const std::vector<tag_detection_t>& detections)
      -> std::vector<position_estimate_t> = 0;
  virtual ~IPositionSolver() = default;
};

}  // namespace localization
