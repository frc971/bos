#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/localization/position_solver.h"

static const uint kmax_tags = 50;

using json = nlohmann::json;

namespace localization {

class MultiTagSolver : public IPositionSolver {
 public:
  MultiTagSolver(
      const std::string& intrinsics_path, const std::string& extrinsics_path,
      const frc::AprilTagFieldLayout& layout = kapriltag_layout,
      const std::vector<cv::Point3d>& tag_corners = kapriltag_corners);
  MultiTagSolver(
      camera::Camera camera_config,
      const frc::AprilTagFieldLayout& layout = kapriltag_layout,
      const std::vector<cv::Point3d>& tag_corners = kapriltag_corners);
  auto EstimatePosition(const std::vector<tag_detection_t>& detections)
      -> std::vector<position_estimate_t> override;

 private:
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  cv::Mat camera_to_robot_;
  std::array<std::optional<std::array<cv::Point3d, 4>>, kmax_tags> tag_corners_;
};
}  // namespace localization
