#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/localization/position_solver.h"

using json = nlohmann::json;

namespace localization {

class SquareSolver : public IPositionSolver {
 public:
  SquareSolver(const std::string& intrinsics_path,
               const std::string& extrinsics_path,
               frc::AprilTagFieldLayout layout = kapriltag_layout,
               std::vector<cv::Point3f> tag_corners = kapriltag_corners);
  SquareSolver(camera::Camera camera_config,
               frc::AprilTagFieldLayout layout = kapriltag_layout,
               std::vector<cv::Point3f> tag_corners = kapriltag_corners);
  auto EstimatePosition(const std::vector<tag_detection_t>& detections)
      -> std::vector<position_estimate_t> override;

 private:
  frc::AprilTagFieldLayout layout_;
  std::vector<cv::Point3f> tag_corners_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
};
}  // namespace localization
