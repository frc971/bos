#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"

using json = nlohmann::json;

namespace localization {

class JointSolver : public IPositionSolver {
 public:
  JointSolver(const std::string& intrinsics_path,
              const std::string& extrinsics_path,
              frc::AprilTagFieldLayout layout = kapriltag_layout,
              double tag_size = ktag_size);
  JointSolver(camera::Camera camera_config,
              const frc::AprilTagFieldLayout& layout = kapriltag_layout,
              double tag_size = ktag_size);
  auto EstimatePosition(const std::vector<tag_detection_t>& detections)
      -> std::vector<position_estimate_t> override;

 private:
  frc::AprilTagFieldLayout layout_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  frc::Transform3d camera_to_robot_;
  std::unordered_map<int, std::vector<cv::Point3f>> absolute_apriltag_corners_;
};
}  // namespace localization
