#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/localization/position.h"
#include "src/localization/position_solver.h"

static const uint kmax_tags = 50;

using json = nlohmann::json;

namespace localization {

using ambiguous_estimate_t = struct AmbiguousEstimate {
  position_estimate_t pos1;
  std::optional<position_estimate_t> pos2;
};

class MultiTagSolver : public IPositionSolver {
 public:
  MultiTagSolver(
      const std::string& intrinsics_path, const std::string& extrinsics_path,
      const frc::AprilTagFieldLayout& layout = kapriltag_layout,
      const std::vector<cv::Point3d>& tag_corners = kapriltag_corners);
  MultiTagSolver(
      camera::camera_constant_t camera_constant,
      const frc::AprilTagFieldLayout& layout = kapriltag_layout,
      const std::vector<cv::Point3d>& tag_corners = kapriltag_corners);
  auto EstimatePosition(const std::vector<tag_detection_t>& detections,
                        bool reject_far_tags = true)
      -> std::vector<position_estimate_t> override;
  auto EstimatePositionAmbiguous(const std::vector<tag_detection_t>& detections,
                                 bool reject_far_tags = true)
      -> std::vector<ambiguous_estimate_t>;

 private:
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  cv::Mat camera_to_robot_;
  std::array<std::optional<std::array<cv::Point3d, 4>>, kmax_tags> tag_corners_;
  static constexpr double kvariance_scalar_ = 1;
  static constexpr double kvariance_min_ = 1.0;
};
}  // namespace localization
