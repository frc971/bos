#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "nlohmann/json.hpp"
#include "src/camera/camera_constants.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/transform.h"

static const uint kmax_tags = 50;

using json = nlohmann::json;

namespace localization {

class JointSolver {
 public:
  JointSolver(const std::vector<camera::Camera>& camera_config,
              frc::AprilTagFieldLayout layout = kapriltag_layout,
              double tag_size = ktag_size);
  auto EstimatePosition(
      const std::vector<std::vector<tag_detection_t>>& detections,
      const frc::Pose3d& intial_pose) -> std::vector<position_estimate_t>;

  auto ProjectPoint(const frc::Pose3d& robot_pose, int tag_id, int camera_index)
      -> std::vector<cv::Point2d>;

 private:
  frc::AprilTagFieldLayout layout_;
  std::vector<cv::Mat> camera_matrix_;
  std::vector<cv::Mat> distortion_coefficients_;
  std::vector<cv::Mat> camera_to_robot_;
  cv::Mat rotate_z_;
  cv::Mat pi_;
  std::array<std::optional<std::array<cv::Mat, 4>>, kmax_tags> tag_corners_;
};
}  // namespace localization
