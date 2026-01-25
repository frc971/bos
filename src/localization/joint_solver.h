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
  Eigen::MatrixXd camera_matrix_;
  SquareSolver initial_solver_;
};
}  // namespace localization
