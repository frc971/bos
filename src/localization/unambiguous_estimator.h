#pragma once
#include <wpi/DataLogWriter.h>
#include "multi_tag_solver.h"
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/multi_camera_source.h"
#include "src/localization/apriltag_detector.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/position.h"
#include "src/localization/position_solver.h"
#include "src/utils/pch.h"

namespace localization {
class UnambiguousEstimator : public IJointPositionSolver {
 public:
  UnambiguousEstimator(
      const std::vector<camera::camera_constant_t>& camera_constants);
  auto EstimatePosition(
      std::vector<std::vector<tag_detection_t>>& detection_batches,
      bool reject_far_tags = true)
      -> std::optional<position_estimate_t> override;

 private:
  static auto Cost(const frc::Pose3d& a, const frc::Pose3d& b) -> double;
  auto ComputeCost(const std::vector<position_estimate_t>& poses) -> double;
  static auto WeightedAveragePose(
      const std::vector<position_estimate_t>& solutions) -> frc::Pose3d;
  auto SearchSolutions(
      const std::vector<ambiguous_estimate_t>& all_pose_estimates, size_t index,
      std::vector<position_estimate_t>& current_solution,
      std::vector<position_estimate_t>& best_solution, double& best_cost)
      -> double;
  auto GetAmbiguousEstimates(
      std::vector<std::vector<tag_detection_t>>& detection_batches)
      -> std::vector<ambiguous_estimate_t>;
  auto GetFilteredDetections(
      std::vector<std::vector<tag_detection_t>>& detection_batches)
      -> std::vector<std::optional<std::vector<tag_detection_t>>>;

  std::vector<MultiTagSolver> solvers_;
  std::mutex mutex_;
  std::vector<double> prev_timestamps_;
  std::optional<position_estimate_t> prev_pose_estimate_ = std::nullopt;
  static constexpr double kuse_prev_pose_threshold = 100;  // tune
  bool use_prev_pose_ = false;
};
}  // namespace localization
