#pragma once
#include <networktables/BooleanTopic.h>
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
using latent_estimate_t = struct LatentEstimate {
  position_estimate_t pose_estimate;
  double latency;
  double best_cost;
  bool used_prev_pose;
  std::vector<frc::Pose3d> all_pose_estimates;
  bool invalid = false;
};
class UnambiguousEstimator {
 public:
  UnambiguousEstimator(std::vector<camera::camera_constant_t>& cameras,
                       std::optional<uint> port_start = std::nullopt,
                       bool verbose = false,
                       std::optional<std::vector<std::filesystem::path>>
                           img_dir_paths = std::nullopt);
  void Run();
  static bool log_interesting_timestamp_;

 private:
  auto GetUnambiguatedEstimate() -> latent_estimate_t;
  static auto Cost(const frc::Pose3d& a, const frc::Pose3d& b) -> double;
  auto ComputeCost(const std::vector<position_estimate_t>& poses) -> double;
  auto WeightedAveragePose(const std::vector<position_estimate_t>& solutions)
      -> frc::Pose3d;
  auto SearchSolutions(
      const std::vector<ambiguous_estimate_t>& all_pose_estimates, size_t index,
      std::vector<position_estimate_t>& current_solution,
      std::vector<position_estimate_t>& best_solution, double& best_cost)
      -> double;
  auto GetAmbiguousEstimates() -> std::vector<ambiguous_estimate_t>;
  auto GetUsableFrames(std::vector<camera::timestamped_frame_t>& frames)
      -> std::vector<std::optional<camera::timestamped_frame_t>>;
  static constexpr double interesting_timestamp_start_ = 0;  // 13.265;
  static constexpr double interesting_timestamp_end_ = 600;

  std::vector<camera::CscoreStreamer> streamers_;
  std::unique_ptr<camera::MultiCameraSource> sources_;
  std::vector<std::unique_ptr<IAprilTagDetector>> detectors_;
  std::vector<MultiTagSolver> solvers_;
  const std::optional<uint> port_start_;
  std::mutex mutex_;
  std::vector<double> prev_timestamps_;
  std::optional<position_estimate_t> prev_pose_estimate_ = std::nullopt;
  const bool sim_;
  static constexpr double kuse_prev_pose_threshold = 100;  // tune
  bool use_prev_pose_ = false;
  nt::NetworkTableInstance nt_instance_;
  std::vector<nt::BooleanPublisher> camera_status_publishers_;
};
}  // namespace localization
