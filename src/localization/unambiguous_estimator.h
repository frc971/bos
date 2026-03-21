#pragma once
#include <wpi/DataLogWriter.h>
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/cscore_streamer.h"
#include "src/localization/apriltag_detector.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/position.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/pch.h"

namespace localization {
enum Detector { OPENCV_CPU, AUSTIN_GPU };
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
  UnambiguousEstimator(
      std::vector<std::pair<camera::CameraConstant, Detector>>& cameras,
      std::optional<std::vector<std::filesystem::path>>& img_dir_paths,
      std::optional<uint> port_start = std::nullopt, bool verbose = false);
  void Run();

 private:
  auto GetUnambiguatedEstimate() -> latent_estimate_t;
  auto FillPoseEstimates()
      -> std::vector<std::pair<position_estimate_t, position_estimate_t>>;
  static auto Cost(const frc::Pose3d& a, const frc::Pose3d& b) -> double;
  auto ComputeCost(const std::vector<position_estimate_t>& poses) -> double;
  static auto WeightedAveragePose(
      const std::vector<position_estimate_t>& solutions) -> frc::Pose3d;
  auto SearchSolutions(
      const std::vector<std::pair<position_estimate_t, position_estimate_t>>&
          all_pose_estimates,
      size_t index, std::vector<position_estimate_t>& current_solution,
      std::vector<position_estimate_t>& best_solution, double& best_cost)
      -> double;

 private:
  std::vector<camera::CscoreStreamer> streamers_;
  std::vector<std::unique_ptr<camera::CameraSource>> sources_;
  std::vector<std::unique_ptr<IAprilTagDetector>> detectors_;
  std::vector<SquareSolver> solvers_;
  const std::optional<uint> port_start_;
  std::mutex mutex_;
  std::vector<double> prev_timestamps_;
  std::optional<position_estimate_t> prev_pose_estimate_ = std::nullopt;
  const bool sim_;
  std::optional<wpi::log::DataLogWriter> log_;
  std::optional<wpi::log::StructLogEntry<frc::Pose3d>> pose_log_;
  std::optional<wpi::log::DoubleLogEntry> num_tags_log_;
  std::optional<wpi::log::DoubleLogEntry> timestamp_log_;
  std::optional<wpi::log::DoubleLogEntry> best_cost_log_;
  std::optional<wpi::log::BooleanLogEntry> used_prev_pose_log_;
  std::optional<wpi::log::StructArrayLogEntry<frc::Pose3d>> all_pose_estimates_log_;
  static constexpr double interesting_timestamp_start_ = 34;
  static constexpr double interesting_timestamp_end_ = 35;
  static constexpr double kuse_prev_pose_threshold = 100;
  bool log_interesting_timestamp_ = false;
  bool use_prev_pose_ = false;
};
}  // namespace localization
