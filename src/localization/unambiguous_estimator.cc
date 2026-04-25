#include "src/localization/unambiguous_estimator.h"
#include <frc/DataLogManager.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <wpi/DataLog.h>
#include <wpi/DataLogWriter.h>
#include <future>
#include <unordered_set>
#include <utility>
#include "absl/status/status.h"
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/disk_camera.h"
#include "src/camera/multi_camera_source.h"
#include "src/camera/uvc_camera.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/networktable_sender.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/position_sender.h"
#include "src/localization/position_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/log.h"
#include "src/utils/timer.h"
#include "src/utils/transform.h"

namespace localization {

UnambiguousEstimator::UnambiguousEstimator(
    const std::vector<camera::camera_constant_t>& camera_constants,
    bool verbose)
    : prev_timestamps_(camera_constants.size()), verbose_(verbose) {
  solvers_.reserve(camera_constants.size());
  for (const auto& camera_constant : camera_constants) {
    solvers_.emplace_back(camera_constant);
  }
}

auto UnambiguousEstimator::Cost(const frc::Pose3d& a, const frc::Pose3d& b)
    -> double {
  double translation = a.Translation().Distance(b.Translation()).value();

  frc::Rotation3d delta = a.Rotation() - b.Rotation();
  double rotation = delta.Angle().value();

  constexpr double krotation_weight = 0.1;  // tune
  return translation + krotation_weight * rotation;
}

auto UnambiguousEstimator::ComputeCost(
    const std::vector<position_estimate_t>& poses) -> double {
  double cost = 0.0;

  for (size_t i = 0; i < poses.size(); i++) {
    if (poses[i].invalid) {
      return 1000;
    }
    for (size_t j = i + 1; j < poses.size(); j++) {
      cost += Cost(poses[i].pose, poses[j].pose);
    }
    if (prev_pose_estimate_.has_value() /*use_prev_pose_*/) {
      cost += Cost(poses[i].pose, prev_pose_estimate_.value().pose);
    }
  }

  return cost;
}

auto UnambiguousEstimator::WeightedAveragePose(
    const std::vector<position_estimate_t>& solutions) -> frc::Pose3d {
  if (solutions.empty())
    return frc::Pose3d{};
  if (solutions.size() == 1)
    return solutions[0].pose;

  double total_weight = 0.0;
  for (const auto& est : solutions) {
    total_weight += 1.0 / est.variance;
  }

  double x = 0, y = 0, z = 0;
  double qw = 0.0, qx = 0.0, qy = 0.0, qz = 0.0;

  for (const auto& est : solutions) {
    double w = (1.0 / est.variance) / total_weight;
    x += w * est.pose.X().value();
    y += w * est.pose.Y().value();
    z += w * est.pose.Z().value();

    auto q = est.pose.Rotation().GetQuaternion();

    if (qw * q.W() + qx * q.X() + qy * q.Y() + qz * q.Z() < 0.0) {
      qw += w * (-q.W());
      qx += w * (-q.X());
      qy += w * (-q.Y());
      qz += w * (-q.Z());
    } else {
      qw += w * q.W();
      qx += w * q.X();
      qy += w * q.Y();
      qz += w * q.Z();
    }
  }

  double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  qw /= norm;
  qx /= norm;
  qy /= norm;
  qz /= norm;

  return frc::Pose3d{units::meter_t{x}, units::meter_t{y}, units::meter_t{z},
                     frc::Rotation3d{frc::Quaternion{qw, qx, qy, qz}}};
}

auto UnambiguousEstimator::SearchSolutions(
    const std::vector<ambiguous_estimate_t>& all_pose_estimates, size_t index,
    std::vector<position_estimate_t>& current_solution,
    std::vector<position_estimate_t>& best_solution, double& best_cost)
    -> double {
  if (index == all_pose_estimates.size()) {
    double cost = ComputeCost(current_solution);

    if (cost < best_cost) {
      best_cost = cost;
      best_solution = current_solution;
    }

    return best_cost;
  }

  const ambiguous_estimate_t& maybe_ambiguous_estimate =
      all_pose_estimates[index];

  current_solution.push_back(maybe_ambiguous_estimate.pos1);
  SearchSolutions(all_pose_estimates, index + 1, current_solution,
                  best_solution, best_cost);
  current_solution.pop_back();

  if (maybe_ambiguous_estimate.pos2.has_value()) {
    current_solution.push_back(maybe_ambiguous_estimate.pos2.value());
    SearchSolutions(all_pose_estimates, index + 1, current_solution,
                    best_solution, best_cost);
    current_solution.pop_back();
  }
  return best_cost;
}

auto UnambiguousEstimator::GetAmbiguousEstimates(
    std::vector<std::vector<tag_detection_t>>& detection_batches)
    -> std::vector<ambiguous_estimate_t> {
  std::vector<std::optional<std::vector<tag_detection_t>>> filtered_detections =
      GetFilteredDetections(detection_batches);
  std::vector<ambiguous_estimate_t> estimates;
  for (size_t i = 0; i < filtered_detections.size(); ++i) {
    if (!filtered_detections[i].has_value()) {
      continue;
    }
    prev_timestamps_[i] = filtered_detections[i]->at(0).timestamp;

    std::optional<ambiguous_estimate_t> est =
        solvers_[i].EstimatePositionAmbiguous(filtered_detections[i].value(),
                                              false);

    if (!est.has_value()) {
      continue;
    }

    bool firstOffField = utils::PoseOffField(est->pos1.pose);
    if (est->pos2.has_value()) {
      bool secondOffField = utils::PoseOffField(est->pos2.value().pose);
      if (firstOffField && secondOffField) {
        continue;
      }
      est->pos1.invalid = firstOffField;
      est->pos2->invalid = secondOffField;
    } else if (firstOffField) {
      continue;
    }
    estimates.push_back(std::move(est.value()));
  }
  return estimates;
}

auto UnambiguousEstimator::GetFilteredDetections(
    std::vector<std::vector<tag_detection_t>>& detection_batches)
    -> std::vector<std::optional<std::vector<tag_detection_t>>> {
  std::vector<std::optional<std::vector<tag_detection_t>>> usable_detections(
      detection_batches.size());
  constexpr double kacceptable_frame_recency = 0.25;
  double latest_timestamp = -1;
  for (auto& detection_batch : detection_batches) {
    if (detection_batch.empty()) {
      continue;
    }
    if (detection_batch[0].timestamp > latest_timestamp) {
      latest_timestamp = detection_batch[0].timestamp;
    }
  }
  for (size_t i = 0; i < detection_batches.size(); i++) {
    if (!detection_batches[i].empty() &&
        latest_timestamp - detection_batches[i][0].timestamp <
            kacceptable_frame_recency) {
      usable_detections[i] = std::move(detection_batches[i]);
    }
  }

  return usable_detections;
}

auto UnambiguousEstimator::EstimatePosition(
    std::vector<std::vector<tag_detection_t>>&& detection_batches,
    bool reject_far_tags) -> std::optional<position_estimate_t> {
  utils::Timer computation_timer("Getting estimate", false);
  const auto& ambiguous_estimates = GetAmbiguousEstimates(detection_batches);
  std::vector<position_estimate_t> best_solution;
  std::vector<position_estimate_t> current_solution;
  use_prev_pose_ = true;
  double best_cost = std::numeric_limits<double>::infinity();

  double cost = SearchSolutions(ambiguous_estimates, 0, current_solution,
                                best_solution, best_cost);
  if (best_solution.size() == 0) {
    return std::nullopt;
  }
  double avg_variance = 0;
  double avg_timestamp = 0;
  std::unordered_set<int> tag_ids;
  bool invalid = false;
  for (const position_estimate_t& est : best_solution) {
    avg_variance += est.variance;
    for (const int tag_id : est.tag_ids) {
      tag_ids.insert(tag_id);
    }
    avg_timestamp += est.timestamp;
  }
  avg_timestamp /= best_solution.size();
  avg_variance /= best_solution.size();
  const int num_tags = tag_ids.size();
  double latency = computation_timer.Stop();
  return std::make_optional<position_estimate_t>(
      {.pose = WeightedAveragePose(best_solution),
       .variance = avg_variance,
       .timestamp = avg_timestamp,
       .num_tags = num_tags,
       .latency = latency,
       .invalid = invalid,
       .loss = cost});
}

}  // namespace localization
