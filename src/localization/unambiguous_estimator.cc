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
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/disk_camera.h"
#include "src/camera/multi_camera_source.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/position_sender.h"
#include "src/localization/position_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/log.h"
#include "src/utils/timer.h"
#include "src/utils/transform.h"

namespace localization {

bool UnambiguousEstimator::log_interesting_timestamp_ = false;

UnambiguousEstimator::UnambiguousEstimator(
    std::vector<std::pair<camera::camera_constant_t, Detector>>& cameras,
    std::optional<uint> port_start, bool verbose,
    std::optional<std::vector<std::filesystem::path>> img_dir_paths)
    : port_start_(port_start),
      prev_timestamps_(cameras.size()),
      sim_(img_dir_paths.has_value()) {
  std::string log_path = frc::DataLogManager::GetLogDir();
  auto camera_constants = camera::GetCameraConstants();
  detectors_.reserve(cameras.size());
  solvers_.reserve(cameras.size());
  if (port_start.has_value()) {
    streamers_.reserve(cameras.size());
  }
  std::cout << "Initializing cameras" << std::endl;
  std::vector<std::unique_ptr<camera::ICamera>> icameras;
  for (size_t i = 0; i < cameras.size(); i++) {
    if (sim_) {
      icameras.push_back(std::make_unique<camera::DiskCamera>(
          img_dir_paths.value()[i], cameras[i].first, 10,
          interesting_timestamp_start_ - 1, interesting_timestamp_end_));
    } else {
      icameras.push_back(std::make_unique<camera::CVCamera>(
          cameras[i].first,
          fmt::format("{}/{}", log_path, cameras[i].first.name)));
    }
  }
  sources_ = std::make_unique<camera::MultiCameraSource>(icameras, sim_);
  std::cout << "Initialized cameras" << std::endl;
  std::this_thread::sleep_for(std::chrono::duration<double>(2));
  std::cout << "Initializing estimators and streamers" << std::endl;
  std::vector<cv::Mat> first_frames = sources_->GetCVFrames();
  for (size_t i = 0; i < cameras.size(); i++) {
    switch (cameras[i].second) {
      case OPENCV_CPU:
        detectors_.push_back(std::make_unique<OpenCVAprilTagDetector>(
            first_frames[i].cols, first_frames[i].rows,
            utils::ReadIntrinsics(cameras[i].first.intrinsics_path.value())));
        break;
      case AUSTIN_GPU:
        detectors_.push_back(std::make_unique<GPUAprilTagDetector>(
            first_frames[i].cols, first_frames[i].rows,
            utils::ReadIntrinsics(cameras[i].first.intrinsics_path.value())));
        break;
      default:
        LOG(FATAL) << "Invalid solver type";
        return;
    }
    solvers_.emplace_back(cameras[i].first);
    if (port_start.has_value()) {
      streamers_.emplace_back(cameras[i].first.name, port_start.value() + i, 30,
                              1080, 1080);
    }
  }
  std::cout << "Initialized estimators and streamers" << std::endl;
}

auto UnambiguousEstimator::Cost(const frc::Pose3d& a, const frc::Pose3d& b)
    -> double {
  double translation = a.Translation().Distance(b.Translation()).value();

  frc::Rotation3d delta = a.Rotation() - b.Rotation();
  double rotation = delta.Angle().value();

  constexpr double kRotationWeight = 3.0;  // tune
  return translation + kRotationWeight * rotation;
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
    const std::vector<ambiguous_estimate_t>& all_pose_estimates_, size_t index,
    std::vector<position_estimate_t>& current_solution,
    std::vector<position_estimate_t>& best_solution, double& best_cost)
    -> double {
  if (index == all_pose_estimates_.size()) {
    double cost = ComputeCost(current_solution);

    if (cost < best_cost) {
      best_cost = cost;
      best_solution = current_solution;
    }

    return best_cost;
  }

  const auto& pair = all_pose_estimates_[index];

  current_solution.push_back(pair.pos1);
  SearchSolutions(all_pose_estimates_, index + 1, current_solution,
                  best_solution, best_cost);
  current_solution.pop_back();

  if (pair.pos2.has_value()) {
    current_solution.push_back(pair.pos2.value());
    SearchSolutions(all_pose_estimates_, index + 1, current_solution,
                    best_solution, best_cost);
    current_solution.pop_back();
  }
  return best_cost;
}

void UnambiguousEstimator::Run() {
  frc::DataLogManager::Start();
  localization::PositionSender position_sender("Left", false, sim_);
  while (true) {
    latent_estimate_t pose_estimate = GetUnambiguatedEstimate();
    if (pose_estimate.invalid) {
      continue;
    }
    position_sender.Send(
        std::vector<position_estimate_t>{pose_estimate.pose_estimate},
        pose_estimate.latency, pose_estimate.all_pose_estimates);
  }
}

auto UnambiguousEstimator::GetAmbiguousEstimates()
    -> std::vector<ambiguous_estimate_t> {
  std::vector<camera::timestamped_frame_t> frames =
      sources_->GetTimestampedFrames();
  std::vector<std::optional<camera::timestamped_frame_t>> usable_frames =
      GetUsableFrames(frames);
  std::vector<ambiguous_estimate_t> estimates;
  for (size_t i = 0; i < usable_frames.size(); ++i) {
    if (!usable_frames[i].has_value()) {
      continue;
    }
    if (sim_ && usable_frames[i]->invalid) {
      frc::DataLogManager::Stop();
      std::cout << "Stopped log" << std::endl;
      throw std::runtime_error("DONE");
    }
    prev_timestamps_[i] = usable_frames[i]->timestamp;

    std::vector<tag_detection_t> detections =
        detectors_[i]->GetTagDetections(usable_frames[i].value());

    if (detections.empty()) {
      continue;
    }

    std::optional<ambiguous_estimate_t> est =
        solvers_[i].EstimatePositionAmbiguous(detections, false);

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

auto UnambiguousEstimator::GetUsableFrames(
    std::vector<camera::timestamped_frame_t>& frames)
    -> std::vector<std::optional<camera::timestamped_frame_t>> {
  std::vector<std::optional<camera::timestamped_frame_t>> usable_frames(
      frames.size());
  constexpr double kacceptable_frame_recency = 0.25;
  double latest_timestamp = -1;
  for (auto& frame : frames) {
    if (sim_ && (frame.invalid || frame.frame.empty())) {
      std::cout << "STOPPING LOG" << std::endl;
      frc::DataLogManager::Stop();
      std::exit(0);
    }
    if (frame.timestamp > latest_timestamp) {
      latest_timestamp = frame.timestamp;
    }
    UnambiguousEstimator::log_interesting_timestamp_ =
        frame.timestamp > interesting_timestamp_start_ &&
        frame.timestamp < interesting_timestamp_end_;
  }
  int count = 0;
  for (size_t i = 0; i < frames.size(); i++) {
    if (latest_timestamp - frames[i].timestamp < kacceptable_frame_recency) {
      // streamers_[count].WriteFrame(frames[i].frame);
      usable_frames[i] = std::move(frames[i]);
      count++;
    }
  }

  return usable_frames;
}

auto UnambiguousEstimator::GetUnambiguatedEstimate() -> latent_estimate_t {
  utils::Timer everything_timer("Getting estimate", false);
  const auto& ambiguous_estimates = GetAmbiguousEstimates();
  std::vector<frc::Pose3d> all_pose_estimates_for_log;
  for (const auto& est : ambiguous_estimates) {
    all_pose_estimates_for_log.push_back(est.pos1.pose);
    if (est.pos2.has_value()) {
      all_pose_estimates_for_log.push_back(est.pos2.value().pose);
    }
  }
  std::vector<position_estimate_t> best_solution;
  std::vector<position_estimate_t> current_solution;
  use_prev_pose_ = true;
  double best_cost = std::numeric_limits<double>::infinity();

  double cost = SearchSolutions(ambiguous_estimates, 0, current_solution,
                                best_solution, best_cost);
  if (best_solution.size() == 0) {
    return {.invalid = true};
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
  position_estimate_t averaged_estimate = {
      .pose = WeightedAveragePose(best_solution),
      .variance = avg_variance,
      .timestamp = avg_timestamp,
      .num_tags = num_tags,
      .invalid = invalid,
      .loss = cost};
  prev_pose_estimate_ = std::make_optional(averaged_estimate);
  return {.pose_estimate = averaged_estimate,
          .latency = everything_timer.Stop(),
          .best_cost = cost,
          .used_prev_pose = use_prev_pose_,
          .all_pose_estimates = all_pose_estimates_for_log};
}

}  // namespace localization
