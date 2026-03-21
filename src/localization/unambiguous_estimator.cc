#include "src/localization/unambiguous_estimator.h"
#include <frc/DataLogManager.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <wpi/DataLog.h>
#include <wpi/DataLogWriter.h>
#include <unordered_set>
#include <utility>
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/camera/disk_camera.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/position_sender.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/timer.h"

namespace localization {

UnambiguousEstimator::UnambiguousEstimator(
    std::vector<std::pair<camera::CameraConstant, Detector>>& cameras,
    std::optional<std::vector<std::filesystem::path>>& img_dir_paths,
    std::optional<uint> port_start, bool verbose)
    : port_start_(port_start),
      prev_timestamps_(cameras.size()),
      sim_(img_dir_paths.has_value()) {
  std::string log_path = frc::DataLogManager::GetLogDir();
  auto camera_constants = camera::GetCameraConstants();
  sources_.reserve(cameras.size());
  detectors_.reserve(cameras.size());
  solvers_.reserve(cameras.size());
  if (port_start.has_value()) {
    streamers_.reserve(cameras.size());
  }
  for (size_t i = 0; i < cameras.size(); i++) {
    if (sim_) {
      sources_.push_back(std::make_unique<camera::CameraSource>(
          cameras[i].first.name,
          std::make_unique<camera::DiskCamera>(img_dir_paths.value()[i], 10),
          true, false));
    } else {
      sources_.push_back(std::make_unique<camera::CameraSource>(
          cameras[i].first.name,
          std::make_unique<camera::CVCamera>(
              cameras[i].first,
              fmt::format("{}/{}", log_path, cameras[i].first.name))));
    }
    const cv::Mat first_frame = sources_[i]->GetFrame();
    switch (cameras[i].second) {
      case OPENCV_CPU:
        detectors_.push_back(std::make_unique<OpenCVAprilTagDetector>(
            first_frame.cols, first_frame.rows,
            utils::ReadIntrinsics(cameras[i].first.intrinsics_path.value())));
        break;
      case AUSTIN_GPU:
        detectors_.push_back(std::make_unique<GPUAprilTagDetector>(
            first_frame.cols, first_frame.rows,
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
  if (sim_) {
    std::error_code ec;
    log_.emplace("unambiguous.wpilog", ec);
    if (ec) {
      std::cerr << "Failed to open log: " << ec.message() << std::endl;
      return;
    }

    log_->AddStructSchema<frc::Translation3d>(0);
    log_->AddStructSchema<frc::Rotation3d>(0);
    log_->AddStructSchema<frc::Pose3d>(0);

    pose_log_.emplace(*log_, "/localization/pose");
    num_tags_log_.emplace(*log_, "/localization/num_tags");
    timestamp_log_.emplace(*log_, "/localization/timestamp");
    used_prev_pose_log_.emplace(*log_, "/localization/used_prev_pose");
    best_cost_log_.emplace(*log_, "/localization/best_cost");
    all_pose_estimates_log_.emplace(*log_, "/localization/all_pose_estimates");
  }
}

auto UnambiguousEstimator::Cost(const frc::Pose3d& a, const frc::Pose3d& b)
    -> double {
  double translation = a.Translation().Distance(b.Translation()).value();

  frc::Rotation3d delta = a.Rotation() - b.Rotation();
  double rotation = delta.Angle().value();

  constexpr double kRotationWeight = 0.0;
  return translation + kRotationWeight * rotation;
}

auto UnambiguousEstimator::ComputeCost(
    const std::vector<position_estimate_t>& poses) -> double {
  double cost = 0.0;

  for (size_t i = 0; i < poses.size(); i++) {
    for (size_t j = i + 1; j < poses.size(); j++) {
      cost += Cost(poses[i].pose, poses[j].pose);
    }
    if (use_prev_pose_) {
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
    const std::vector<std::pair<position_estimate_t, position_estimate_t>>&
        all_pose_estimates_,
    size_t index, std::vector<position_estimate_t>& current_solution,
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

  current_solution.push_back(pair.first);
  SearchSolutions(all_pose_estimates_, index + 1, current_solution,
                  best_solution, best_cost);
  current_solution.pop_back();

  current_solution.push_back(pair.second);
  SearchSolutions(all_pose_estimates_, index + 1, current_solution,
                  best_solution, best_cost);
  current_solution.pop_back();
  return best_cost;
}

auto UnambiguousEstimator::FillPoseEstimates()
    -> std::vector<std::pair<position_estimate_t, position_estimate_t>> {
  std::vector<std::pair<position_estimate_t, position_estimate_t>>
      all_pose_estimates_;
  std::vector<std::thread> workers;
  for (size_t i = 0; i < sources_.size(); ++i) {
    workers.emplace_back([&, i]() {
      camera::timestamped_frame_t frame = sources_[i]->Get();
      if (frame.invalid) {
        std::cout << "Stopping log" << std::endl;
        log_.value().Stop();
        std::cout << "Stopped log" << std::endl;
        throw std::runtime_error("DONE");
      }
      if (prev_timestamps_[i] == frame.timestamp) {
        return;
      }
      log_interesting_timestamp_ =
          frame.timestamp > interesting_timestamp_start_ &&
          frame.timestamp < interesting_timestamp_end_;
      if (log_interesting_timestamp_) {
        std::cout << "It's loggin time" << std::endl;
      }
      prev_timestamps_[i] = frame.timestamp;

      std::vector<tag_detection_t> detections =
          detectors_[i]->GetTagDetections(frame);

      std::vector<std::pair<position_estimate_t, position_estimate_t>>
          pose_estimates =
              solvers_[i].EstimatePositionAmbiguous(detections, false);

      {
        std::lock_guard<std::mutex> lock(mutex_);
        all_pose_estimates_.insert(all_pose_estimates_.end(),
                                   pose_estimates.begin(),
                                   pose_estimates.end());
      }

      if (port_start_.has_value()) {
        streamers_[i].WriteFrame(frame.frame);
      }
    });
  }
  for (auto& t : workers) {
    t.join();
  }
  if (all_pose_estimates_.empty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  use_prev_pose_ =
      prev_pose_estimate_.has_value() && !all_pose_estimates_.empty() &&
      all_pose_estimates_[0].first.timestamp - prev_pose_estimate_->timestamp <
          kuse_prev_pose_threshold;
  if (log_interesting_timestamp_ && !all_pose_estimates_.empty()) {
    std::cout << "time diff: "
              << all_pose_estimates_[0].first.timestamp -
                     prev_pose_estimate_->timestamp
              << std::endl;
  }
  return all_pose_estimates_;
}

void UnambiguousEstimator::Run() {
  if (!sim_) {
    localization::PositionSender position_sender("Front");
    while (true) {
      latent_estimate_t estimate = GetUnambiguatedEstimate();
      position_sender.Send(
          std::vector<position_estimate_t>{estimate.pose_estimate},
          estimate.latency);
    }
  } else {
    while (true) {
      latent_estimate_t pose_estimate = GetUnambiguatedEstimate();
      if (pose_estimate.invalid) {
        continue;
      }
      if (std::isnan(pose_estimate.pose_estimate.timestamp)) {
        std::cout << "How this even possible" << std::endl;
      }
      auto log_time =
          static_cast<int64_t>(pose_estimate.pose_estimate.timestamp * 1e6);
      pose_log_.value().Append(pose_estimate.pose_estimate.pose, log_time);
      num_tags_log_.value().Append(pose_estimate.pose_estimate.num_tags,
                                   log_time);
      timestamp_log_.value().Append(pose_estimate.pose_estimate.timestamp,
                                    log_time);
      best_cost_log_.value().Append(pose_estimate.best_cost, log_time);
      used_prev_pose_log_.value().Append(pose_estimate.used_prev_pose,
                                         log_time);
      all_pose_estimates_log_.value().Append(pose_estimate.all_pose_estimates,
                                             log_time);
    }
  }
}

auto UnambiguousEstimator::GetUnambiguatedEstimate() -> latent_estimate_t {
  utils::Timer fetch_timer("Fetch", false);
  const std::vector<std::pair<position_estimate_t, position_estimate_t>>
      all_pose_estimates = FillPoseEstimates();
  std::vector<frc::Pose3d> all_pose_estimates_for_log;
  for (const auto& est : all_pose_estimates) {
    all_pose_estimates_for_log.push_back(est.first.pose);
    all_pose_estimates_for_log.push_back(est.second.pose);
  }
  fetch_timer.Stop();
  std::vector<position_estimate_t> best_solution;
  std::vector<position_estimate_t> current_solution;

  double best_cost = std::numeric_limits<double>::infinity();

  utils::Timer search_timer("Search", false);
  double cost = SearchSolutions(all_pose_estimates, 0, current_solution,
                                best_solution, best_cost);
  search_timer.Stop();
  if (best_solution.size() == 0) {
    return {.invalid = true};
  }
  utils::Timer everything_timer("everything else", false);
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
  if (std::isnan(avg_timestamp)) {
    std::cout << "Morbin time" << std::endl;
  }
  const int num_tags = tag_ids.size();
  position_estimate_t averaged_estimate = {
      .pose = WeightedAveragePose(best_solution),
      .variance = avg_variance,
      .timestamp = avg_timestamp,
      .num_tags = num_tags,
      .invalid = invalid};
  everything_timer.Stop();
  prev_pose_estimate_ = std::make_optional(averaged_estimate);
  return {.pose_estimate = averaged_estimate,
          .latency = 0,
          .best_cost = cost,
          .used_prev_pose = use_prev_pose_,
          .all_pose_estimates = all_pose_estimates_for_log};
}

}  // namespace localization
