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
  }
}

auto UnambiguousEstimator::Cost(const frc::Pose3d& a, const frc::Pose3d& b)
    -> double {
  double translation = a.Translation().Distance(b.Translation()).value();

  frc::Rotation3d delta = a.Rotation() - b.Rotation();
  double rotation = delta.Angle().value();

  constexpr double kRotationWeight = 1.0;
  return translation + kRotationWeight * rotation;
}

auto UnambiguousEstimator::ComputeCost(
    const std::vector<position_estimate_t>& poses) -> double {
  double cost = 0.0;

  for (size_t i = 0; i < poses.size(); i++) {
    for (size_t j = i + 1; j < poses.size(); j++) {
      cost += Cost(poses[i].pose, poses[j].pose);
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
  double roll = 0, pitch = 0, yaw = 0;

  for (const auto& est : solutions) {
    double w = (1.0 / est.variance) / total_weight;
    x += w * est.pose.X().value();
    y += w * est.pose.Y().value();
    z += w * est.pose.Z().value();
    roll += w * est.pose.Rotation().X().value();
    pitch += w * est.pose.Rotation().Y().value();
    yaw += w * est.pose.Rotation().Z().value();
  }

  return frc::Pose3d{
      units::meter_t{x}, units::meter_t{y}, units::meter_t{z},
      frc::Rotation3d{units::radian_t{roll}, units::radian_t{pitch},
                      units::radian_t{yaw}}};
}

void UnambiguousEstimator::SearchSolutions(
    const std::vector<std::pair<position_estimate_t, position_estimate_t>>&
        all_pose_estimates_,
    size_t index, std::vector<position_estimate_t>& current_solution,
    std::vector<position_estimate_t>& best_solution, double& best_cost) {
  if (index == all_pose_estimates_.size()) {
    double cost = ComputeCost(current_solution);

    if (cost < best_cost) {
      best_cost = cost;
      best_solution = current_solution;
    }

    return;
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
}

auto UnambiguousEstimator::FillPoseEstimates()
    -> std::vector<std::pair<position_estimate_t, position_estimate_t>> {
  // std::cout << "FILLING POSE ESTIMATES" << std::endl;
  std::vector<std::pair<position_estimate_t, position_estimate_t>>
      all_pose_estimates_;
  std::vector<std::thread> workers;
  for (size_t i = 0; i < sources_.size(); ++i) {
    workers.emplace_back([&, i]() {
      // std::cout << "Fetching " << std::endl;
      camera::timestamped_frame_t frame = sources_[i]->Get();
      if (frame.invalid) {
        std::cout << "Stopping log" << std::endl;
        log_.value().Stop();
        std::cout << "Stopped log" << std::endl;
        throw std::runtime_error("DONE");
      }
      if (prev_timestamps_[i] == frame.timestamp) {
        // std::cout << "Rejecting " << frame.timestamp << std::endl;
        return;
      }
      if (frame.timestamp > interesting_timestamp_start_ &&
          frame.timestamp < interesting_timestamp_end_) {
        log_interesting_timestamp_ = true;
        std::cout << "Using: " << frame.timestamp << std::endl;
        // cv::imwrite(fmt::format("{}.jpg", frame.timestamp), frame.frame);
      } else {
        log_interesting_timestamp_ = false;
      }
      prev_timestamps_[i] = frame.timestamp;

      std::vector<tag_detection_t> detections =
          detectors_[i]->GetTagDetections(frame);

      // std::cout << "Timestamp: " << frame.timestamp << " has "
      //           << detections.size() << " detections" << std::endl;

      if (log_interesting_timestamp_ && detections.size() <= 1) {
        CHECK(frame.timestamp < 6);
        std::cout << "Writing bad frame with timestamp: " << frame.timestamp
                  << std::endl;
        cv::imwrite(fmt::format("joint_bad_frames/{}.jpg", frame.timestamp),
                    frame.frame);
      }

      std::vector<std::pair<position_estimate_t, position_estimate_t>>
          pose_estimates = solvers_[i].EstimatePositionAmbiguous(detections);

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
  } else {
    if (log_interesting_timestamp_) {
      // std::cout << "Timestamp: " << all_pose_estimates_[0].first.timestamp
      //           << " Num estimates: " << all_pose_estimates_.size()
      //           << std::endl;
    }
  }
  return all_pose_estimates_;
  // std::cout << "Received: " << all_pose_estimates_.size() << " estimates"
  //           << std::endl;
}

void UnambiguousEstimator::Run() {
  std::cout << "Running run" << std::endl;
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
      // std::cout << "Loop " << std::endl;
      position_estimate_t pose_estimate =
          GetUnambiguatedEstimate().pose_estimate;
      if (pose_estimate.invalid) {
        continue;
      }
      std::cout << "ab";
      auto log_time = static_cast<int64_t>(pose_estimate.timestamp * 1e6);
      pose_log_.value().Append(pose_estimate.pose, log_time);
      num_tags_log_.value().Append(pose_estimate.num_tags, log_time);
      timestamp_log_.value().Append(pose_estimate.timestamp, log_time);
    }
  }
}

auto UnambiguousEstimator::GetUnambiguatedEstimate() -> latent_estimate_t {
  // std::cout << "GETTING ESTIMATE " << std::endl;
  utils::Timer fetch_timer("Fetch", false);
  const auto& all_pose_estimates = FillPoseEstimates();
  if (all_pose_estimates.empty()) {
    std::cout << "NO ESTIMATES" << std::endl;
  } else {
    std::cout << "Timestamp " << all_pose_estimates[0].first.timestamp
              << " has " << all_pose_estimates.size() / 2 << " estimates"
              << std::endl;
  }
  fetch_timer.Stop();
  std::vector<position_estimate_t> best_solution;
  std::vector<position_estimate_t> current_solution;

  double best_cost = std::numeric_limits<double>::infinity();

  utils::Timer search_timer("Search", false);
  SearchSolutions(all_pose_estimates, 0, current_solution, best_solution,
                  best_cost);
  search_timer.Stop();
  if (log_interesting_timestamp_ && best_solution.size() == 0) {
    // std::cout << "Nothing in the solution " << std::endl;
    if (!all_pose_estimates.empty() &&
        all_pose_estimates[0].first.timestamp > 0.1) {
      for (const auto& est : all_pose_estimates) {
        std::cout << est.first << std::endl;
      }
      std::exit(0);
    }
    return {.invalid = true};
  }
  // std::cout << "Num estimates used: " << best_solution.size() << std::endl;
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
  // if (avg_timestamp >= 15 && avg_timestamp < 16) {
  //   std::cout << "Had estimates: " << std::endl;
  //   for (const auto& est : all_pose_estimates_) {
  //     std::cout << est.first;
  //     std::cout << est.second;
  //   }
  //   std::cout << "Used estimates: " << std::endl;
  //   for (const position_estimate_t& est : best_solution) {
  //     std::cout << est;
  //   }
  // }
  avg_variance /= best_solution.size();
  const int num_tags = tag_ids.size();
  position_estimate_t averaged_estimate = {
      .pose = WeightedAveragePose(best_solution),
      .variance = avg_variance,
      .timestamp = avg_timestamp,
      .num_tags = num_tags,
      .invalid = invalid};
  everything_timer.Stop();
  if (log_interesting_timestamp_) {
    std::cout << "Timestamp" << avg_timestamp
              << "Num estimates used: " << best_solution.size() << std::endl;
  }
  return {.pose_estimate = averaged_estimate, .latency = 0};
}

}  // namespace localization
