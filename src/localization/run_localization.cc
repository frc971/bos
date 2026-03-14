#include "src/localization/run_localization.h"
#include <utility>
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/joint_solver.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/position_sender.h"
#include "src/localization/position_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/timer.h"

namespace localization {

auto PrintTagDetections = [](const auto& detections_map) {
  for (const auto& [camera_const, detections] : detections_map) {
    std::cout << "Camera: " << camera_const << "\n";
    for (const auto& det : detections) {
      std::cout << "  " << det << "\n";
    }
  }
};

// TODO remove extrinsics
void RunLocalization(camera::CameraSource& source,
                     std::unique_ptr<localization::IAprilTagDetector> detector,
                     std::unique_ptr<localization::IPositionSolver> solver,
                     const std::string& extrinsics, std::optional<uint> port,
                     bool verbose) {
  localization::PositionSender position_sender(source.GetName(), verbose);

  std::optional<camera::CscoreStreamer> streamer =
      port.has_value() ? std::make_optional(camera::CscoreStreamer(
                             source.GetName(), port.value(), 30, 1080, 1080))
                       : std::nullopt;

  while (true) {
    utils::Timer timer(source.GetName(), verbose);
    camera::timestamped_frame_t timestamped_frame = source.Get();
    if (streamer.has_value()) {
      streamer->WriteFrame(timestamped_frame.frame);
    }
    std::vector<localization::tag_detection_t> tag_detections =
        detector->GetTagDetections(timestamped_frame);
    std::vector<position_estimate_t> position_estimates =
        solver->EstimatePosition(tag_detections, false);
    position_sender.Send(position_estimates, timer.Stop());
  }
}

void RunJointSolve(
    std::vector<std::pair<camera::CameraConstant,
                          std::unique_ptr<camera::CameraSource>>>&
        camera_sources,
    uint port, bool square_solve_start, bool verbose) {
  std::vector<camera::CameraConstant> camera_configs;
  std::string name = "";
  std::vector<camera::CscoreStreamer> streamers;
  streamers.reserve(camera_sources.size());
  std::vector<localization::OpenCVAprilTagDetector> detectors;
  detectors.reserve(camera_sources.size());
  for (size_t i = 0; i < camera_sources.size(); i++) {
    name += ", " + camera_sources[i].second->GetName();
    camera_configs.push_back(camera_sources[i].first);
    streamers.emplace_back(camera_sources[i].second->GetName(), port + i, 30,
                           1080, 1080);
    detectors.emplace_back(
        camera_sources[0].second->GetFrame().cols,
        camera_sources[0].second->GetFrame().rows,
        utils::ReadIntrinsics(camera_sources[i].first.intrinsics_path.value()));
  }
  // localization::PositionSender position_sender(name, verbose);
  // TODO do this with position receiver
  frc::Pose3d prev_estimate =
      GetSquareSolveEstimates(camera_sources, detectors);
  std::vector<frc::Pose3d> estimates_over_time;
  camera_configs.reserve(camera_sources.size());
  JointSolver solver(camera_configs);
  double last_timestamp = -1;
  while (true) {
    size_t num_detections = 0;
    std::map<camera::CameraConstant, std::vector<localization::tag_detection_t>>
        tag_detections;
    double timestamp = -1;
    for (int i = 0; i < camera_sources.size(); i++) {
      camera::timestamped_frame_t timestamped_frame =
          camera_sources[i].second->Get(true);
      // streamers[i].WriteFrame(timestamped_frame.frame);
      std::vector<tag_detection_t> detections;
      for (tag_detection_t& detection :
           detectors[i].GetTagDetections(timestamped_frame)) {
        // if (detection.tag_id == 26 || detection.tag_id == 25) {
        detections.push_back(detection);
        num_detections++;
        timestamp = detection.timestamp;
        // }
      }
      tag_detections.insert({camera_configs[i], detections});
    }
    if (num_detections == 0 || timestamp < 11.9) {
      continue;
    }
    if (timestamp == last_timestamp) {
      continue;
    }
    std::cout << "Considering" << timestamp << std::endl;
    last_timestamp = timestamp;
    // PrintTagDetections(tag_detections);
    if (timestamp > 13.6) {
      std::cout << "Finished" << std::endl;
      for (const auto& estimate : estimates_over_time) {
        utils::PrintPose3d(estimate);
      }
      std::exit(0);
    }
    utils::Timer timer(name, verbose);
    localization::position_estimate_t position_estimate =
        solver.EstimatePosition(tag_detections, prev_estimate, true, false);
    std::cout << "Solver took " << timer.Stop() << " seconds" << std::endl;
    // frc::Pose3d position_estimate =
    //     GetSquareSolveEstimates(camera_sources, detectors);
    prev_estimate = position_estimate.pose;
    // prev_estimate = position_estimate;
    estimates_over_time.push_back(prev_estimate);
    utils::PrintPose3d(prev_estimate);
    // position_sender.Send(std::vector<position_estimate_t>{position_estimate},
    //                      timer.Stop());
  }
}

auto GetSquareSolveEstimates(
    std::vector<std::pair<camera::CameraConstant,
                          std::unique_ptr<camera::CameraSource>>>&
        camera_sources,
    std::vector<localization::OpenCVAprilTagDetector>& detectors)
    -> frc::Pose3d {
  double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
  int num_estimates = 0;
  for (int i = 0; i < camera_sources.size(); i++) {
    localization::SquareSolver square_solver(camera_sources[i].first);
    camera::timestamped_frame_t frame = camera_sources[i].second->Get();
    const std::vector<localization::position_estimate_t> estimates =
        square_solver.EstimatePosition(detectors[i].GetTagDetections(frame));
    for (const localization::position_estimate_t& estimate : estimates) {
      num_estimates++;
      x += estimate.pose.X().value();
      y += estimate.pose.Y().value();
      z += estimate.pose.Z().value();
      roll += estimate.pose.Rotation().X().value();
      pitch += estimate.pose.Rotation().Y().value();
      yaw += estimate.pose.Rotation().Z().value();
    }
  }
  x /= num_estimates;
  y /= num_estimates;
  z /= num_estimates;
  roll /= num_estimates;
  pitch /= num_estimates;
  yaw /= num_estimates;
  return frc::Pose3d{
      frc::Translation3d{units::meter_t{x}, units::meter_t{y},
                         units::meter_t{z}},
      frc::Rotation3d{units::radian_t{roll}, units::radian_t{pitch},
                      units::radian_t{yaw}}};
}

}  // namespace localization
