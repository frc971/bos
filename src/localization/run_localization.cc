#include "src/localization/run_localization.h"
#include <utility>
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/joint_solver.h"
#include "src/localization/position_sender.h"
#include "src/localization/position_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/timer.h"

namespace localization {

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
    std::unique_ptr<localization::IAprilTagDetector> detector, uint port,
    bool square_solve_start, bool verbose) {
  std::vector<camera::CameraConstant> camera_configs;
  std::string name = "";
  std::vector<camera::CscoreStreamer> streamers;
  streamers.reserve(camera_sources.size());
  for (const auto& camera : camera_sources) {
    name += ", " + camera.second->GetName();
    camera_configs.push_back(camera.first);
    streamers.emplace_back(camera.second->GetName(), port, 30, 1080, 1080);
  }
  localization::PositionSender position_sender(name, verbose);
  // TODO do this with position receiver
  frc::Pose3d prev_estimate = GetSquareSolveEstimates(camera_sources, detector);
  camera_configs.reserve(camera_sources.size());
  JointSolver solver(camera_configs);
  while (true) {
    utils::Timer timer(name, verbose);
    std::map<camera::CameraConstant, std::vector<localization::tag_detection_t>>
        tag_detections;
    for (int i = 0; i < camera_sources.size(); i++) {
      camera::timestamped_frame_t timestamped_frame =
          camera_sources[i].second->Get();
      streamers[i].WriteFrame(timestamped_frame.frame);
      std::vector<tag_detection_t> detections;
      for (tag_detection_t& detection :
           detector->GetTagDetections(timestamped_frame)) {
        detections.push_back(detection);
      }
      tag_detections.insert({camera_configs[i], detections});
    }
    position_estimate_t position_estimate =
        solver.EstimatePosition(tag_detections, prev_estimate, true);
    prev_estimate = position_estimate.pose;
    position_sender.Send(std::vector<position_estimate_t>{position_estimate},
                         timer.Stop());
  }
}

auto GetSquareSolveEstimates(
    std::vector<std::pair<camera::CameraConstant,
                          std::unique_ptr<camera::CameraSource>>>&
        camera_sources,
    std::unique_ptr<localization::IAprilTagDetector>& detector) -> frc::Pose3d {
  double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
  int num_estimates = 0;
  for (auto& camera_source : camera_sources) {
    localization::SquareSolver square_solver(camera_source.first);
    camera::timestamped_frame_t frame = camera_source.second->Get();
    const std::vector<localization::position_estimate_t> estimates =
        square_solver.EstimatePosition(detector->GetTagDetections(frame));
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
