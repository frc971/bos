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

void RunLocalization(camera::CameraSource& source,
                     std::unique_ptr<localization::IAprilTagDetector> detector,
                     std::unique_ptr<localization::IPositionSolver> solver,
                     uint port, bool verbose) {
  const std::string& camera_name =
      camera::camera_constants[source.GetCameraConfig()].name;
  localization::PositionSender position_sender(camera_name, verbose);

  camera::CscoreStreamer streamer(camera_name, port, 30, 1080, 1080);

  while (true) {
    utils::Timer timer(camera_name, verbose);
    camera::timestamped_frame_t timestamped_frame = source.Get();
    streamer.WriteFrame(timestamped_frame.frame);
    std::vector<localization::tag_detection_t> tag_detections =
        detector->GetTagDetections(timestamped_frame);
    std::vector<position_estimate_t> position_estimates =
        solver->EstimatePosition(tag_detections, false);
    position_sender.Send(position_estimates, timer.Stop());
  }
}

void RunJointSolve(std::vector<camera::CameraSource>& sources,
                   std::unique_ptr<localization::IAprilTagDetector> detector,
                   uint port, bool square_solve_start, bool verbose) {
  std::string name = "";
  std::vector<camera::Camera> camera_constants;
  std::vector<camera::CscoreStreamer> streamers;
  streamers.reserve(sources.size());
  for (const camera::CameraSource& camera : sources) {
    const std::string& camera_name =
        camera::camera_constants[camera.GetCameraConfig()].name;
    name += ", " + camera_name;
    streamers.emplace_back(camera_name, port, 30, 1080, 1080);
    camera_constants.push_back(camera.GetCameraConfig());
  }
  localization::PositionSender position_sender(name, verbose);
  // TODO do this with position receiver
  frc::Pose3d prev_estimate = GetSquareSolveEstimates(sources, detector);
  JointSolver solver(camera_constants);
  while (true) {
    utils::Timer timer(name, verbose);
    std::map<camera::Camera, std::vector<localization::tag_detection_t>>
        tag_detections;
    for (int i = 0; i < sources.size(); i++) {
      camera::timestamped_frame_t timestamped_frame = sources[i].Get();
      streamers[i].WriteFrame(timestamped_frame.frame);
      std::vector<tag_detection_t> detections;
      for (tag_detection_t& detection :
           detector->GetTagDetections(timestamped_frame)) {
        detections.push_back(detection);
      }
      tag_detections.insert({camera_constants[i], detections});
    }
    position_estimate_t position_estimate =
        solver.EstimatePosition(tag_detections, prev_estimate, true);
    prev_estimate = position_estimate.pose;
    position_sender.Send(std::vector<position_estimate_t>{position_estimate},
                         timer.Stop());
  }
}

auto GetSquareSolveEstimates(
    std::vector<camera::CameraSource>& sources,
    std::unique_ptr<localization::IAprilTagDetector>& detector) -> frc::Pose3d {
  double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
  int num_estimates = 0;
  for (camera::CameraSource& source : sources) {
    localization::SquareSolver square_solver(source.GetCameraConfig());
    camera::timestamped_frame_t frame = source.Get();
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
