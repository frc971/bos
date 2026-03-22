#include "src/localization/run_localization.h"
#include <frc/DataLogManager.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#include <wpi/DataLog.h>
#include <wpi/DataLogWriter.h>
#include <utility>
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/position_sender.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
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

void RunLocalizationSimulation(
    camera::CameraSource& source,
    std::unique_ptr<localization::IAprilTagDetector> detector,
    std::unique_ptr<localization::IPositionSolver> solver,
    const std::string& extrinsics, std::optional<uint> port, bool verbose) {
  std::error_code ec;
  auto log = std::make_unique<wpi::log::DataLogWriter>("multitag.wpilog", ec);
  if (ec) {
    std::cerr << "Failed to open log: " << ec.message() << std::endl;
    return;
  }
  log->AddStructSchema<frc::Translation3d>(0);
  log->AddStructSchema<frc::Rotation3d>(0);
  log->AddStructSchema<frc::Pose3d>(0);
  wpi::log::StructLogEntry<frc::Pose3d> pose_log(*log, "/localization/pose");
  wpi::log::DoubleLogEntry num_tags_log(*log, "/localization/num_tags");
  wpi::log::DoubleLogEntry timestamp_log(*log, "/localization/timestamp");
  while (true) {
    camera::timestamped_frame_t timestamped_frame = source.Get();
    double timestamp = timestamped_frame.timestamp;
    if (timestamped_frame.invalid) {
      std::cout << "Stopping log" << std::endl;
      log->Stop();
      std::cout << "Stopped log" << std::endl;
      return;
    }
    std::cout << "Reading from timestamp: " << timestamp << std::endl;
    std::vector<localization::tag_detection_t> tag_detections =
        detector->GetTagDetections(timestamped_frame);
    std::vector<position_estimate_t> position_estimates =
        solver->EstimatePosition(tag_detections, false);
    auto log_time = static_cast<int64_t>(timestamp * 1e6);
    for (const auto& estimate : position_estimates) {
      pose_log.Append(estimate.pose, log_time);
      num_tags_log.Append(estimate.num_tags, log_time);
      timestamp_log.Append(estimate.timestamp, log_time);
    }
  }
}

}  // namespace localization
