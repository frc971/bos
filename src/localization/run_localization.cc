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
#include "src/localization/networktable_sender.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/position_sender.h"
#include "src/localization/position_solver.h"
#include "src/localization/square_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/timer.h"

namespace localization {

void RunLocalization(
    std::unique_ptr<camera::CameraSource> source,
    std::unique_ptr<localization::IAprilTagDetector> detector,
    std::unique_ptr<localization::IPositionSolver> solver,
    std::vector<std::unique_ptr<localization::IPositionSender>> senders,
    std::optional<uint> port, bool verbose) {

  std::optional<camera::CscoreStreamer> streamer =
      port.has_value() ? std::make_optional(camera::CscoreStreamer(
                             source->GetName(), port.value(), 30, 1080, 1080))
                       : std::nullopt;

  while (true) {
    utils::Timer timer(source->GetName(), verbose);
    camera::timestamped_frame_t timestamped_frame = source->Get();
    if (streamer.has_value()) {
      streamer->WriteFrame(timestamped_frame.frame);
    }
    std::vector<localization::tag_detection_t> tag_detections =
        detector->GetTagDetections(timestamped_frame);
    std::vector<position_estimate_t> position_estimates =
        solver->EstimatePosition(tag_detections, false);
    const double latency = timer.Stop();
    for (auto& position_estimate : position_estimates) {
      position_estimate.latency = latency;
    }
    for (auto& s : senders) {
      s->Send(position_estimates);
    }
  }
}

void RunJointLocalization(
    MultiCameraDetector& detector_source,
    std::unique_ptr<localization::IJointPositionSolver> solver,
    std::unique_ptr<localization::IPositionSender> sender, bool verbose) {
  while (true) {
    auto detections = detector_source.GetTagDetections();
    std::optional<position_estimate_t> estimated_pose =
        solver->EstimatePosition(detections);
    if (!estimated_pose.has_value()) {
      continue;
    }
    sender->Send(estimated_pose.value());
  }
}

}  // namespace localization
