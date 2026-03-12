#include "src/localization/run_localization.h"
#include <utility>
#include "src/camera/cscore_streamer.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/position_sender.h"
#include "src/localization/position_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/timer.h"

namespace localization {

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
}  // namespace localization
