#include "src/localization/run_localization.h"
#include <utility>
#include "src/camera/cscore_streamer.h"
#include "src/localization/get_field_relitive_position.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/position_sender.h"
#include "src/localization/position_solver.h"
#include "src/utils/camera_utils.h"
#include "src/utils/timer.h"

namespace localization {

void run_localization(camera::CameraSource& source,
                      std::unique_ptr<localization::IAprilTagDetector> detector,
                      std::unique_ptr<localization::IPositionSolver> solver,
                      const std::string& extrinsics, uint port, bool verbose) {

  localization::PositionSender position_sender(source.GetName(), verbose);

  camera::CscoreStreamer streamer(source.GetName(), port, 30, 1080, 1080);

  while (true) {
    utils::Timer timer(source.GetName(), verbose);
    camera::timestamped_frame_t timestamped_frame = source.Get();
    streamer.WriteFrame(timestamped_frame.frame);
    std::vector<localization::tag_detection_t> tag_detections =
        detector->GetTagDetections(timestamped_frame);
    std::vector<position_estimate_t> position_estimates =
        solver->EstimatePosition(tag_detections);
    position_sender.Send(position_estimates, timer.Stop());
  }
}
}  // namespace localization
