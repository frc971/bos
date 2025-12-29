#include "src/localization/run_localization.h"
#include "src/camera/cscore_streamer.h"
#include "src/localization/get_field_relitive_position.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/position_sender.h"
#include "src/utils/camera_utils.h"
#include "src/utils/timer.h"

namespace localization {
void run_localization(const int frame_width, const int frame_height,
                      camera::CameraSource& source, std::string intrinsics,
                      std::string extrinsics, uint port, bool verbose) {

  localization::GPUAprilTagDetector detector(
      frame_width, frame_height, utils::read_intrinsics(intrinsics));
  localization::PositionSender position_sender(source.GetName());

  camera::CscoreStreamer streamer(source.GetName(), port, 30, 1080, 1080);

  nlohmann::json extrinsics_json = utils::read_extrinsics(extrinsics);
  frc::Transform3d camera_to_robot =
      localization::ExtrinsicsJsonToCameraToRobot(extrinsics_json);
  while (true) {
    utils::Timer timer(source.GetName(), verbose);
    camera::timestamped_frame_t timestamped_frame = source.Get();
    streamer.WriteFrame(timestamped_frame.frame);
    std::vector<localization::tag_detection_t> estimates =
        localization::GetFeildRelitivePosition(
            detector.GetTagDetections(timestamped_frame), camera_to_robot);
    position_sender.Send(estimates, timer.Stop());
  }
}
}  // namespace localization
