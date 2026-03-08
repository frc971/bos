#include "src/localization/run_localization.h"
#include <utility>
#include "src/camera/cscore_streamer.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/joint_solver.h"
#include "src/localization/position_receiver.h"
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
                   uint port, bool verbose) {
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
  localization::PositionReceiver position_receiver;
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
        solver.EstimatePosition(tag_detections, position_receiver.Get(), true);
    position_sender.Send(std::vector<position_estimate_t>{position_estimate},
                         timer.Stop());
  }
}
}  // namespace localization
