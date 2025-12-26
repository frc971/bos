#include "src/localization/tag_estimator.h"
#include <fstream>
#include <sstream>
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"

using json = nlohmann::json;

int main() {
  camera::CscoreStreamer streamer("tag_estimator_test", 4971, 30, 1080, 1080);

  camera::Camera config = camera::SelectCameraConfig();
  std::unique_ptr<camera::ICamera> camera = camera::GetCameraStream(config);
  cv::Mat frame;
  camera->GetFrame(frame);

  localization::TagEstimator tag_estimator(
      frame.cols, frame.rows, camera::camera_constants[config].intrinsics_path,
      camera::camera_constants[config].extrinsics_path);
  while (true) {
    camera->GetFrame(frame);
    streamer.WriteFrame(frame);
    std::vector<localization::tag_detection_t> estimates =
        tag_estimator.GetRawPositionEstimates(frame, 0);
    for (auto& estimate : estimates) {
      std::cout << estimate;
    }
    if (!estimates.empty()) {
      std::cout << "----------" << std::endl;
    }
  }
}
