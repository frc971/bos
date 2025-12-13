#include "src/localization/tag_estimator.h"
#include <fstream>
#include <sstream>
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"

using json = nlohmann::json;

int main() {
  // TODO no more imx296 streamer
  camera::CscoreStreamer streamer(
      camera::IMX296Streamer("tag_estimator_test", 4971, 30));

  auto camera = camera::SelectCamera();
  camera::CVCamera cap(
      cv::VideoCapture(camera::camera_constants[camera].pipeline));
  cv::Mat frame;
  cap.GetFrame(frame);

  localization::TagEstimator tag_estimator(
      frame.cols, frame.rows, camera::camera_constants[camera].intrinsics_path,
      camera::camera_constants[camera].extrinsics_path);
  while (true) {
    cap.GetFrame(frame);
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
