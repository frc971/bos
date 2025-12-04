#include "src/localization/tag_estimator.h"
#include <fstream>
#include <sstream>
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/select_camera.h"

using json = nlohmann::json;

json read_intrinsics(std::string path) {
  json intrinsics;

  std::ifstream intrinsics_file(path);
  if (!intrinsics_file.is_open()) {
    std::cerr << "Error: Cannot open intrinsics file: " << path << std::endl;
  } else {
    intrinsics_file >> intrinsics;
  }
  return intrinsics;
}

json read_extrinsics(std::string path) {
  json extrinsics;
  std::ifstream extrinsics_file(path);
  if (!extrinsics_file.is_open()) {
    std::cerr << "Error: Cannot open extrinsics file: " << path << std::endl;
  } else {
    extrinsics_file >> extrinsics;
  }
  return extrinsics;
}

int main() {
  camera::CscoreStreamer streamer(
      camera::IMX296Streamer("tag_estimator_test", 4971, 30));

  auto camera = camera::SelectCamera();
  camera::CVCamera cap(
      cv::VideoCapture(camera::camera_constants[camera].pipeline));
  cv::Mat frame;
  cap.GetFrame(frame);

  localization::TagEstimator tag_estimator(
      frame.cols, frame.rows,
      read_intrinsics(camera::camera_constants[camera].intrinsics_path),
      read_extrinsics(camera::camera_constants[camera].extrinsics_path));
  while (true) {
    cap.GetFrame(frame);
    streamer.WriteFrame(frame);
    std::vector<localization::tag_detection_t> estimates =
        tag_estimator.GetRawPositionEstimates(frame, 0);
    for (auto& estimate : estimates) {
      std::cout << estimate;
    }
    std::cout << "----------" << std::endl;
  }
}
