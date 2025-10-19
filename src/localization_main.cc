#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
#include "camera/imx296_camera.h"
#include "localization/position_sender.h"
#include "localization/tag_estimator.h"
#include "src/camera/camera_constants.h"
#include "src/camera/cscore_streamer.h"
#include "src/camera/cv_camera.h"

using json = nlohmann::json;

void start_networktables() {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StopClient();
  inst.StopLocal();
  inst.StartClient4("orin_localization");
  inst.SetServerTeam(971);
  frc::DataLogManager::Start("/bos/logs/");
  std::cout << "Started networktables!" << std::endl;
}

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

void run_estimator(std::unique_ptr<camera::CVCamera> camera, json intrinsics,
                   json extrinsics,
                   localization::PositionSender& position_sender) {

  localization::TagEstimator tag_estimator(1280, 720, intrinsics, extrinsics);

  camera::CscoreStreamer streamer(
      camera::IMX296Streamer("frame_logger", 4971, 30));

  cv::Mat frame;
  while (true) {
    camera->GetFrame(frame);
    std::cout << frame.size[0] << " " << frame.size[1] << " " << std::endl;
    std::cout << frame.channels() << std::endl;
    streamer.WriteFrame(frame);
    std::vector<localization::tag_detection_t> estimates =
        tag_estimator.Estimate(frame);
    std::cout << estimates.size() << std::endl;
    position_sender.Send(estimates);
  }
}

int main() {

  start_networktables();

  localization::PositionSender position_sender(false);

  // std::thread camera_one_thread(run_estimator,
  //                               std::make_unique<camera::CVCamera>(
  //                                   cv::VideoCapture(camera::gstreamer1_30fps)),
  //                               read_intrinsics(camera::camera1_intrinsics),
  //                               read_extrinsics(camera::camera1_extrinsics),
  //                               std::ref(position_sender));

  std::thread camera_two_thread(
      run_estimator,
      std::make_unique<camera::CVCamera>(cv::VideoCapture("/dev/video2")),
      read_intrinsics(camera::camera1_intrinsics),
      read_extrinsics(camera::camera1_extrinsics), std::ref(position_sender));

  // camera_one_thread.join();
  camera_two_thread.join();

  return 0;
}
