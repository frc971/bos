#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
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

void run_estimator(const int frame_width, const int frame_height,
                   std::unique_ptr<camera::CVCamera> cap, json intrinsics,
                   json extrinsics,
                   localization::PositionSender& position_sender, bool rotate) {

  localization::TagEstimator tag_estimator(frame_width, frame_height,
                                           intrinsics, extrinsics);

  camera::CscoreStreamer streamer(
      camera::IMX296Streamer("frame_logger", 4971, 30));

  cv::Mat frame;
  while (true) {
    cap->GetFrame(frame);
    streamer.WriteFrame(frame);
    std::vector<localization::tag_detection_t> estimates =
        tag_estimator.Estimate(frame);
    position_sender.Send(estimates);
  }
}

int main() {

  start_networktables();

  localization::PositionSender position_sender(true);

  std::thread usb0_thread(
      run_estimator, 640, 480,
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::USB0].pipeline)),
      read_intrinsics(
          camera::camera_constants[camera::Camera::USB0].intrinsics_path),
      read_extrinsics(
          camera::camera_constants[camera::Camera::USB0].extrinsics_path),
      std::ref(position_sender), true);

  std::thread usb1_thread(
      run_estimator, 1280, 720,
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::USB1].pipeline)),
      read_intrinsics(
          camera::camera_constants[camera::Camera::USB1].intrinsics_path),
      read_extrinsics(
          camera::camera_constants[camera::Camera::USB1].extrinsics_path),
      std::ref(position_sender), false);

  usb1_thread.join();

  return 0;
}
