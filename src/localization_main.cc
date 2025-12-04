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
#include "src/camera/camera_source.h"
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
                   camera::CameraSource& source, json intrinsics,
                   json extrinsics, uint port) {

  localization::TagEstimator tag_estimator(frame_width, frame_height,
                                           intrinsics, extrinsics);
  localization::PositionSender position_sender(source.GetName());

  camera::CscoreStreamer streamer(
      camera::IMX296Streamer(source.GetName(), port, 30));

  while (true) {
    camera::timestamped_frame_t timestamped_frame = source.Get();
    streamer.WriteFrame(timestamped_frame.frame);
    std::vector<localization::tag_detection_t> estimates =
        tag_estimator.Estimate(timestamped_frame.frame,
                               timestamped_frame.timestamp);
    position_sender.Send(estimates);
  }
}

int main() {

  start_networktables();

  camera::CameraSource back_left_camera(
      "back_left",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::USB0].pipeline)));

  camera::CameraSource back_right_camera(
      "back_right",
      std::make_unique<camera::CVCamera>(cv::VideoCapture(
          camera::camera_constants[camera::Camera::USB1].pipeline)));

  std::thread usb0_thread(
      run_estimator, 640, 480, std::ref(back_left_camera),
      read_intrinsics(
          camera::camera_constants[camera::Camera::USB0].intrinsics_path),
      read_extrinsics(
          camera::camera_constants[camera::Camera::USB0].extrinsics_path),
      4971);

  std::thread usb1_thread(
      run_estimator, 1280, 720, std::ref(back_right_camera),
      read_intrinsics(
          camera::camera_constants[camera::Camera::USB1].intrinsics_path),
      read_extrinsics(
          camera::camera_constants[camera::Camera::USB1].extrinsics_path),
      4971);

  usb1_thread.join();

  return 0;
}
