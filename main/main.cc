#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
#include "camera/camera.h"
#include "localization/pose_estimator.h"
#include "localization/position_sender.h"
#include "wpilibc/frc/RuntimeType.h"

using json = nlohmann::json;

void start_networktables() {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StartClient4("orin");
  inst.SetServerTeam(971);
  std::cout << "Started networktables!" << std::endl;
}

void run_camera1(Camera::CameraInfo camera_info) {
  std::cout << "Starting camera 1" << std::endl;
  Camera::Camera camera(camera_info);

  json intrinsics;
  std::ifstream intrinsics_file(camera_info.intrinsics_path);
  if (!intrinsics_file.is_open()) {
    std::cerr << "Error: Cannot open intrinsics file: "
              << camera_info.intrinsics_path << std::endl;
  } else {
    intrinsics_file >> intrinsics;
  }

  json extrinsics;
  std::ifstream extrinsics_file(camera_info.extrinsics_path);
  if (!extrinsics_file.is_open()) {
    std::cerr << "Error: Cannot open extrinsics file: "
              << camera_info.extrinsics_path << std::endl;
  } else {
    extrinsics_file >> extrinsics;
  }

  PoseEstimator::PoseEstimator estimator(intrinsics, extrinsics);
  PositionSender sender({1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13,
                         14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25});

  cv::Mat frame;
  while (true) {
    camera.getFrame(frame);
    std::vector<PoseEstimator::position_estimate_t> estimates =
        estimator.Estimate(frame);
    sender.Send(estimates);
  }
}

int main() {

  start_networktables();

  std::thread camera_one_thread(run_camera1, Camera::CAMERAS.gstreamer1_30fps);
  camera_one_thread.join();

  return 0;
}
