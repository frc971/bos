#include <networktables/NetworkTableInstance.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
#include "camera/imx296_camera.h"
#include "localization/position_sender.h"
#include "localization/tag_estimator.h"

using json = nlohmann::json;

#define STREAM_VIDEO false

void start_networktables() {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StartClient4("orin_localization");
  inst.SetServerTeam(971);
  std::cout << "Started networktables!" << std::endl;
}

void run_camera1(Camera::CameraInfo camera_info) {
  std::cout << "Starting camera 1" << std::endl;
  Camera::IMX296Camera camera(camera_info);

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

  Localization::TagEstimator estimator(intrinsics, extrinsics);
  PositionSender sender(camera_info.name,
                        {1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,
                         12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22});

  cv::Mat frame;
  while (true) {
    camera.getFrame(frame);
    std::vector<Localization::tag_detection_t> estimates =
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
