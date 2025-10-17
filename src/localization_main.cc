#include <frc/DataLogManager.h>
#include <networktables/NetworkTableInstance.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
#include "camera/imx296_camera.h"
#include "localization/position_sender.h"
#include "localization/tag_estimator.h"
#include "src/localization/pose_estimator.h"

using json = nlohmann::json;

#define STREAM_VIDEO false

void start_networktables() {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StopClient();
  inst.StopLocal();
  inst.StartClient4("orin_localization");

  inst.SetServerTeam(971);
  frc::DataLogManager::Start("/bos/logs/");
  std::cout << "Started networktables!" << std::endl;
}

void run_estimator(camera::CameraInfo camera_info,
                   localization::PositionSender& position_sender) {

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

  localization::TagEstimator tag_estimator(intrinsics, extrinsics);
  camera::IMX296Camera camera(camera_info);

  cv::Mat frame;
  while (true) {
    camera.GetFrame(frame);
    std::vector<localization::tag_detection_t> estimates =
        tag_estimator.Estimate(frame);
    position_sender.Send(estimates);
  }
}

int main() {

  start_networktables();

  localization::PositionSender position_sender(false);

  std::thread camera_one_thread(run_estimator, camera::gstreamer1_30fps,
                                std::ref(position_sender));

  std::thread camera_two_thread(run_estimator, camera::gstreamer2_30fps,
                                std::ref(position_sender));
  camera_one_thread.join();
  // camera_two_thread.join();

  return 0;
}
