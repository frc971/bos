#include <networktables/NetworkTableInstance.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include "apriltag/apriltag.h"
#include "camera/imx296_camera.h"
#include "localization/position_sender.h"
#include "localization/tag_estimator.h"
#include "main/localization/pose_estimator.h"
#include "mutex"

using json = nlohmann::json;

#define STREAM_VIDEO false

void start_networktables() {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StartClient4("orin_localization");
  inst.SetServerTeam(971);
  std::cout << "Started networktables!" << std::endl;
}

std::vector<Camera::IMX296Camera> cameras;
std::vector<Localization::TagEstimator> estimators;
std::mutex mtx;

void setup_camera(Camera::CameraInfo camera_info) {
  std::cout << "Starting camera " << camera_info.id << std::endl;
  cameras.push_back(Camera::IMX296Camera(camera_info));

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

 estimators.push_back(Localization::TagEstimator(intrinsics, extrinsics));
}

void run_estimator(Camera::IMX296Camera& camera, Localization::TagEstimator& camera_estimator, Localization::PoseEstimator& pose_estimator) {
  cv::Mat frame;
  while (true) {
    camera.getFrame(frame);
    std::vector<Localization::tag_detection_t> estimates =
        camera_estimator.Estimate(frame);
    mtx.lock();
    pose_estimator.Update(estimates);
    mtx.unlock();
    // sender.Send(estimates);
  }
}

int main() {

  start_networktables();

  Localization::SimpleKalmanConfig x_filter_config{.position = 0,
                                                   .velocity = 0,
                                                   .time = 0,
                                                   .measurment_noise = 0.5,
                                                   .process_noise = 0.5};

  Localization::SimpleKalmanConfig y_filter_config{.position = 0,
                                                   .velocity = 0,
                                                   .time = 0,
                                                   .measurment_noise = 0.5,
                                                   .process_noise = 0.5};

  Localization::SimpleKalmanConfig rotation_filter_config{
      .position = 0,
      .velocity = 0,
      .time = 0,
      .measurment_noise = 0.5,
      .process_noise = 0.5};
  Localization::PoseEstimator pose_estimator(x_filter_config, y_filter_config,
                                             rotation_filter_config);
  PositionSender sender;

  // std::thread camera_one_thread(run_camera1, Camera::CAMERAS.gstreamer1_30fps);
  // camera_one_thread.join();
  std::vector<std::thread> workers(cameras.size());
  for (int i = 0; i < cameras.size(); i++) {
    workers.at(i) = std::thread(run_estimator,
                            std::ref(cameras.at(i)),
                            std::ref(estimators.at(i)),
                            std::ref(pose_estimator));
  }
  for (int i = 0; i < cameras.size(); i++) {
    workers.at(i).join();
  }

  return 0;
}
