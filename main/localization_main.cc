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

using json = nlohmann::json;

#define STREAM_VIDEO false

void start_networktables() {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StartClient4("orin_localization");
  inst.SetServerTeam(971);
  std::cout << "Started networktables!" << std::endl;
}

void run_estimator(Camera::CameraInfo camera_info,
                   localization::PoseEstimator& pose_estimator,
                   localization::PositionSender& position_sender) {

  localization::TagEstimator tag_estimator(camera_info.intrinsics_path,
                                           camera_info.extrinsics_path);
  Camera::IMX296Camera camera(camera_info);

  cv::Mat frame;
  while (true) {
    camera.getFrame(frame);
    std::vector<localization::tag_detection_t> estimates =
        tag_estimator.Estimate(frame);
    pose_estimator.Update(estimates);
    position_sender.Send(pose_estimator.GetPose(),
                         pose_estimator.GetPoseVarience());
  }
}

int main() {

  start_networktables();

  localization::SimpleKalmanConfig x_filter_config{.position = 0,
                                                   .velocity = 0,
                                                   .time = 0,
                                                   .measurment_noise = 0.5,
                                                   .process_noise = 0.5};

  localization::SimpleKalmanConfig y_filter_config{.position = 0,
                                                   .velocity = 0,
                                                   .time = 0,
                                                   .measurment_noise = 0.5,
                                                   .process_noise = 0.5};
  localization::SimpleKalmanConfig rotation_filter_config{
      .position = 0,
      .velocity = 0,
      .time = 0,
      .measurment_noise = 0.5,
      .process_noise = 0.5};

  localization::PoseEstimator pose_estimator(x_filter_config, y_filter_config,
                                             rotation_filter_config);
  localization::PositionSender position_sender;

  std::thread camera_one_thread(run_estimator, Camera::CAMERAS.gstreamer1_30fps,
                                std::ref(pose_estimator),
                                std::ref(position_sender));

  std::thread camera_two_thread(run_estimator, Camera::CAMERAS.gstreamer2_30fps,
                                std::ref(pose_estimator),
                                std::ref(position_sender));

  return 0;
}
