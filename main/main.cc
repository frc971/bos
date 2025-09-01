#include "camera/camera.h"
#include "pose_estimator.h"
#include "position_sender.h"
#include "kalman_filter.h"
#include <fstream>
#include "wpilibc/frc/RuntimeType.h"
#include "apriltag/apriltag.h"
#include <iostream>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <sstream>

using json = nlohmann::json;

int main() {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  inst.StartClient4("orin");
  inst.SetServerTeam(971);

  Camera::Camera camera(Camera::CAMERAS.gstreamer1_30fps);

  json intrinsics;
  std::ifstream file("calibration/intrinsics.json");
  file >> intrinsics;

  PoseEstimator::PoseEstimator estimator(intrinsics, PoseEstimator::kapriltag_dimensions);
  PositionSender sender;

  cv::Mat frame;
  while (true) {
    camera.getFrame(frame);
    std::vector<PoseEstimator::position_estimate_t> estimates = estimator.Estimate(frame);
    sender.Send(estimates);
  }

  return 0;
}
