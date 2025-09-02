#include "camera/camera.h"
#include "pose_estimator.h"
#include "position_sender.h"
#include <fstream>
#include "wpilibc/frc/RuntimeType.h"
#include "apriltag/apriltag.h"
#include <iostream>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/highgui.hpp>
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
  PositionSender sender({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25});

  cv::Mat frame;
  while (true) {
    camera.getFrame(frame);
    std::vector<PoseEstimator::position_estimate_t> estimates = estimator.Estimate(frame);
    sender.Send(estimates);
    cv::imshow("", frame);
    cv::waitKey(1);
  }

  return 0;
}
