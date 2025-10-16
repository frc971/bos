#pragma once

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include "src/localization/position.h"
#include "tag_estimator.h"

namespace localization {
class PositionSender {
 public:
  PositionSender(bool verbose = false);
  void Send(pose2d_t position_estimate, pose2d_t variance);
  void Send(std::vector<localization::tag_detection_t> detections);

 private:
  nt::NetworkTableInstance instance_;

  nt::StructPublisher<frc::Pose2d> pose_publisher_;
  nt::DoubleArrayPublisher tag_estimation_publisher_;

  std::mutex mutex_;
  bool verbose_;
};
}  // namespace localization
