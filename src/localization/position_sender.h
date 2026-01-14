#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include "src/localization/position.h"

namespace localization {
class PositionSender {
 public:
  PositionSender(const std::string& camera_name, bool verbose = false);
  void Send(const std::vector<localization::position_estimate_t>& detections,
            double latency);

 private:
  nt::NetworkTableInstance instance_;

  nt::StructPublisher<frc::Pose2d> pose_publisher_;
  nt::DoublePublisher latency_publisher_;
  nt::DoubleArrayPublisher tag_estimation_publisher_;

  std::mutex mutex_;
  bool verbose_;
};
}  // namespace localization
