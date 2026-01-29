#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include "src/utils/pch.h"

namespace pathing {
class VelocitySender {
 public:
  VelocitySender();
  void Send(const double ax, const double ay, double latency);

 private:
  nt::NetworkTableInstance instance_;

  nt::DoubleArrayPublisher vel_publisher_;
  nt::DoublePublisher latency_publisher_;

  std::mutex mutex_;
};
}  // namespace pathing