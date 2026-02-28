#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <atomic>
#include <cstdint>

namespace pathing {
class Controller {
 public:
  Controller();
  void Send();
  void Stop();

 private:
  nt::NetworkTableInstance instance_;
  nt::StructSubscriber<frc::Pose2d> current_pose_sub_;
  nt::StructSubscriber<frc::Pose2d> target_pose_sub_;
};

}  // namespace pathing
