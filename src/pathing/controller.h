#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <cstdint>

namespace pathing {
class Controller {
 public:
  Controller();
  void Send();

 private:
  nt::NetworkTableInstance instance_;
  nt::StructSubscriber<frc::Pose2d> current_pose_sub_;
  nt::StructSubscriber<frc::Pose2d> target_pose_sub_;

  int64_t kDtUs = 20'000;
  double ax;
  double ay;
};

}  // namespace pathing
