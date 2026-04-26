#pragma once

#include <frc/geometry/Pose3d.h>
#include <networktables/StructTopic.h>
#include "src/utils/pch.h"

namespace localization {
// TODO Use velocity to interpolate position more accurately
class PositionReceiver {
 public:
  PositionReceiver();
  auto Get() -> frc::Pose2d;

 private:
  nt::StructSubscriber<frc::Pose2d> pose_subscriber_;
};
}  // namespace localization
