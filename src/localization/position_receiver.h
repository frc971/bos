#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StructTopic.h>
#include "src/utils/pch.h"

namespace localization {
// TODO Use velocity to interpolate position more accurately
class PositionReceiver {
 public:
  PositionReceiver();
  auto Get() -> frc::Pose2d;

 private:
  nt::StructSubscriber<frc::Pose2d> pose2d_subscriber_;

  std::mutex mutex_;
};
}  // namespace localization