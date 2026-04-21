#include "src/localization/position_receiver.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/StructTopic.h>
#include <units/length.h>
#include "src/utils/log.h"

namespace localization {

PositionReceiver::PositionReceiver() {
  auto instance = nt::NetworkTableInstance::GetDefault();
  auto pose_table = instance.GetTable("DriveState");
  nt::StructTopic<frc::Pose2d> pose2d_topic =
      pose_table->GetStructTopic<frc::Pose2d>("Pose");
  pose2d_subscriber_ = pose2d_topic.Subscribe({});
}

auto PositionReceiver::Get() -> frc::Pose2d {
  auto pose = pose2d_subscriber_.Get();

  return pose;
}

}  // namespace localization