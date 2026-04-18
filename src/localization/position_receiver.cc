#include "src/localization/position_receiver.h"
#include <frc/geometry/Pose3d.h>

namespace localization {

PositionReceiver::PositionReceiver() {
  auto instance = nt::NetworkTableInstance::GetDefault();
  auto pose_table = instance.GetTable("/Pose");
  nt::StructTopic<frc::Pose3d> pose3d_topic =
      pose_table->GetStructTopic<frc::Pose3d>("robotPose");
  pose3d_subscriber_ = pose3d_topic.Subscribe({});
}

auto PositionReceiver::Get() -> frc::Pose3d {
  return pose3d_subscriber_.Get();
}

}  // namespace localization
