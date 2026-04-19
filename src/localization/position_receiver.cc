#include "src/localization/position_receiver.h"
#include <frc/geometry/Pose3d.h>
#include <networktables/DoubleArrayTopic.h>
#include <units/length.h>

namespace localization {

PositionReceiver::PositionReceiver() {
  auto instance = nt::NetworkTableInstance::GetDefault();
  // auto pose_table = instance.GetTable("/Orin/PoseEstimate/main_bot_left");
  // nt::StructTopic<frc::Pose3d> pose3d_topic =
  //     pose_table->GetStructTopic<frc::Pose3d>("Pose3d");
  // pose3d_subscriber_ = pose3d_topic.Subscribe({});

  auto pose_table = instance.GetTable("/Pose");
  nt::DoubleArrayTopic pose3d_topic =
      pose_table->GetDoubleArrayTopic("robotPose");
  auto pose3d_subscriber_ = pose3d_topic.Subscribe({});
}

auto PositionReceiver::Get() -> frc::Pose2d {
  frc::Pose2d pose(units::meter_t{pose3d_subscriber_.Get()[0]},
                   units::meter_t{pose3d_subscriber_.Get()[1]},
                   frc::Rotation2d());

  return pose;
}

}  // namespace localization