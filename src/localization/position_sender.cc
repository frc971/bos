#include "position_sender.h"
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <wpilibc/frc/Timer.h>
#include <cmath>
#include <string>
#include "frc/DataLogManager.h"
#include "src/localization/position.h"

namespace localization {

constexpr auto RadianToDegree(double radian) -> double {
  return radian * (180 / M_PI);
}

PositionSender::PositionSender(const std::string& camera_name, bool verbose)
    : instance_(nt::NetworkTableInstance::GetDefault()), verbose_(verbose) {
  std::shared_ptr<nt::NetworkTable> table =
      instance_.GetTable("Orin/PoseEstimate/" + camera_name);

  nt::StructTopic<frc::Pose2d> pose_topic =
      table->GetStructTopic<frc::Pose2d>("Pose");
  pose_publisher_ = pose_topic.Publish();

  nt::DoubleTopic latency_topic = table->GetDoubleTopic("Latency");
  latency_publisher_ = latency_topic.Publish();

  nt::DoubleArrayTopic tag_estimation_topic =
      table->GetDoubleArrayTopic("TagEstimation");
  tag_estimation_publisher_ = tag_estimation_topic.Publish(
      {.periodic = 0.01, .sendAll = true, .keepDuplicates = true});
}

void PositionSender::Send(
    const std::vector<localization::tag_detection_t>& detections,
    double latency) {
  if (mutex_.try_lock()) {
    for (auto& detection : detections) {
      double variance = detection.distance;
      std::array<double, 7> tag_estimation{
          detection.pose.X().value(),
          detection.pose.Y().value(),
          detection.pose.Rotation().Z().value(),
          variance,
          detection.timestamp +
              instance_.GetServerTimeOffset().value_or(0) / 1000000.0,
          static_cast<double>(detection.tag_id),
          latency};

      pose_publisher_.Set(
          frc::Pose2d(units::meter_t{detection.pose.X().value()},
                      units::meter_t{detection.pose.Y().value()},
                      units::radian_t{detection.pose.Rotation().Z().value()}));

      tag_estimation_publisher_.Set(tag_estimation);
      latency_publisher_.Set(latency);
    }

    mutex_.unlock();
  }
}
}  // namespace localization
