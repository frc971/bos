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

constexpr double RadianToDegree(double radian) {
  return radian * (180 / M_PI);
}

PositionSender::PositionSender(std::string camera_name, bool verbose)
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

void PositionSender::Send(std::vector<localization::tag_detection_t> detections,
                          double latency) {
  if (mutex_.try_lock()) {
    for (size_t i = 0; i < detections.size(); i++) {
      double variance = detections[i].distance;
      double tag_estimation[7] = {
          detections[i].pose.X().value(),
          detections[i].pose.Y().value(),
          detections[i].pose.Rotation().Z().value(),
          variance,
          detections[i].timestamp +
              instance_.GetServerTimeOffset().value_or(0) / 1000000.0,
          static_cast<double>(detections[i].tag_id),
          latency};

      pose_publisher_.Set(frc::Pose2d(
          units::meter_t{detections[i].pose.X().value()},
          units::meter_t{detections[i].pose.Y().value()},
          units::radian_t{detections[i].pose.Rotation().Z().value()}));

      tag_estimation_publisher_.Set(tag_estimation);
      latency_publisher_.Set(latency);
    }

    mutex_.unlock();
  }
}
}  // namespace localization
