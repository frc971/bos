#include "position_sender.h"
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
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

  nt::DoubleArrayTopic tag_estimation_topic =
      table->GetDoubleArrayTopic("TagEstimation");
  tag_estimation_publisher_ = tag_estimation_topic.Publish(
      {.periodic = 0.01, .sendAll = true, .keepDuplicates = true});
}

void PositionSender::Send(
    std::vector<localization::tag_detection_t> detections) {
  if (mutex_.try_lock()) {

    for (size_t i = 0; i < detections.size(); i++) {
      double variance = std::pow(detections[i].distance, 1);
      variance *= 0.5;
      double tag_estimation[5] = {
          detections[i].translation.x, detections[i].translation.y,
          detections[i].rotation.z, variance,
          detections[i].timestamp +
              instance_.GetServerTimeOffset().value() / 1000000.0};

      pose_publisher_.Set(
          frc::Pose2d(units::meter_t{detections[i].translation.x},
                      units::meter_t{detections[i].translation.y},
                      units::radian_t{detections[i].rotation.z}));

      tag_estimation_publisher_.Set(tag_estimation);

      std::cout << "latency: "
                << frc::Timer::GetFPGATimestamp().value() -
                       detections[i].timestamp
                << "\n";
    }

    mutex_.unlock();
  }
}
}  // namespace localization
