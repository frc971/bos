#include "position_sender.h"
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <wpilibc/frc/Timer.h>
#include <string>
#include "frc/DataLogManager.h"
#include "src/localization/position.h"

namespace localization {

constexpr double RadianToDegree(double radian) {
  return radian * (180 / M_PI);
}

PositionSender::PositionSender(bool verbose)
    : instance_(nt::NetworkTableInstance::GetDefault()), verbose_(verbose) {
  std::shared_ptr<nt::NetworkTable> table =
      instance_.GetTable("orin/pose_estimate");

  nt::DoubleTopic translation_x_topic = table->GetDoubleTopic("translation_x");
  nt::DoubleTopic translation_y_topic = table->GetDoubleTopic("translation_y");

  nt::DoubleTopic rotation_topic = table->GetDoubleTopic("rotation");

  nt::DoubleTopic translation_x_varience_topic =
      table->GetDoubleTopic("translation_x_varience");
  nt::DoubleTopic translation_y_varience_topic =
      table->GetDoubleTopic("translation_y_varience");

  nt::DoubleTopic rotation_varience_topic =
      table->GetDoubleTopic("rotation_varience");

  nt::StructTopic<frc::Pose2d> pose_topic =
      table->GetStructTopic<frc::Pose2d>("pose");

  translation_x_publisher_ =
      translation_x_topic.Publish({.sendAll = true, .keepDuplicates = true});
  translation_y_publisher_ =
      translation_y_topic.Publish({.sendAll = true, .keepDuplicates = true});

  rotation_publisher_ =
      rotation_topic.Publish({.sendAll = true, .keepDuplicates = true});

  translation_x_varience_publisher_ = translation_x_varience_topic.Publish(
      {.sendAll = true, .keepDuplicates = true});
  translation_y_varience_publisher_ = translation_y_varience_topic.Publish(
      {.sendAll = true, .keepDuplicates = true});

  rotation_varience_publisher_ = rotation_varience_topic.Publish(
      {.sendAll = true, .keepDuplicates = true});

  nt::DoubleTopic timestamp_topic = table->GetDoubleTopic("timestamp");
  timestamp_publisher_ =
      timestamp_topic.Publish({.sendAll = true, .keepDuplicates = true});

  pose_publisher_ =
      pose_topic.Publish({.sendAll = true, .keepDuplicates = true});

  nt::DoubleArrayTopic tag_estimation_topic =
      table->GetDoubleArrayTopic("tag_estimation");
  tag_estimation_publisher_ =
      tag_estimation_topic.Publish({.sendAll = true, .keepDuplicates = true});
}

void PositionSender::Send(pose2d_t position_estimates, pose2d_t varience) {
  if (mutex_.try_lock()) {
    translation_x_publisher_.Set(position_estimates.x);
    translation_y_publisher_.Set(position_estimates.y);
    rotation_publisher_.Set(position_estimates.rotation);

    translation_x_varience_publisher_.Set(varience.x);
    translation_y_varience_publisher_.Set(varience.y);
    rotation_varience_publisher_.Set(varience.rotation);
    pose_publisher_.Set(
        frc::Pose2d(units::meter_t{position_estimates.x},
                    units::meter_t{position_estimates.y},
                    units::radian_t{position_estimates.rotation}));
    timestamp_publisher_.Set(frc::Timer::GetFPGATimestamp().value() +
                             instance_.GetServerTimeOffset().value() * 1000000);

    mutex_.unlock();
  }
  if (verbose_) {
    std::cout << "Position Sender: "
              << "\n";
    std::cout << "Translation: "
              << "\n";
    std::cout << position_estimates.x << "\n";
    std::cout << position_estimates.y << "\n";
    std::cout << RadianToDegree(position_estimates.rotation) << "\n";

    std::cout << "Varience: "
              << "\n";
    std::cout << varience.x << "\n";
    std::cout << varience.y << "\n";
    std::cout << varience.rotation << "\n";
  }
}

void PositionSender::Send(
    std::vector<localization::tag_detection_t> detections) {
  for (int i = 0; i < detections.size(); i++) {
    translation_x_publisher_.Set(detections[i].translation.x);
    translation_y_publisher_.Set(detections[i].translation.y);
    rotation_publisher_.Set(detections[i].rotation.z);

    double varience = 10 * detections[i].distance * detections[i].distance;

    translation_x_varience_publisher_.Set(varience);
    translation_y_varience_publisher_.Set(varience);
    rotation_varience_publisher_.Set(varience);
    pose_publisher_.Set(frc::Pose2d(units::meter_t{detections[i].translation.x},
                                    units::meter_t{detections[i].translation.y},
                                    units::radian_t{detections[i].rotation.z}));
    timestamp_publisher_.Set(detections[i].timestamp);

    double tag_estimation[5] = {
        detections[i].translation.x, detections[i].translation.y,
        detections[i].rotation.z, varience,
        detections[i].timestamp +
            instance_.GetServerTimeOffset().value() * 1000000};

    tag_estimation_publisher_.Set(tag_estimation);

    std::cout << "latency: "
              << frc::Timer::GetFPGATimestamp().value() -
                     detections[i].timestamp;
  }
}
}  // namespace localization
