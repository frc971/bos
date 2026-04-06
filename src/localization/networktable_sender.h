#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/BooleanArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <wpi/DataLogWriter.h>
#include "src/localization/position.h"
#include "src/localization/position_sender.h"
#include "src/utils/pch.h"

namespace localization {
// Sends position estimates from the orin to the rio. It is safe to call Send() on multiple threads because of the mutex
class NetworkTableSender : public IPositionSender {
 public:
  NetworkTableSender(const std::string& camera_name, bool verbose = false,
                     bool sim = false);
  void Send(const std::vector<localization::position_estimate_t>& detections)
      override;

 private:
  nt::NetworkTableInstance instance_;

  nt::StructPublisher<frc::Pose2d> pose_publisher_;
  nt::StructPublisher<frc::Pose3d> pose3d_publisher_;
  nt::StructArrayPublisher<frc::Pose3d> all_estimates_publisher_;
  nt::DoublePublisher latency_publisher_;
  nt::DoublePublisher timestamp_publisher_;
  nt::IntegerPublisher num_tags_publisher_;
  nt::DoublePublisher varience_publisher_;
  nt::DoublePublisher loss_publisher_;
  nt::DoubleArrayPublisher tag_estimation_publisher_;
  nt::BooleanArrayPublisher tag_ids_publisher_;
  nt::BooleanArrayPublisher rejected_tag_ids_publisher_;
  std::optional<wpi::log::DataLogWriter> log_;
  std::optional<wpi::log::StructLogEntry<frc::Pose3d>> pose3d_log_;
  std::optional<wpi::log::StructArrayLogEntry<frc::Pose3d>> all_estimates_log_;
  std::optional<wpi::log::DoubleLogEntry> latency_log_;
  std::optional<wpi::log::DoubleLogEntry> timestamp_log_;
  std::optional<wpi::log::IntegerLogEntry> num_tags_log_;
  std::optional<wpi::log::DoubleLogEntry> varience_log_;
  std::optional<wpi::log::DoubleLogEntry> loss_log_;
  std::optional<wpi::log::DoubleArrayLogEntry> tag_estimation_log_;
  std::optional<wpi::log::BooleanArrayLogEntry> tag_ids_log_;
  std::optional<wpi::log::BooleanArrayLogEntry> rejected_tag_ids_log_;

  std::mutex mutex_;
  bool verbose_;
};
}  // namespace localization
