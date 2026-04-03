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
#include "src/localization/position.h"
#include "src/utils/pch.h"

namespace localization {
// Sends position estimates from the orin to the rio. It is safe to call Send() on multiple threads because of the mutex
class PositionSender {
 public:
  PositionSender(const std::string& camera_name, bool verbose = false);
  void Send(const std::vector<localization::position_estimate_t>& detections,
            double latency,
            const std::optional<std::vector<frc::Pose3d>>& all_estimates =
                std::nullopt);

 private:
  nt::NetworkTableInstance instance_;

  nt::StructPublisher<frc::Pose2d> pose_publisher_;
  nt::StructPublisher<frc::Pose3d> pose3d_publisher_;
  nt::StructArrayPublisher<frc::Pose3d> all_estimates_publisher_;
  nt::DoublePublisher latency_publisher_;
  nt::DoublePublisher timestamp_publisher_;
  nt::IntegerPublisher num_tags_publisher_;
  nt::DoublePublisher varience_publisher_;
  nt::DoubleArrayPublisher tag_estimation_publisher_;
  nt::BooleanArrayPublisher tag_ids_publisher_;
  nt::BooleanArrayPublisher rejected_tag_ids_publisher_;

  std::mutex mutex_;
  bool verbose_;
};
}  // namespace localization
