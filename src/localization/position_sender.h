#pragma once

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include "src/localization/position.h"
#include "tag_estimator.h"

namespace localization {
class PositionSender {
 public:
  PositionSender(bool verbose = false);
  void Send(pose2d_t position_estimate, pose2d_t varience);
  void Send(std::vector<localization::tag_detection_t> detections);

 private:
  nt::NetworkTableInstance instance_;

  // nt::DoublePublisher translation_x_publisher_;
  // nt::DoublePublisher translation_y_publisher_;
  //
  // nt::DoublePublisher rotation_publisher_;
  //
  // nt::DoublePublisher translation_x_varience_publisher_;
  // nt::DoublePublisher translation_y_varience_publisher_;
  //
  // nt::DoublePublisher rotation_varience_publisher_;
  //
  // nt::DoublePublisher timestamp_publisher_;

  nt::StructPublisher<frc::Pose2d> pose_publisher_;

  nt::DoubleArrayPublisher tag_estimation_publisher_;

  std::mutex mutex_;
  bool verbose_;
};
}  // namespace localization
