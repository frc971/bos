#ifndef POSITION_SENDER_H
#define POSITION_SENDER_H

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "main/localization/position.h"
#include "tag_estimator.h"

namespace localization {
class PositionSender {
 public:
  PositionSender(bool verbose=false);
  void Send(pose2d_t position_estimate, pose2d_t varience);

 private:
  nt::NetworkTableInstance instance_;

  nt::DoublePublisher translation_x_publisher_;
  nt::DoublePublisher translation_y_publisher_;

  nt::DoublePublisher rotation_publisher_;

  nt::DoublePublisher translation_x_varience_publisher_;
  nt::DoublePublisher translation_y_varience_publisher_;

  nt::DoublePublisher rotation_varience_publisher_;

  std::mutex mutex_;
  bool verbose_;
};
}  // namespace localization
#endif  // POSITION_SENDER_H
