#ifndef POSITION_SENDER_H
#define POSITION_SENDER_H

#include "pose_estimator.h"
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

class PositionSender {
public:
  PositionSender();
  void Send(PoseEstimator::position_estimate_t position_estimate);

private:
  nt::DoublePublisher translation_x_publisher_;
  nt::DoublePublisher translation_y_publisher_;
  nt::DoublePublisher translation_z_publisher_;

  nt::DoublePublisher rotation_x_publisher_;
  nt::DoublePublisher rotation_y_publisher_;
  nt::DoublePublisher rotation_z_publisher_;
};

#endif // POSITION_SENDER_H
