#pragma once  // Headers garuds are cooked right now, we need to do this intead https://google.github.io/styleguide/cppguide.html#The__define_Guard
#include <frc/EigenCore.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/system/LinearSystem.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include "frc/estimator/DifferentialDrivePoseEstimator.h"
#include "frc/estimator/KalmanFilter.h"
#include "position.h"
#include "simple_kalman.h"
#include "units/angle.h"
// Uses wpilib's DifferentialDrivePoseEstimator with a mutex to agregate mutliple camera's multiple detections
namespace Localization {

class PoseEstimator {
 public:
  PoseEstimator(SimpleKalmanConfig x_filter_config,
                SimpleKalmanConfig y_filter_config,
                SimpleKalmanConfig rotation_filter_config);
  void Update(double x, double y, double rotation, double time);
  void Update(std::vector<tag_detection_t> position);
  pose2d_t GetPose();

 private:
  SimpleKalman x_filter_;
  SimpleKalman y_filter_;
  SimpleKalman rotation_filter_;
  double timestamp;
};
}  // namespace Localization
