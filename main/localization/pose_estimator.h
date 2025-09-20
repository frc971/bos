#pragma once  // Headers garuds are cooked right now, we need to do this intead https://google.github.io/styleguide/cppguide.html#The__define_Guard
#include <frc/EigenCore.h>
#include <frc/system/LinearSystem.h>
#include "frc/estimator/DifferentialDrivePoseEstimator.h"
#include "frc/estimator/KalmanFilter.h"

// Uses wpilib's DifferentialDrivePoseEstimator with a mutex to agregate mutliple camera's multiple detections
namespace Localization {

class PoseEstimator {
 public:
  PoseEstimator(double x, double y);

 private:
  frc::KalmanFilter<6, 3, 3> estimator_;
  frc::LinearSystem<6, 3, 3> plant();
  frc::Matrixd<6, 6> transition_matrx;
  double timestamp;
};
}  // namespace Localization
