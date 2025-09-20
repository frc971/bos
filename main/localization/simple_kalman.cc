#include "simple_kalman.h"

namespace Localization {

SimpleKalman::SimpleKalman(double position, double velocity, double time,
                           int measurment_noise, int process_noise) {
  constexpr double placeholder_dt = 1;
  Eigen::MatrixXd A(2, 2);
  Eigen::MatrixXd C(1, 2);
  Eigen::MatrixXd Q(2, 2);
  Eigen::MatrixXd R(1, 1);
  Eigen::MatrixXd P(2, 2);

  A << 1, placeholder_dt, 0,
      1;  // This is temp, we set A each time we update position
  C << 1, 0;
  Q << process_noise;
  R << measurment_noise;
  P << 1, 0, 0, 1;
  kalman_filter_ = KalmanFilter(placeholder_dt, A, C, Q, R, P);
  Eigen::VectorXd initial_pose(2);
  initial_pose << position, velocity;
  kalman_filter_.init(time, initial_pose);
}

SimpleKalman::SimpleKalman(SimpleKalmanConfig config) {
  SimpleKalman(config.position, config.velocity, config.time,
               config.measurment_noise, config.process_noise);
}

void SimpleKalman::Update(double position_update) {
  Eigen::VectorXd position_update_(1);
  position_update_ << position_update;
  kalman_filter_.update(position_update_);
}

}  // namespace Localization
