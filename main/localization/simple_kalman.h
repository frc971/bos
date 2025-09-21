#include <frc/EigenCore.h>
#include <frc/geometry/Pose2d.h>
#include "third_party/kalman-cpp/kalman.h"
namespace Localization {
struct SimpleKalmanConfig {
  double position;
  double velocity;
  double time;
  double measurment_noise;
  double process_noise;
};
class SimpleKalman {
 public:
  SimpleKalman(double position, double velocity, double time,
               double measurment_noise, double process_noise);
  SimpleKalman(SimpleKalmanConfig config);
  void Update(double position_update, double time);

  double position() { return kalman_filter_.state()[0]; }
  double velocity() { return kalman_filter_.state()[1]; }
  double time() { return kalman_filter_.time(); }

  double position_varience() { return kalman_filter_.P(0, 0); }
  double velocity_varience() { return kalman_filter_.P(1, 1); }

  // double position_varience() {}

 private:
  KalmanFilter kalman_filter_;
  double time_;  // Current time of the kalman filter (seconds)
};
}  // namespace Localization
