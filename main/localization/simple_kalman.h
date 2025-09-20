#include <frc/EigenCore.h>
#include <frc/geometry/Pose2d.h>
#include "third_party/kalman-cpp/kalman.hpp"
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
               int measurment_noise, int process_noise);
  SimpleKalman(SimpleKalmanConfig config);
  void Update(double position_update);

  double position() { return kalman_filter_.state()[0]; }
  double velocity() { return kalman_filter_.state()[1]; }
  double time() { return kalman_filter_.time(); }

 private:
  KalmanFilter kalman_filter_;
};
}  // namespace Localization
