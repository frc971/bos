#include "src/utils/log.h"
#include <iomanip>
#include <iostream>

namespace utils {

void PrintPose3d(const frc::Pose3d& pose) {
  // Extract translation (in meters)
  double x = pose.X().value();
  double y = pose.Y().value();
  double z = pose.Z().value();

  // Extract rotation (in degrees)
  double roll = pose.Rotation().X().value();  // radians → will convert below
  double pitch = pose.Rotation().Y().value();
  double yaw = pose.Rotation().Z().value();

  // Convert radians to degrees
  roll = roll * 180.0 / M_PI;
  pitch = pitch * 180.0 / M_PI;
  yaw = yaw * 180.0 / M_PI;

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "Pose3d -> X: " << x << " m, Y: " << y << " m, Z: " << z << " m"
            << ", Roll: " << roll << "°, Pitch: " << pitch << "°, Yaw: " << yaw
            << "°" << std::endl;
}

void PrintTransform3d(const frc::Transform3d& transform) {
  const auto& tr = transform.Translation();
  const auto& r = transform.Rotation();

  fmt::print(
      "Transform3d: "
      "translation (x={:.3f} m, y={:.3f} m, z={:.3f} m), "
      "rotation (roll={:.2f} deg, pitch={:.2f} deg, yaw={:.2f} deg)\n",
      tr.X().value(), tr.Y().value(), tr.Z().value(),
      units::degree_t{r.X()}.value(), units::degree_t{r.Y()}.value(),
      units::degree_t{r.Z()}.value());
}

}  // namespace utils
