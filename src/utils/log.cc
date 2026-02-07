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
void PrintTransformationMatrix(const cv::Mat& T) {

  CV_Assert(T.rows == 4 && T.cols == 4);
  CV_Assert(T.type() == CV_64F || T.type() == CV_32F);

  // Translation
  double x = T.at<double>(0, 3);
  double y = T.at<double>(1, 3);
  double z = T.at<double>(2, 3);

  // Rotation matrix
  cv::Mat R = T(cv::Rect(0, 0, 3, 3));

  // ZYX Euler angles (Yaw-Pitch-Roll)
  double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                        R.at<double>(1, 0) * R.at<double>(1, 0));

  bool singular = sy < 1e-6;

  double roll, pitch, yaw;

  if (!singular) {
    roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    pitch = std::atan2(-R.at<double>(2, 0), sy);
    yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
  } else {
    // Gimbal lock
    roll = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
    pitch = std::atan2(-R.at<double>(2, 0), sy);
    yaw = 0.0;
  }

  // Convert radians → degrees
  roll *= 180.0 / CV_PI;
  pitch *= 180.0 / CV_PI;
  yaw *= 180.0 / CV_PI;

  std::cout << "Transformation Matrix-> "
            << "X: " << x << " m, "
            << "Y: " << y << " m, "
            << "Z: " << z << " m, "
            << "Roll: " << roll << "°, "
            << "Pitch: " << pitch << "°, "
            << "Yaw: " << yaw << "°" << std::endl;
}

}  // namespace utils
