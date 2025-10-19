#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include "src/localization/position.h"
#include "src/localization/tag_estimator.h"

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

inline void PrintTransform3d(const frc::Transform3d& T) {
  const auto& tr = T.Translation();
  const auto& r = T.Rotation();

  fmt::print(
      "Transform3d: "
      "translation (x={:.3f} m, y={:.3f} m, z={:.3f} m), "
      "rotation (roll={:.2f} deg, pitch={:.2f} deg, yaw={:.2f} deg)\n",
      tr.X().value(), tr.Y().value(), tr.Z().value(),
      units::degree_t{r.X()}.value(), units::degree_t{r.Y()}.value(),
      units::degree_t{r.Z()}.value());
}

int main() {
  frc::AprilTagFieldLayout layout(frc::AprilTagFieldLayout::LoadField(
      frc::AprilTagField::k2025ReefscapeAndyMark));

  std::cout << "camera to tag: \n";
  frc::Transform3d camera_to_tag{1.0_m, 0_m, 0_m,
                                 frc::Rotation3d{0_deg, 0_deg, 180_deg}};
  PrintTransform3d(camera_to_tag);

  std::cout << "tag_to_camera: \n";
  PrintTransform3d(camera_to_tag.Inverse());
  frc::Transform3d tag_to_camera(camera_to_tag.Inverse());
  PrintTransform3d(tag_to_camera);

  std::cout << "feild to tag: ";
  frc::Pose3d feild_to_tag = layout.GetTagPose(10).value();
  PrintPose3d(feild_to_tag);

  std::cout << "feild to camera: \n";
  frc::Pose3d feild_to_cacmera = feild_to_tag.TransformBy(tag_to_camera);
  PrintPose3d(feild_to_cacmera);

  std::cout << "feild to robot: \n";
  frc::Transform3d robot_to_camera{0_m, 0_m, 0_m,
                                   frc::Rotation3d{0_deg, 0_deg, 0_deg}};
  frc::Transform3d camera_to_robot(robot_to_camera.Inverse());
  frc::Pose3d feild_to_robot = feild_to_cacmera.TransformBy(camera_to_robot);
  PrintPose3d(feild_to_robot);
}
