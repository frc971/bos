#include <frc/apriltag/AprilTagFieldLayout.h>
#include <iostream>
int main() {
  frc::AprilTagFieldLayout layout(frc::AprilTagFieldLayout::LoadField(
      frc::AprilTagField::k2025ReefscapeAndyMark));

  std::cout << layout.GetTagPose(7)->Rotation().Z().value() << "a\n";
}
