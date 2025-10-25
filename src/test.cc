#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include "src/camera/camera_constants.h"
#include "src/localization/position.h"
#include "src/localization/tag_estimator.h"

int main() {
  std::cout << camera::camera_constants[0].pipeline << std::endl;
}
