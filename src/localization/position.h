#pragma once
#include <fmt/ostream.h>
#include <frc/geometry/Transform3d.h>
#include <math.h>
#include <wpi/struct/Struct.h>
#include <wpilibc/frc/Timer.h>
#include <cmath>
#include <iostream>
#include <ostream>
#include "src/utils/log.h"
namespace localization {
typedef struct Point3d {
  double x;
  double y;
  double z;
} point3d_t;

typedef struct TagDetection {
  frc::Pose3d pose;
  double timestamp;
  double distance;
  int tag_id;
  friend std::ostream& operator<<(std::ostream& os, const TagDetection& t) {

    const auto& tr = t.pose.Translation();
    const auto& r = t.pose.Rotation();

    fmt::print(os,
               "Transform3d: "
               "translation (x={:.3f} m, y={:.3f} m, z={:.3f} m), "
               "rotation (roll={:.2f} deg, pitch={:.2f} deg, yaw={:.2f} deg)\n",
               tr.X().value(), tr.Y().value(), tr.Z().value(),
               units::degree_t{r.X()}.value(), units::degree_t{r.Y()}.value(),
               units::degree_t{r.Z()}.value());
    return os;
  }
} tag_detection_t;

typedef struct Pose2d {
  double x;
  double y;
  double rotation;
} pose2d_t;

typedef struct Pose3d {
  point3d_t translation;
  point3d_t rotation;
} pose3d_t;
}  // namespace localization
