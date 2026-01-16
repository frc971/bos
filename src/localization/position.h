#pragma once
#include <fmt/ostream.h>
#include <frc/geometry/Transform3d.h>
#include <wpi/struct/Struct.h>
#include <wpilibc/frc/Timer.h>
#include <array>
#include <cmath>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <ostream>
#include "src/utils/log.h"
namespace localization {
using point3d_t = struct Point3d {
  double x;
  double y;
  double z;
};

using tag_detection_t = struct TagDetection {
  int tag_id;
  std::array<cv::Point2f, 4> corners;  // Image coordinates of tag corners
  double timestamp;
  double confidence;
};

using position_estimate_t = struct PositionEstimate {
  frc::Pose3d pose;
  double distance;
  double timestamp;
  friend auto operator<<(std::ostream& os, const PositionEstimate& t)
      -> std::ostream& {
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
};

using pose2d_t = struct Pose2d {
  double x;
  double y;
  double rotation;
};

using pose3d_t = struct Pose3d {
  point3d_t translation;
  point3d_t rotation;
};
}  // namespace localization
