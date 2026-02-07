#pragma once
#include <fmt/ostream.h>
#include <frc/geometry/Transform3d.h>
#include "src/utils/log.h"
#include "src/utils/pch.h"
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
  friend auto operator==(const PositionEstimate& left,
                         const PositionEstimate& right) -> bool {
    const auto& lt = left.pose.Translation();
    const auto& lr = left.pose.Rotation();
    const auto& rt = left.pose.Translation();
    const auto& rr = left.pose.Rotation();

    return lt.X().value() == rt.X().value() &&
           lt.Y().value() == rt.Y().value() &&
           lt.Z().value() == rt.Z().value() &&
           lr.X().value() == rr.X().value() &&
           lr.Y().value() == rr.Y().value() && lr.Z().value() == rr.Z().value();
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
