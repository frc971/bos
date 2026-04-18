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
  std::array<cv::Point2d, 4> corners;  // Image coordinates of tag corners
  double timestamp;
  double confidence;

  auto operator==(const TagDetection& other) const -> bool {
    return tag_id == other.tag_id && corners == other.corners &&
           timestamp == other.timestamp && confidence == other.confidence;
  }

  friend auto operator<<(std::ostream& os, const TagDetection& t)
      -> std::ostream& {
    os << "ID: " << t.tag_id << "\nCorners:\n";
    for (const cv::Point2d& corner : t.corners) {
      os << corner << "\n";
    }
    os << "Timestamp: " << t.timestamp << "\nConfidence: " << t.confidence
       << std::endl;
    return os;
  }
};

using position_estimate_t = struct PositionEstimate {
  std::vector<int> tag_ids;
  std::vector<int> rejected_tag_ids;
  frc::Pose3d pose;
  double variance;
  double timestamp;
  int num_tags;
  double avg_tag_dist;
  double latency = -1;
  bool invalid = false;
  double loss = 0;
  friend auto operator<<(std::ostream& os, const PositionEstimate& t)
      -> std::ostream& {
    const auto& tr = t.pose.Translation();
    const auto& r = t.pose.Rotation();

    fmt::print(os,
               "Transform3d: "
               "translation (x={:.3f} m, y={:.3f} m, z={:.3f} m), "
               "rotation (roll={:.2f} deg, pitch={:.2f} deg, yaw={:.2f} deg), "
               "variance: {:.3f}\n",
               tr.X().value(), tr.Y().value(), tr.Z().value(),
               units::degree_t{r.X()}.value(), units::degree_t{r.Y()}.value(),
               units::degree_t{r.Z()}.value(), t.variance);
    return os;
  }
};
}  // namespace localization
