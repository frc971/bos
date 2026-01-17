#pragma once
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <array>
#include "src/localization/apriltag_detector.h"

namespace localization {

class AbsoluteAprilTagLayout {

 public:
  AbsoluteAprilTagLayout(const frc::AprilTagFieldLayout& layout,
                         float tag_size);
  // The points are in wpilib coordinates
  // 0 = x
  // 1 = y
  // 2 = z
  auto GetTagPoints(int tag_id) -> std::optional<std::vector<cv::Point3f>>;

 private:
  std::map<int, std::vector<cv::Point3f>> absolute_apriltag_layout_;
};
}  // namespace localization
