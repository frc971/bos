#include <frc/apriltag/AprilTagFieldLayout.h>
#include <array>
#include "src/localization/apriltag_detector.h"

namespace localization {

class AbsoluteAprilTagLayout {

 public:
  AbsoluteAprilTagLayout(const frc::AprilTagFieldLayout& layout,
                         float tag_size);
  auto GetTagPoints(int tag_id) -> std::vector<cv::Point3f>*;

 private:
  std::map<int, std::vector<cv::Point3f>> absolute_apriltag_layout_;
};
}  // namespace localization
