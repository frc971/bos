#pragma once
#include <frc/geometry/Transform3d.h>
#include <opencv2/core/mat.hpp>
#include "src/camera/camera_source.h"
#include "src/localization/position.h"

namespace localization {

constexpr double ktag_size = 0.1651;  // meters
const std::vector<cv::Point3f> kapriltag_dimensions = {
    {-ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, -ktag_size / 2, 0},
    {-ktag_size / 2, -ktag_size / 2, 0}};

class IAprilTagDetector {
 public:
  virtual std::vector<tag_detection_t> GetTagDetections(
      camera::timestamped_frame_t& frame) = 0;
  virtual ~IAprilTagDetector() = default;
};
}  // namespace localization
