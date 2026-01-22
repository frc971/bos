#pragma once
#include "src/camera/camera_source.h"
#include "src/localization/position.h"
#include "src/utils/pch.h"

namespace localization {

constexpr double ktag_size = 0.1651;  // meters
const std::vector<cv::Point3f> kapriltag_dimensions = {
    {-ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, ktag_size / 2, 0},
    {ktag_size / 2, -ktag_size / 2, 0},
    {-ktag_size / 2, -ktag_size / 2, 0}};

class IAprilTagDetector {
 public:
  virtual auto GetTagDetections(camera::timestamped_frame_t& frame)
      -> std::vector<tag_detection_t> = 0;
  virtual ~IAprilTagDetector() = default;
};
}  // namespace localization
