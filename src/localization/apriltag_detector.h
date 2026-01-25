#pragma once
#include "src/camera/camera_source.h"
#include "src/localization/position.h"
#include "src/utils/pch.h"

namespace localization {

class IAprilTagDetector {
 public:
  virtual auto GetTagDetections(camera::timestamped_frame_t& frame)
      -> std::vector<tag_detection_t> = 0;
  virtual ~IAprilTagDetector() = default;
};
}  // namespace localization
