#pragma once
#include "src/utils/pch.h"
namespace camera {

using timestamped_frame_t = struct TimestampedFrame {
  cv::Mat frame;
  double timestamp;
};

class ICamera {
 public:
  virtual auto GetFrame() -> timestamped_frame_t = 0;
  virtual ~ICamera() = default;
};
}  // namespace camera
