#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
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
