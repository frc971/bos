#pragma once
#include "src/camera/camera_constants.h"
#include "src/utils/pch.h"
namespace camera {

using timestamped_frame_t = struct TimestampedFrame {
  cv::Mat frame;
  double timestamp;
  bool invalid = false;

  friend auto operator<<(std::ostream& os, const TimestampedFrame& f)
      -> std::ostream& {
    os << "TimestampedFrame{"
       << "timestamp=" << f.timestamp << ", invalid=" << std::boolalpha
       << f.invalid;

    if (!f.frame.empty()) {
      os << ", frame=(" << f.frame.cols << "x" << f.frame.rows
         << ", channels=" << f.frame.channels() << ")";
    } else {
      os << ", frame=<empty>";
    }

    os << "}";
    return os;
  }
};

class ICamera {
 public:
  virtual void GetFrame(timestamped_frame_t* output) = 0;
  virtual void Restart() = 0;
  [[nodiscard]] virtual auto GetCameraConstant() const -> camera_constant_t = 0;
  virtual auto IsDone() -> bool { return false; }
  virtual ~ICamera() = default;
};
}  // namespace camera
