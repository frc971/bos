#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
namespace camera {
class ICamera {
 public:
  virtual void GetFrame(cv::Mat& mat) = 0;
};
}  // namespace camera
