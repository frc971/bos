#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <string>
namespace camera {

class RealSenseCamera {
 public:
  RealSenseCamera();
  ~RealSenseCamera();
  void getFrame(cv::Mat& mat);

 private:
};

}  // namespace camera
