#pragma once
#include <librealsense2/rs.hpp>
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
  rs2::pipeline pipe_;
};

}  // namespace camera
