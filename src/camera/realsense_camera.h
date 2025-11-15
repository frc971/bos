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
  void getFrame(cv::Mat& color_mat, cv::Mat& depth_mat);
  static void showDevices();

 private:
  rs2::pipeline pipe_;
  rs2::align align_to_color;
};

}  // namespace camera
