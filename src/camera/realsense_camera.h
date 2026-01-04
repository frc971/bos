#pragma once
#include <librealsense2/rs.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include "src/camera/camera.h"
namespace camera {

class RealSenseCamera : public ICamera {
 public:
  RealSenseCamera();
  ~RealSenseCamera() override;
  void GetFrame(cv::Mat& mat) override;
  void GetFrame(cv::Mat& color_mat, cv::Mat& depth_mat);
  static void showDevices();

 private:
  rs2::pipeline pipe_;
  rs2::align align_to_color_;
  bool warmed_up_ = false;
};

}  // namespace camera
