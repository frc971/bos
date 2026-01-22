#pragma once
#include <librealsense2/rs.hpp>
#include "src/camera/camera.h"
#include "src/utils/pch.h"
namespace camera {

class RealSenseCamera : public ICamera {
 public:
  RealSenseCamera();
  ~RealSenseCamera() override;
  auto GetFrame() -> timestamped_frame_t override;
  void GetFrame(cv::Mat& color_mat, cv::Mat& depth_mat);
  static void showDevices();

 private:
  rs2::pipeline pipe_;
  rs2::align align_to_color_;
  bool warmed_up_ = false;
};

}  // namespace camera
