#ifndef REALSENSE_CAMERA_H
#define REALSENSE_CAMERA_H
#include <librealsense2/hpp/rs_frame.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include "camera.h"
#include <librealsense2/rs.hpp>
namespace Camera {

class RealSenseCamera final : public Camera {
 public:
  RealSenseCamera(int id);
  ~RealSenseCamera();
  void getFrame(cv::Mat& mat);
private:
  rs2::pipeline pipe_;
  rs2::frameset frames_;
  rs2::frame color_image_;
  rs2::depth_frame depth_image_;
};

} // namespace Camera

#endif  // REALSENSE_CAMERA_H
