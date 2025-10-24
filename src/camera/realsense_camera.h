#ifndef REALSENSE_CAMERA_H
#define REALSENSE_CAMERA_H
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
  rs2::frameset frames_;
  rs2::video_frame color_frame_;
  rs2::depth_frame depth_frame_;
};

}  // namespace camera

#endif  // REALSENSE_CAMERA_H
