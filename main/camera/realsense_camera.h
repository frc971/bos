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
  cv::VideoCapture cap_;
};

}  // namespace camera

#endif  // REALSENSE_CAMERA_H
