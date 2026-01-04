#include "cv_camera.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

namespace camera {

CVCamera::CVCamera(const cv::VideoCapture& cap) : cap_(cap) {}

void CVCamera::GetFrame(cv::Mat& frame) {
  cap_.read(frame);
  // TODO remove
  cv::rotate(frame, frame, cv::ROTATE_180);
}

}  // namespace camera
