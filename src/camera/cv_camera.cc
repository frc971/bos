#include "cv_camera.h"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

namespace camera {

CVCamera::CVCamera(std::unique_ptr<cv::VideoCapture> cap)
    : cap_(std::move(cap)) {}

void CVCamera::GetFrame(cv::Mat& frame) {
  cap_->read(frame);
}

}  // namespace camera
