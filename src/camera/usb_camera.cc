#include "usb_camera.h"
#include <opencv2/opencv.hpp>

namespace camera {

UsbCamera::UsbCamera(std::string pipeline) : pipeline_(pipeline) {
  cap_ = cv::VideoCapture(pipeline);
}

void UsbCamera::GetFrame(cv::Mat& frame) {
  cap_ >> frame;
}

}  // namespace camera
