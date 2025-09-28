#include "realsense_camera.h"
#include <iostream>
namespace camera {

RealSenseCamera::RealSenseCamera() {
  cap_.open(0, cv::CAP_ANY);
  if (!cap_.isOpened()) {
    std::cerr << "ERROR: Could not open RealSense camera\n";
  }
}

RealSenseCamera::~RealSenseCamera() {
  if (cap_.isOpened()) {
    cap_.release();
  }
}

void RealSenseCamera::getFrame(cv::Mat& mat) {
  if (cap_.isOpened()) {
    cap_ >> mat;
  }
}

}  // namespace camera
