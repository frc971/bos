#include "src/camera/select_camera.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cv_camera.h"

namespace camera {

/*
    Asks Users for input and selects camera based on that.
    If any input is invalid, the function returns a call to itself.
*/
CVCamera SelectCamera() {
  std::cout << "Which input do you want to use; usb or mipi; use 1 or 2: ";
  int camType;  // Which camera type to use
  std::cin >> camType;
  if (camType == 1) {
    std::cout << "Which camera number; 0-3: ";
    int camNumUSB;  // Which USB camera to to use
    std::cin >> camNumUSB;
    return camera::CVCamera(
        cv::VideoCapture("/dev/video" + std::to_string(camNumUSB)));
  } else if (camType == 2) {
    std::cout << "Which camera number; 1-2: ";
    int camNumMIPI;  // Which MIPI camera to use
    std::cin >> camNumMIPI;

    
    return SelectCamera(); // temporary

  } else {
    std::cout << "INVALID INPUT: 1 or 2";
    return SelectCamera();
  }
}
}  // namespace camera
