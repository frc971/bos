#include "src/camera/select_camera.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cv_camera.h"
#include "src/camera/camera_constants.h"

namespace camera {

/*
    Asks Users for input and selects camera based on that.
    If any input is invalid, the function returns a call to itself.
*/

void PrintCameraConstant(Camera camera) {
  std::cout << "Selected camera" << std::endl;
  std::cout << "Pipline: " << camera_constants[camera].pipeline << std::endl;
}
Camera SelectCamera() {
  std::cout << "Please type in what cameras you want" << std::endl;
  std::cout << "Options: " << std::endl;
  std::cout << "mipi0" << std::endl;
  std::cout << "mipi1" << std::endl;
  std::cout << "usb0" << std::endl;
  std::cout << "defaultusb0" << std::endl;

  std::string choice;
  std::cin >> choice;

  if (choice == "mipi0") {
    PrintCameraConstant(Camera::IMX296_0);
    return Camera::IMX296_0;
  }

  if (choice == "mipi1") {
    PrintCameraConstant(Camera::IMX296_1);
    return Camera::IMX296_1;
  }

  if (choice == "usb0") {
    PrintCameraConstant(Camera::USB0);
    return Camera::USB0;
  }

  if (choice == "usb1") {
    PrintCameraConstant(Camera::USB1);
    return Camera::USB1;
  }

  if (choice == "defaultusb0") {
    PrintCameraConstant(Camera::DEFAULT_USB0);
    return Camera::DEFAULT_USB0;
  }

  std::cout << "You did not give a valid input. Retrying..." << std::endl;

  return SelectCamera();
}
}  // namespace camera
