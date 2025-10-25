#include "src/camera/select_camera.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cv_camera.h"

namespace camera {

/*
    Asks Users for input and selects camera based on that.
    If any input is invalid, the function returns a call to itself.
*/
Camera SelectCamera() {
  std::cout << "Please type in what cameras you want" << std::endl;
  std::cout << "Options: " << std::endl;
  std::cout << "mipi0" << std::endl;
  std::cout << "mipi1" << std::endl;
  std::cout << "usb0" << std::endl;
  std::cout << "usb1" << std::endl;

  std::string choice;
  std::cin >> choice;

  if (choice == "mipi0")
    return Camera::IMX296_0;

  if (choice == "mipi1")
    return Camera::IMX296_1;

  if (choice == "usb0")
    return Camera::USB0;

  if (choice == "usb1")
    return Camera::USB1;

  std::cout << "You did not give a valid input. Retrying..." << std::endl;

  return SelectCamera();
}
}  // namespace camera
