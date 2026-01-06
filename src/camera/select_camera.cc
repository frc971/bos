#include "src/camera/select_camera.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cv_camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/realsense_camera.h"

namespace camera {

/*
    Asks Users for input and selects camera based on that.
    If any input is invalid, the function returns a call to itself.
*/

void PrintCameraConstant(Camera camera) {
  std::cout << "Selected camera" << std::endl;
  std::cout << "Pipline: " << camera_constants[camera].pipeline << std::endl;
}

auto SelectCameraConfig() -> Camera {
  std::cout << "Please type in what camera you want." << std::endl;
  std::cout << "Options: " << std::endl;
  std::cout << "mipi0" << std::endl;
  std::cout << "mipi1" << std::endl;
  std::cout << "usb0" << std::endl;
  std::cout << "usb1" << std::endl;
  std::cout << "defaultusb0" << std::endl;
  std::cout << "realsense" << std::endl;

  std::string choice;
  std::cin >> choice;

  if (choice == "mipi0") {
    PrintCameraConstant(Camera::IMX296_0);
    return Camera::IMX296_0;
  } else if (choice == "mipi1") {
    PrintCameraConstant(Camera::IMX296_1);
    return Camera::IMX296_1;
  } else if (choice == "usb0") {
    PrintCameraConstant(Camera::FIDDLER_USB0);
    return Camera::FIDDLER_USB0;
  } else if (choice == "usb1") {
    PrintCameraConstant(Camera::FIDDLER_USB1);
    return Camera::FIDDLER_USB1;
  } else if (choice == "defaultusb0") {
    PrintCameraConstant(Camera::DEFAULT_USB0);
    return Camera::DEFAULT_USB0;
  } else if (choice == "realsense") {
    PrintCameraConstant(Camera::REALSENSE);
    return Camera::REALSENSE;
  } else {
    std::cout << "You did not give a valid input. Retrying..." << std::endl;
    return SelectCameraConfig();
  }
}

auto GetCameraStream(Camera camera) -> std::unique_ptr<ICamera> {
  switch (camera) {
    case Camera::REALSENSE:
      return std::make_unique<camera::RealSenseCamera>();
    default:
      return std::make_unique<camera::CVCamera>(
          cv::VideoCapture(camera_constants[camera].pipeline));
  }
}
}  // namespace camera
