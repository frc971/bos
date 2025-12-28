#include "src/camera/select_camera.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cv_camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/realsense_camera.h"

ABSL_FLAG(std::string, camera_choice, "usb0", "Camera options\n"
"Options:\n"
"  mipi0\n"
"  mipi1\n"
"  usb0\n"
"  usb1\n"
"  usb2\n"
"  usb3\n"
"  defaultusb0\n"
"  realsense");

namespace camera {

/*
    Asks Users for input and selects camera based on that.
    If any input is invalid, the function returns a call to itself.
*/

void PrintCameraConstant(Camera camera) {
  std::cout << "Selected camera" << std::endl;
  std::cout << "Pipline: " << camera_constants[camera].pipeline << std::endl;
}

Camera SelectCameraConfig() {
  const std::string& choice = absl::GetFlag(FLAGS_camera_choice);

  if (choice == "mipi0") {
    PrintCameraConstant(Camera::IMX296_0);
    return Camera::IMX296_0;
  } else if (choice == "mipi1") {
    PrintCameraConstant(Camera::IMX296_1);
    return Camera::IMX296_1;
  } else if (choice == "usb0") {
    PrintCameraConstant(Camera::USB0);
    return Camera::USB0;
  } else if (choice == "usb1") {
    PrintCameraConstant(Camera::USB1);
    return Camera::USB1;
  } else if (choice == "usb2") {
    PrintCameraConstant(Camera::USB2);
    return Camera::USB2;
  } else if (choice == "usb3") {
    PrintCameraConstant(Camera::USB3);
    return Camera::USB3;
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

std::unique_ptr<ICamera> GetCameraStream(Camera camera) {
  switch (camera) {
    case Camera::REALSENSE:
      return std::make_unique<camera::RealSenseCamera>();
    default:
      return std::make_unique<camera::CVCamera>(
          cv::VideoCapture(camera_constants[camera].pipeline));
  }
}
}  // namespace camera
