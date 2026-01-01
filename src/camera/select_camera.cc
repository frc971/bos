#include "src/camera/select_camera.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cv_camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/realsense_camera.h"
#include <unordered_map>
#include <memory>
#include <string>

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

const std::unordered_map<std::string, Camera> name_to_camera{
    {"mipi0", Camera::IMX296_0}, 
    {"mipi1", Camera::IMX296_1},
    {"usb0", Camera::USB0}, 
    {"usb1", Camera::USB1},
    {"usb2", Camera::USB2}, 
    {"usb3", Camera::USB3},
    {"defaultusb0", Camera::DEFAULT_USB0}, 
    {"realsense", Camera::REALSENSE}
};

/*
    Asks Users for input and selects camera based on that.
    If any input is invalid, the function returns a call to itself.
*/

void PrintCameraConstant(Camera camera) {
  std::cout << "Selected camera" << std::endl;
  std::cout << "Pipline: " << camera_constants[camera].pipeline << std::endl;
}

// can be called with  const std::string& choice = absl::GetFlag(FLAGS_camera_choice);
// to access the absl flag value 
Camera SelectCameraConfig(const std::string &name) {
    if (!name_to_camera.count(name)) {
        std::cout << "Warning: no camera found for name " + name << std::endl;
        std::cout << "Retrying..." << std::endl;
        return SelectCameraConfig();
    }

    Camera cam = name_to_camera[name];
    PrintCameraConstant(cam);
    return cam;

}

Camera SelectCameraConfig() {
  std::cout << "Please type in what camera you want." << std::endl;
  std::cout << "Options: " << std::endl;
  std::cout << "mipi0" << std::endl;
  std::cout << "mipi1" << std::endl;
  std::cout << "usb0" << std::endl;
  std::cout << "usb1" << std::endl;
  std::cout << "usb2" << std::endl;
  std::cout << "usb3" << std::endl;
  std::cout << "defaultusb0" << std::endl;
  std::cout << "realsense" << std::endl;

  std::string choice;
  std::cin >> choice;

  return SelectCameraConfig(choice);
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
