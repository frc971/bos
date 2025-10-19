#include <iostream>
#include <memory>
#include "src/camera/camera.h"
#include "src/camera/cv_camera.h"
#include "src/camera/imx296_camera.h"

namespace calibration {

std::unique_ptr<camera::Camera> SelectCamera() {
  std::string camera_type;
  int id;

  std::cout << std::endl;
  std::cout << "Would you like imx296 or usb camera?" << std::endl;
  std::cin >> camera_type;

  std::cout << "What is the id of the camera?" << std::endl;
  std::cin >> id;

  if (camera_type == "imx296") {
  } else if (camera_type == "usb") {
  }
  std::cout << "Received invalid type. Retrying..." << std::endl;
  return SelectCamera();
}
}  // namespace calibration
