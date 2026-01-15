#include "src/camera/select_camera.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "cv_camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/realsense_camera.h"
#include "src/utils/log.h"

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
  std::cout << "Please select a camera" << std::endl;
  for (const auto& camera_constant : camera_constants) {
    std::cout << camera_constant.name << std::endl;
  }
  std::string choice;
  std::cin >> choice;
  return SelectCameraConfig(choice);
}

auto SelectCameraConfig(const std::string& choice) -> Camera {
  for (int i = 0; i < Camera::CAMERA_LENGTH; i++) {
    if (choice == camera_constants[i].name) {
      return static_cast<Camera>(i);
    }
  }
  std::cout << "Did not find camera that match fallback to manual select";
  return SelectCameraConfig();
}

auto SelectCameraConfig(std::optional<std::string> choice) -> Camera {
  return choice.has_value() ? SelectCameraConfig(choice.value())
                            : SelectCameraConfig();
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
