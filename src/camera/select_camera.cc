#include "src/camera/select_camera.h"
#include <string>
#include <utility>
#include "cv_camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/disk_camera.h"
#include "src/utils/log.h"

using camera::camera_constants_t;

namespace camera {

auto SelectCameraConfig(const camera_constants_t& camera_constants)
    -> camera_constant_t {
  std::cout << "Available cameras:" << std::endl;
  for (const auto& entry : camera_constants) {
    std::cout << "  - " << entry.first << std::endl;
  }
  std::cout << "Please select a camera: " << std::flush;
  std::string choice;
  std::cin >> choice;

  if (choice.find('/') != std::string::npos) {
    return camera_constant_t{.name = choice};
  } else {
    return SelectCameraConfig(choice, camera_constants);
  }
}

auto SelectCameraConfig(const std::string& choice,
                        const camera_constants_t& camera_constants)
    -> camera_constant_t {
  return camera_constants.contains(choice)
             ? camera_constants.at(choice)
             : SelectCameraConfig(camera_constants);
}

auto SelectCameraConfig(std::optional<std::string> choice,
                        const camera_constants_t& camera_constants)
    -> camera_constant_t {
  return choice.has_value() ? SelectCameraConfig(choice.value(), camera_constants)
                            : SelectCameraConfig(camera_constants);
}

auto SelectCamera(const std::string& name, std::optional<std::string> choice,
                  const camera::camera_constants_t& camera_constants)
    -> CameraSource {
  return SelectCamera(name, choice, camera_constants, 1.0);
}

auto SelectCamera(const std::string& name, std::optional<std::string> choice,
                  const camera::camera_constants_t& camera_constants,
                  double disk_speed)
    -> CameraSource {
  auto config = SelectCameraConfig(std::move(choice), camera_constants);
  if (config.name.find('/') != std::string::npos) {
    return CameraSource{name, std::make_unique<DiskCamera>(config.name, disk_speed)};
  }
  return CameraSource{name, std::make_unique<CVCamera>(config)};
}

}  // namespace camera
