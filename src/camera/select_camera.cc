#include "src/camera/select_camera.h"
#include <filesystem>
#include <string>
#include "cv_camera.h"
#include "src/camera/camera_constants.h"
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
  return SelectCameraConfig(choice, camera_constants);
}

auto SelectCameraConfig(const std::string& choice,
                        const camera_constants_t& camera_constants)
    -> camera_constant_t {
  if (choice.find('/') != std::string::npos) {
    std::string camera_name = std::filesystem::path(choice).filename().string();

    auto it = camera_constants.find(camera_name);
    if (it == camera_constants.end()) {
      it = camera_constants.find("main_bot_" + camera_name);
    }

    if (it != camera_constants.end()) {
      auto config = it->second;
      config.name = choice;
      return config;
    }
    return camera_constant_t{.name = choice};
  }

  if (camera_constants.contains(choice)) {
    return camera_constants.at(choice);
  }

  return SelectCameraConfig(camera_constants);
}

auto SelectCameraConfig(std::optional<std::string> choice,
                        const camera_constants_t& camera_constants)
    -> camera_constant_t {
  return choice.has_value()
             ? SelectCameraConfig(choice.value(), camera_constants)
             : SelectCameraConfig(camera_constants);
}

auto GetCameraStream(camera_constant_t camera) -> std::unique_ptr<ICamera> {
  return std::make_unique<camera::CVCamera>(camera);
}

}  // namespace camera
