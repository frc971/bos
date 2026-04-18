#include "src/camera/select_camera.h"
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include "absl/flags/flag.h"
#include "absl/flags/internal/flag.h"
#include "absl/status/status.h"
#include "cv_camera.h"
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/disk_camera.h"
#include "src/camera/uvc_camera.h"
#include "src/utils/log.h"

ABSL_FLAG(std::optional<std::string>, folder_path, std::nullopt,  // NOLINT
          "Folder path to folder with the images logs");

using camera::camera_constants_t;

namespace camera {

auto SelectCameraConfig(const camera_constants_t& camera_constants)
    -> std::unique_ptr<ICamera> {
  LOG(INFO) << "Available cameras: ";
  for (const auto& entry : camera_constants) {
    LOG(INFO) << "  - " << entry.first;
  }
  LOG(INFO) << "Please select a camera: ";
  std::string choice;
  std::cin >> choice;
  return SelectCameraConfig(choice, camera_constants);
}

auto SelectCameraConfig(const std::string& choice,
                        const camera_constants_t& camera_constants)
    -> std::unique_ptr<ICamera> {

  if (absl::GetFlag(FLAGS_folder_path).has_value()) {
    if (std::filesystem::is_directory(
            absl::GetFlag(FLAGS_folder_path).value())) {
      return std::make_unique<camera::DiskCamera>(
          absl::GetFlag(FLAGS_folder_path).value(),
          camera::GetCameraConstants()[choice]);
    } else {
      LOG(WARNING) << "You entered in an invalid camera";
    }
  }
  if (camera_constants.contains(choice)) {
    if (camera_constants.at(choice).serial_id.has_value()) {
      LOG(INFO) << "Initializing via uvc";
      absl::Status status;
      auto camera =
          std::make_unique<UVCCamera>(camera_constants.at(choice), status);
      if (!status.ok()) {
        LOG(WARNING) << "Failed to select camera via uvc: " << status.message();
      }
      return camera;
    }
    return std::make_unique<camera::CVCamera>(
        camera::GetCameraConstants()[choice]);
  } else {
    return SelectCameraConfig(camera_constants);
  }
  // return camera_constants.contains(choice)
  //            ? std::make_unique<camera::CVCamera>(
  //                  camera::GetCameraConstants()[choice])
  //            : SelectCameraConfig(camera_constants);
}

auto SelectCameraConfig(std::optional<std::string> choice,
                        const camera_constants_t& camera_constants)
    -> std::unique_ptr<ICamera> {

  if (choice.has_value()) {
    return SelectCameraConfig(choice.value(), camera_constants);
  } else {
    return SelectCameraConfig(camera_constants);
  }
  // return choice.has_value()
  //            ? SelectCameraConfig(choice.value(), camera_constants)
  //            : SelectCameraConfig(camera_constants);
}

}  // namespace camera
