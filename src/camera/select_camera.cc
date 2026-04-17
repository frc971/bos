#include "src/camera/select_camera.h"
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include "absl/flags/flag.h"
#include "absl/flags/internal/flag.h"
#include "cv_camera.h"
#include "src/camera/camera.h"
#include "src/camera/camera_constants.h"
#include "src/camera/disk_camera.h"
#include "src/utils/log.h"

ABSL_FLAG(std::optional<std::string>, folder_path, std::nullopt,  // NOLINT
          "Folder path to folder with the images logs");

using camera::camera_constants_t;

namespace camera {

namespace {

auto ResolveCameraChoice(const std::string& choice,
                         const camera_constants_t& camera_constants)
    -> std::optional<std::string> {
  if (camera_constants.contains(choice)) {
    return choice;
  }

  constexpr std::string_view kPrefixes[] = {"main_bot_", "second_bot_",
                                            "turret_bot_"};
  for (std::string_view prefix : kPrefixes) {
    if (choice.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string stripped = choice.substr(prefix.size());
    if (camera_constants.contains(stripped)) {
      return stripped;
    }
  }

  return std::nullopt;
}

}  // namespace

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
  const std::optional<std::string> resolved_choice =
      ResolveCameraChoice(choice, camera_constants);

  if (absl::GetFlag(FLAGS_folder_path).has_value()) {
    if (std::filesystem::is_directory(
            absl::GetFlag(FLAGS_folder_path).value())) {
      if (!resolved_choice.has_value()) {
        LOG(WARNING) << "You entered in an invalid camera";
      } else {
        return std::make_unique<camera::DiskCamera>(
            absl::GetFlag(FLAGS_folder_path).value(),
            camera_constants.at(resolved_choice.value()));
      }
    } else {
      LOG(WARNING) << "You entered in an invalid camera";
    }
  }
  if (resolved_choice.has_value()) {

    return std::make_unique<camera::CVCamera>(
        camera_constants.at(resolved_choice.value()));
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
