#include <frc/DataLogManager.h>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <memory>
#include <optional>
#include <thread>
#include <vector>
#include "absl/flags/flag.h"
#include "absl/flags/internal/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/log.h"
#include "frc/DataLogManager.h"
#include "src/camera/camera_constants.h"
#include "src/camera/camera_source.h"
#include "src/camera/disk_camera.h"
#include "src/localization/gpu_apriltag_detector.h"
#include "src/localization/multi_tag_solver.h"
#include "src/localization/networktable_sender.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/simulation_sender.h"
#include "src/utils/camera_utils.h"
#include "src/utils/log.h"

// Example command:
// ./build/src/test/integration_test/localization_test --robot=main_bot --camera_name=right --image_folder=logs/log181/right --speed=0.5
// To each camera has two streams. 1. Raw video stream, 2. Position estimate stream. The port for raw video stream is 580x and the port for position estimate stream is 480x. x is different for each camera. It starts at 1 and counts up. For example, to view the third camera's position estimate stream, go to localhost:4803

ABSL_FLAG(std::string, image_folder, "",  //NOLINT
          "Path to folder of test images");
ABSL_FLAG(std::string, robot, "main_bot",  // NOLINT
          "Robot name used to select robot-specific camera constants");
ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt,  //NOLINT
          "Camera name");
ABSL_FLAG(int, port, 5801, "Port");                   //NOLINT
ABSL_FLAG(double, speed, 1, "Delay between frames");  //NOLINT

namespace {

auto GetRobotCameraConstantsPath(const std::string& robot) -> std::string {
  return "/bos/constants/" + robot + "/camera_constants.json";
}

auto MakeDetector(const camera::camera_constant_t& camera_constant,
                  int image_width, int image_height)
    -> std::unique_ptr<localization::IAprilTagDetector> {
  const auto intrinsics =
      utils::ReadIntrinsics(camera_constant.intrinsics_path.value());
  if (camera_constant.detector_backend == camera::DetectorBackend::kGpu) {
    return std::make_unique<localization::GPUAprilTagDetector>(
        image_width, image_height, intrinsics);
  }
  return std::make_unique<localization::OpenCVAprilTagDetector>(
      image_width, image_height, intrinsics);
}

}  // namespace

auto HasRegularFiles(const std::filesystem::path& path) -> bool {
  for (const auto& entry : std::filesystem::directory_iterator(path)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    std::string extension = entry.path().extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (extension == ".png" || extension == ".jpg" || extension == ".jpeg") {
      return true;
    }
  }
  return false;
}

auto FindCameraFolders(const std::filesystem::path& path)
    -> std::vector<std::filesystem::path> {
  std::vector<std::filesystem::path> camera_folders;
  for (const auto& entry : std::filesystem::directory_iterator(path)) {
    if (entry.is_directory()) {
      camera_folders.push_back(entry.path());
    }
  }

  if (camera_folders.empty()) {
    camera_folders.push_back(path);
  }

  std::sort(camera_folders.begin(), camera_folders.end());
  return camera_folders;
}

auto ResolveCameraName(const std::string& directory_name,
                       const camera::camera_constants_t& constants)
    -> std::string {
  if (constants.contains(directory_name)) {
    return directory_name;
  }

  constexpr std::string_view kPrefixes[] = {"main_bot_", "second_bot_",
                                            "turret_bot_"};
  for (std::string_view prefix : kPrefixes) {
    if (directory_name.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string stripped = directory_name.substr(prefix.size());
    if (constants.contains(stripped)) {
      return stripped;
    }
  }

  if (!constants.contains(directory_name)) {
    LOG(FATAL) << "Could not resolve camera constants name for directory: "
               << directory_name;
  }

  return directory_name;
}

auto main(int argc, char** argv) -> int {

  absl::ParseCommandLine(argc, argv);

  frc::DataLogManager::Start(frc::DataLogManager::GetLogDir());

  const std::string image_folder_root = absl::GetFlag(FLAGS_image_folder);
  const std::filesystem::path image_root_path(image_folder_root);
  if (image_folder_root.empty() || !std::filesystem::exists(image_root_path) ||
      !std::filesystem::is_directory(image_root_path)) {
    LOG(FATAL) << "Folder empty or doesn't exist";
  }

  auto camera_folders = FindCameraFolders(image_root_path);
  auto constants = camera::GetCameraConstants(
      GetRobotCameraConstantsPath(absl::GetFlag(FLAGS_robot)));
  std::optional<std::string> camera_name_override =
      absl::GetFlag(FLAGS_camera_name);
  if (camera_name_override.has_value() && camera_folders.size() > 1) {
    LOG(FATAL) << "--camera_name may only be used with a single camera folder";
  }

  std::vector<std::thread> localization_threads;
  localization_threads.reserve(camera_folders.size());

  const int base_port = absl::GetFlag(FLAGS_port);
  const double speed = absl::GetFlag(FLAGS_speed);

  for (size_t i = 0; i < camera_folders.size(); ++i) {
    const std::filesystem::path& camera_folder = camera_folders[i];
    if (!HasRegularFiles(camera_folder)) {
      LOG(WARNING) << "Skipping empty camera folder: " << camera_folder;
      continue;
    }
    const std::string camera_name =
        camera_name_override.has_value()
            ? camera_name_override.value()
            : ResolveCameraName(camera_folder.filename().string(), constants);

    if (!constants.contains(camera_name)) {
      LOG(FATAL) << "Unknown camera name: " << camera_name;
    }

    localization_threads.emplace_back([camera_name, camera_folder, speed,
                                       constants, base_port, i] {
      const auto& camera_constant = constants.at(camera_name);
      auto camera_source = std::make_unique<camera::CameraSource>(
          camera_name, std::make_unique<camera::DiskCamera>(
                           camera_folder.string(), camera_constant, speed));
      auto frame = camera_source->GetFrame();
      if (frame.empty()) {
        LOG(FATAL) << "No readable images found in folder: " << camera_folder;
      }
      std::vector<std::unique_ptr<localization::IPositionSender>> senders;
      senders.emplace_back(std::make_unique<localization::NetworkTableSender>(
          camera_name, false, true));
      senders.emplace_back(std::make_unique<localization::SimulationSender>(
          camera_name, base_port + i - 1000));
      localization::RunLocalization(
          std::move(camera_source), MakeDetector(camera_constant, frame.cols,
                                                 frame.rows),
          std::make_unique<localization::MultiTagSolver>(camera_constant),
          std::move(senders), camera_constant.extrinsics_path.value(),
          base_port + i, true);
    });
  }

  if (localization_threads.empty()) {
    LOG(FATAL) << "No readable images found in any camera folder under: "
               << image_folder_root;
  }

  for (auto& thread : localization_threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}
