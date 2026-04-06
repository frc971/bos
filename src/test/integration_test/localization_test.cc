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
#include "src/localization/multi_tag_solver.h"
#include "src/localization/networktable_sender.h"
#include "src/localization/opencv_apriltag_detector.h"
#include "src/localization/run_localization.h"
#include "src/localization/simulation_sender.h"
#include "src/utils/camera_utils.h"
#include "src/utils/log.h"

// for reference, example command:
// ./build/src/test/integration_test/localization_test --camera_name=main_bot_right --image_folder=logs/log181/right --speed=0.5

ABSL_FLAG(std::string, image_folder, "",  //NOLINT
          "Path to folder of test images");
ABSL_FLAG(std::optional<std::string>, camera_name, std::nullopt,  //NOLINT
          "Camera name");
ABSL_FLAG(int, port, 5801, "Port");                   //NOLINT
ABSL_FLAG(double, speed, 1, "Delay between frames");  //NOLINT

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
  std::string resolved_name = directory_name.rfind("main_bot_", 0) == 0
                                  ? directory_name
                                  : "main_bot_" + directory_name;

  if (!constants.contains(resolved_name)) {
    LOG(FATAL) << "Could not resolve camera constants name for directory: "
               << directory_name
               << ", expected constants entry: " << resolved_name;
  }

  return resolved_name;
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
  auto constants = camera::GetCameraConstants();
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
      auto camera_source = std::make_unique<camera::CameraSource>(
          camera_name, std::make_unique<camera::DiskCamera>(
                           camera_folder.string(), std::nullopt, speed));
      auto frame = camera_source->GetFrame();
      if (frame.empty()) {
        LOG(FATAL) << "No readable images found in folder: " << camera_folder;
      }
      const auto& camera_constant = constants.at(camera_name);
      std::vector<std::unique_ptr<localization::IPositionSender>> senders;
      senders.emplace_back(std::make_unique<localization::NetworkTableSender>(
          camera_name, true));
      senders.emplace_back(std::make_unique<localization::SimulationSender>(
          camera_name, base_port + i - 1000));
      localization::RunLocalization(
          std::move(camera_source),
          std::make_unique<localization::OpenCVAprilTagDetector>(
              frame.cols, frame.rows,
              utils::ReadIntrinsics(camera_constant.intrinsics_path.value())),
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
